#include "bsp.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/soc.h"
#include "lvgl.h"

#include "app_config.h"
#include "spi_psram_dma_wrap.h"

static const char *TAG = "bsp_lcd";

/* LR2021 uses SPI2 inside the module; keep the external LCD on SPI3. */
#define BSP_LCD_SPI_HOST SPI3_HOST
#define LCD_DMA_CACHE_ALIGNMENT 64U
#define LCD_PIXEL_DUTY_CYCLE_POS APP_LCD_SPI_PIXEL_DUTY_CYCLE_POS
#define LCD_PSRAM_STAGE_BUFFER_COUNT 2U
#define LCD_PSRAM_STAGE_BYTES 1024U

/* Optional frame checksum before and after a PSRAM-backed panel transfer. */
#define LCD_VIDEO_INTEGRITY_CHECK 0

/* Optional internal-RAM staging path for isolating PSRAM DMA faults. */
#define LCD_VIDEO_FORCE_PSRAM_STAGING 0

static esp_lcd_panel_io_handle_t s_lcd_io;
static spi_device_handle_t s_lcd_pixel_spi;
static esp_lcd_panel_handle_t s_lcd_panel;
static bool s_lcd_bus_ready;
static bool s_lcd_ready;
static bool s_lvgl_started;
static bool s_lcd_suspended;
static volatile bool s_video_direct_owns_panel;
static bool s_video_status_refresh;
/* Panel rows owned by direct video presentation. */
static volatile uint32_t s_video_region_y0 = 0;
static i2c_master_dev_handle_t s_touch;
static uint8_t s_touch_addr;
static lv_disp_drv_t *s_lvgl_disp_drv;
static SemaphoreHandle_t s_lvgl_lock;
static SemaphoreHandle_t s_lcd_color_idle;
static SemaphoreHandle_t s_video_frame_done;

typedef enum {
    LCD_TRANSFER_OWNER_NONE = 0,
    LCD_TRANSFER_OWNER_LVGL,
    LCD_TRANSFER_OWNER_VIDEO,
    LCD_TRANSFER_OWNER_OTHER,
} lcd_transfer_owner_t;

static volatile lcd_transfer_owner_t s_lcd_transfer_owner;
static volatile uint32_t s_lcd_transfer_owner_sequence;
static volatile uint32_t s_lcd_owner_mismatch_count;
static volatile bool s_video_frame_inflight;
static volatile uint32_t s_video_frame_sequence;
static volatile uint32_t s_video_frame_done_sequence;
static volatile int64_t s_video_frame_isr_done_us;
static bool s_video_path_logged;
static bool s_split_draw_logged;
static size_t s_lcd_pixel_max_transfer_bytes;
static size_t s_lcd_pixel_inflight;
static uint32_t s_lcd_pixel_dma_error_count;
static spi_transaction_t s_lcd_pixel_transactions[APP_LCD_SPI_QUEUE_DEPTH];
static DMA_ATTR uint8_t
    s_lcd_psram_stage[LCD_PSRAM_STAGE_BUFFER_COUNT][LCD_PSRAM_STAGE_BYTES]
        __attribute__((aligned(LCD_DMA_CACHE_ALIGNMENT)));
static lv_obj_t *s_camera_status_label;
static lv_obj_t *s_camera_canvas;
static lv_color_t *s_camera_canvas_buf;
static bsp_lcd_capture_cb_t s_capture_cb;
static void *s_capture_user;

typedef struct {
    uint32_t sequence;
    uint32_t period_us;
    uint32_t total_us;
    uint32_t flush_count;
    uint32_t pixel_count;
    uint32_t swap_us;
    uint32_t submit_us;
    uint32_t xfer_us;
} lcd_refresh_stats_t;

typedef struct {
    int64_t submit_start_us;
    bool is_last;
} lcd_flush_pending_t;

#define LCD_STATS_PENDING_MAX 16U

static uint32_t lcd_spi_actual_clock_hz(uint32_t requested_hz,
                                         uint32_t duty_cycle_pos)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const int actual_hz = spi_get_actual_clock(
        APB_CLK_FREQ, requested_hz, duty_cycle_pos);
#pragma GCC diagnostic pop
    return actual_hz > 0 ? (uint32_t)actual_hz : 0U;
}

static void *lcd_alloc_internal_dma(size_t bytes)
{
    return heap_caps_aligned_alloc(
        LCD_DMA_CACHE_ALIGNMENT, bytes,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
}

static bool lcd_pixel_needs_staging(const void *color)
{
    const bool aligned =
        ((uintptr_t)color & (LCD_DMA_CACHE_ALIGNMENT - 1U)) == 0U;
    if (esp_ptr_external_ram(color)) {
        if (LCD_VIDEO_FORCE_PSRAM_STAGING) {
            return true;
        }
        return !esp_ptr_dma_ext_capable(color) || !aligned;
    }
    return !esp_ptr_dma_capable(color) || !aligned;
}

#if LCD_VIDEO_INTEGRITY_CHECK
/* Split per SPI chunk so a mismatch points at a screen band: segment i covers
 * canvas rows [i*40, (i+1)*40), which is exactly the i-th 19200-byte transfer.
 * One sample per cache line, so no single bad line can hide. Reading every
 * line also means the "after" pass refills the whole buffer from PSRAM once it
 * has been invalidated. */
#define LCD_INTEGRITY_SEGMENTS 8

static void lcd_video_fingerprint(const uint16_t *pixels, size_t count,
                                  uint32_t out[LCD_INTEGRITY_SEGMENTS])
{
    const size_t step = LCD_DMA_CACHE_ALIGNMENT / sizeof(uint16_t);
    const size_t seg_pixels = count / LCD_INTEGRITY_SEGMENTS;

    for (size_t seg = 0; seg < LCD_INTEGRITY_SEGMENTS; ++seg) {
        const size_t begin = seg * seg_pixels;
        const size_t end = begin + seg_pixels;
        uint32_t hash = 2166136261u;
        for (size_t i = begin; i < end; i += step) {
            hash = (hash ^ pixels[i]) * 16777619u;
        }
        out[seg] = hash;
    }
}
#endif

static int lcd_gpio_drive_capability(gpio_num_t gpio)
{
    gpio_drive_cap_t capability;
    return gpio_get_drive_capability(gpio, &capability) == ESP_OK
               ? (int)capability
               : -1;
}

static inline void lcd_cs_select(void)
{
    gpio_set_level(BSP_LCD_SPI_CS_GPIO, 0);
}

static inline void lcd_cs_deselect(void)
{
    gpio_set_level(BSP_LCD_SPI_CS_GPIO, 1);
}

static portMUX_TYPE s_lcd_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_lcd_refresh_start_us;
static int64_t s_lcd_last_refresh_done_us;
static int64_t s_lcd_last_xfer_done_us;
static uint32_t s_lcd_refresh_flush_count;
static uint32_t s_lcd_refresh_pixel_count;
static uint32_t s_lcd_refresh_swap_us;
static uint32_t s_lcd_refresh_submit_us;
static uint32_t s_lcd_refresh_xfer_us;
static lcd_flush_pending_t s_lcd_pending[LCD_STATS_PENDING_MAX];
static uint32_t s_lcd_pending_head;
static uint32_t s_lcd_pending_tail;
static uint32_t s_lcd_pending_count;
static lcd_refresh_stats_t s_lcd_refresh_ready;

typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t len;
    uint16_t delay_ms;
} lcd_init_cmd_t;

static esp_err_t lcd_tx_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(s_lcd_io, ESP_ERR_INVALID_STATE, TAG, "lcd io not ready");
    lcd_cs_select();
    esp_err_t ret = esp_lcd_panel_io_tx_param(s_lcd_io, cmd, data, len);
    lcd_cs_deselect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd_tx_cmd(0x%02X) FAILED: %s", cmd, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t lcd_panel_init_low_speed(void)
{
    static const uint8_t madctl = LCD_CMD_BGR_BIT;
    static const uint8_t colmod = 0x55;
    static const uint8_t ramctrl[] = {0x00, 0xf8};

    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_SWRESET, NULL, 0),
                        TAG, "low-speed SWRESET");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_SLPOUT, NULL, 0),
                        TAG, "low-speed SLPOUT");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_MADCTL, &madctl, 1),
                        TAG, "low-speed MADCTL");
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_COLMOD, &colmod, 1),
                        TAG, "low-speed COLMOD");
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(0xb0, ramctrl, sizeof(ramctrl)),
                        TAG, "low-speed RAMCTRL");
    return ESP_OK;
}

static esp_err_t touch_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_touch) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_touch, &reg, 1, data, len, 20);
}

static esp_err_t lcd_send_vendor_init(void)
{
    static const uint8_t madctl[] = {0x08};
    static const uint8_t colmod[] = {0x05};
    static const uint8_t porch[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    static const uint8_t gate[] = {0x35};
    static const uint8_t vcom[] = {0x36};
    static const uint8_t vdv_vrh_en[] = {0x01};
    static const uint8_t vrh[] = {0x13};
    static const uint8_t vdv[] = {0x20};
    static const uint8_t frame_rate[] = {0x0f};
    static const uint8_t gate_ctrl[] = {0xa1};
    static const uint8_t power[] = {0xa4, 0xa1};
    static const uint8_t gamma_pos[] = {
        0xf0, 0x08, 0x0e, 0x09, 0x08, 0x04, 0x2f,
        0x33, 0x45, 0x36, 0x13, 0x12, 0x2a, 0x2d,
    };
    static const uint8_t gamma_neg[] = {
        0xf0, 0x0e, 0x12, 0x0c, 0x0a, 0x15, 0x2e,
        0x32, 0x44, 0x39, 0x17, 0x18, 0x2b, 0x2f,
    };

    static const lcd_init_cmd_t cmds[] = {
        {LCD_CMD_MADCTL, madctl, sizeof(madctl), 0},
        {LCD_CMD_COLMOD, colmod, sizeof(colmod), 0},
        {0xb2, porch, sizeof(porch), 0},
        {0xb7, gate, sizeof(gate), 0},
        {0xbb, vcom, sizeof(vcom), 0},
        {0xc2, vdv_vrh_en, sizeof(vdv_vrh_en), 0},
        {0xc3, vrh, sizeof(vrh), 0},
        {0xc4, vdv, sizeof(vdv), 0},
        {0xc6, frame_rate, sizeof(frame_rate), 0},
        {0xd6, gate_ctrl, sizeof(gate_ctrl), 0},
        {0xd0, power, sizeof(power), 0},
        {0xe0, gamma_pos, sizeof(gamma_pos), 0},
        {0xe1, gamma_neg, sizeof(gamma_neg), 0},
        {LCD_CMD_COLMOD, colmod, sizeof(colmod), 0},
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); ++i) {
        ESP_RETURN_ON_ERROR(lcd_tx_cmd(cmds[i].cmd, cmds[i].data, cmds[i].len),
                            TAG, "lcd cmd 0x%02x", cmds[i].cmd);
        if (cmds[i].delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }

    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_SLPOUT, NULL, 0), TAG, "SLPOUT");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_DISPON, NULL, 0), TAG, "DISPON");
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}

static esp_err_t lcd_reset_gpio(void)
{
    esp_err_t err = bsp_ioexp_set_pin(BSP_IO_EXP_LCD_RST_PIN, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "lcd reset via IO expander unavailable: %s",
                 esp_err_to_name(err));
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(bsp_ioexp_set_pin(BSP_IO_EXP_LCD_RST_PIN, true),
                        TAG, "lcd reset high");
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(APP_LCD_LVGL_TICK_MS);
}

static esp_err_t lcd_pixel_take_completed(spi_transaction_t **completed_out,
                                          TickType_t wait_ticks)
{
    spi_transaction_t *completed = NULL;
    esp_err_t ret = spi_device_get_trans_result(
        s_lcd_pixel_spi, &completed, wait_ticks);

    /* ESP-IDF returns the completed descriptor together with
     * ESP_ERR_INVALID_STATE when PSRAM DMA reports RX/TX data loss. The
     * descriptor has already left the return queue in that case, so its
     * inflight slot must still be retired before the error is propagated. */
    if (completed != NULL && s_lcd_pixel_inflight > 0) {
        s_lcd_pixel_inflight--;
    }

    if (ret != ESP_OK) {
        const uint32_t flags = completed ? completed->flags : 0U;
        const uint32_t error_count = ++s_lcd_pixel_dma_error_count;
        ESP_LOGE(TAG,
                 "[LCD-SPI] completion error count=%lu err=%s "
                 "flags=0x%08lx tx_underflow=%d rx_overflow=%d "
                 "remaining=%u",
                 (unsigned long)error_count, esp_err_to_name(ret),
                 (unsigned long)flags,
                 (flags & SPI_TRANS_DMA_TX_FAIL) ? 1 : 0,
                 (flags & SPI_TRANS_DMA_RX_FAIL) ? 1 : 0,
                 (unsigned)s_lcd_pixel_inflight);
    }

    if (completed_out) {
        *completed_out = completed;
    }
    return ret;
}

static esp_err_t lcd_pixel_recycle_completed(void)
{
    esp_err_t first_err = ESP_OK;
    while (s_lcd_pixel_inflight > 0) {
        spi_transaction_t *completed = NULL;
        esp_err_t ret = lcd_pixel_take_completed(
            &completed, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK && first_err == ESP_OK) {
            first_err = ret;
        }

        /* A timeout or API failure without a returned descriptor means there
         * is nothing safe to retire in this pass. */
        if (completed == NULL) {
            break;
        }
    }
    return first_err;
}

static esp_err_t lcd_pixel_queue_staged(const uint8_t *color,
                                        size_t color_size)
{
    size_t next_offset = 0;

    for (size_t slot = 0;
         slot < LCD_PSRAM_STAGE_BUFFER_COUNT && next_offset < color_size;
         ++slot) {
        const size_t chunk_size =
            color_size - next_offset > LCD_PSRAM_STAGE_BYTES
                ? LCD_PSRAM_STAGE_BYTES
                : color_size - next_offset;
        memcpy(s_lcd_psram_stage[slot], color + next_offset, chunk_size);

        spi_transaction_t *trans = &s_lcd_pixel_transactions[slot];
        memset(trans, 0, sizeof(*trans));
        trans->length = chunk_size * 8U;
        trans->tx_buffer = s_lcd_psram_stage[slot];
        next_offset += chunk_size;
        trans->user = next_offset == color_size ? (void *)1 : NULL;

        esp_err_t ret = spi_device_queue_trans(
            s_lcd_pixel_spi, trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            return ret;
        }
        s_lcd_pixel_inflight++;
    }

    while (next_offset < color_size) {
        spi_transaction_t *completed = NULL;
        esp_err_t ret = lcd_pixel_take_completed(&completed, portMAX_DELAY);
        if (ret != ESP_OK) {
            return ret;
        }
        size_t slot = LCD_PSRAM_STAGE_BUFFER_COUNT;
        for (size_t i = 0; i < LCD_PSRAM_STAGE_BUFFER_COUNT; ++i) {
            if (completed == &s_lcd_pixel_transactions[i]) {
                slot = i;
                break;
            }
        }
        ESP_RETURN_ON_FALSE(slot < LCD_PSRAM_STAGE_BUFFER_COUNT,
                            ESP_ERR_INVALID_STATE, TAG,
                            "unexpected staged pixel descriptor");
        const size_t chunk_size =
            color_size - next_offset > LCD_PSRAM_STAGE_BYTES
                ? LCD_PSRAM_STAGE_BYTES
                : color_size - next_offset;
        memcpy(s_lcd_psram_stage[slot], color + next_offset, chunk_size);

        memset(completed, 0, sizeof(*completed));
        completed->length = chunk_size * 8U;
        completed->tx_buffer = s_lcd_psram_stage[slot];
        next_offset += chunk_size;
        completed->user = next_offset == color_size ? (void *)1 : NULL;

        ret = spi_device_queue_trans(
            s_lcd_pixel_spi, completed, portMAX_DELAY);
        if (ret != ESP_OK) {
            return ret;
        }
        s_lcd_pixel_inflight++;
    }

    return ESP_OK;
}

static esp_err_t lcd_pixel_tx_color(const void *color, size_t color_size)
{
    ESP_RETURN_ON_FALSE(s_lcd_pixel_spi && color && color_size > 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid pixel transfer");
    ESP_RETURN_ON_FALSE(s_lcd_pixel_max_transfer_bytes > 0,
                        ESP_ERR_INVALID_STATE, TAG,
                        "pixel max transfer size unavailable");

    const bool staged_source = lcd_pixel_needs_staging(color);
    const size_t chunk_count = staged_source
                                   ? 0U
                                   : (color_size +
                                      s_lcd_pixel_max_transfer_bytes - 1U) /
                                         s_lcd_pixel_max_transfer_bytes;
    ESP_RETURN_ON_FALSE(staged_source ||
                            chunk_count <= APP_LCD_SPI_QUEUE_DEPTH,
                        ESP_ERR_INVALID_SIZE, TAG,
                        "pixel transfer needs %u chunks, queue depth is %u",
                        (unsigned)chunk_count,
                        (unsigned)APP_LCD_SPI_QUEUE_DEPTH);

    ESP_RETURN_ON_ERROR(
        spi_device_acquire_bus(s_lcd_pixel_spi, portMAX_DELAY),
        TAG, "acquire pixel SPI device");

    esp_err_t ret = lcd_pixel_recycle_completed();
    if (ret == ESP_OK) {
        gpio_ll_set_level(&GPIO, BSP_LCD_SPI_DC_GPIO, 1);
        gpio_ll_output_enable(&GPIO, BSP_LCD_SPI_DC_GPIO);

        if (staged_source) {
            ret = lcd_pixel_queue_staged(color, color_size);
        } else {
            const uint8_t *chunk = color;
            size_t remaining = color_size;
            for (size_t i = 0; i < chunk_count; ++i) {
                const size_t chunk_size =
                    remaining > s_lcd_pixel_max_transfer_bytes
                        ? s_lcd_pixel_max_transfer_bytes
                        : remaining;
                spi_transaction_t *trans = &s_lcd_pixel_transactions[i];
                memset(trans, 0, sizeof(*trans));
                trans->length = chunk_size * 8U;
                trans->tx_buffer = chunk;
                trans->user = (i + 1U == chunk_count) ? (void *)1 : NULL;

                ret = spi_device_queue_trans(
                    s_lcd_pixel_spi, trans, portMAX_DELAY);
                if (ret != ESP_OK) {
                    break;
                }
                s_lcd_pixel_inflight++;
                chunk += chunk_size;
                remaining -= chunk_size;
            }
        }
    }

    spi_device_release_bus(s_lcd_pixel_spi);
    if (ret != ESP_OK && s_lcd_pixel_inflight > 0) {
        esp_err_t drain_ret = lcd_pixel_recycle_completed();
        if (drain_ret != ESP_OK) {
            ESP_LOGE(TAG, "drain failed pixel transfer: %s",
                     esp_err_to_name(drain_ret));
        }
    }
    return ret;
}

static esp_err_t lcd_draw_rgb565_bitmap(uint32_t x0, uint32_t y0,
                                        uint32_t x1, uint32_t y1,
                                        const uint16_t *pixels,
                                        lcd_transfer_owner_t owner,
                                        uint32_t owner_sequence)
{
    ESP_RETURN_ON_FALSE(s_lcd_ready && s_lcd_io && s_lcd_pixel_spi && pixels,
                        ESP_ERR_INVALID_STATE, TAG, "lcd not ready");
    ESP_RETURN_ON_FALSE(x1 > x0 && y1 > y0 &&
                        x1 <= APP_LCD_H_RES && y1 <= APP_LCD_V_RES,
                        ESP_ERR_INVALID_ARG, TAG, "invalid draw area");
    ESP_RETURN_ON_FALSE(s_lcd_color_idle, ESP_ERR_INVALID_STATE, TAG,
                        "lcd color semaphore not ready");

    if (xSemaphoreTake(s_lcd_color_idle, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "previous LCD pixel transfer did not finish");
        return ESP_ERR_TIMEOUT;
    }

    lcd_transfer_owner_t stale_owner;
    uint32_t stale_sequence;
    portENTER_CRITICAL(&s_lcd_stats_mux);
    stale_owner = s_lcd_transfer_owner;
    stale_sequence = s_lcd_transfer_owner_sequence;
    s_lcd_transfer_owner = owner;
    s_lcd_transfer_owner_sequence = owner_sequence;
    portEXIT_CRITICAL(&s_lcd_stats_mux);
    if (stale_owner != LCD_TRANSFER_OWNER_NONE) {
        ESP_LOGW(TAG,
                 "[LCD-OWNER] stale owner=%d seq=%lu replaced_by=%d seq=%lu",
                 (int)stale_owner, (unsigned long)stale_sequence,
                 (int)owner, (unsigned long)owner_sequence);
    }

    const uint32_t x_start = x0 + APP_LCD_X_GAP;
    const uint32_t x_end = x1 + APP_LCD_X_GAP;
    const uint32_t y_start = y0 + APP_LCD_Y_GAP;
    const uint32_t y_end = y1 + APP_LCD_Y_GAP;
    const uint8_t caset[] = {
        (uint8_t)(x_start >> 8), (uint8_t)x_start,
        (uint8_t)((x_end - 1U) >> 8), (uint8_t)(x_end - 1U),
    };
    const uint8_t raset[] = {
        (uint8_t)(y_start >> 8), (uint8_t)y_start,
        (uint8_t)((y_end - 1U) >> 8), (uint8_t)(y_end - 1U),
    };
    const size_t color_bytes =
        (size_t)(x1 - x0) * (size_t)(y1 - y0) * sizeof(uint16_t);

    if (!s_split_draw_logged) {
        ESP_LOGI(TAG,
                 "[LCD-SPI] first split draw: window=%lux%lu bytes=%u "
                 "commands=%uHz pixels=%uHz pixel_duty=%u/256",
                 (unsigned long)(x1 - x0), (unsigned long)(y1 - y0),
                 (unsigned)color_bytes, (unsigned)APP_LCD_SPI_CMD_PCLK_HZ,
                 (unsigned)APP_LCD_SPI_PCLK_HZ,
                 (unsigned)LCD_PIXEL_DUTY_CYCLE_POS);
        s_split_draw_logged = true;
    }

    lcd_cs_select();
    esp_err_t ret = esp_lcd_panel_io_tx_param(s_lcd_io, LCD_CMD_CASET,
                                               caset, sizeof(caset));
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_io_tx_param(s_lcd_io, LCD_CMD_RASET,
                                        raset, sizeof(raset));
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_io_tx_param(s_lcd_io, LCD_CMD_RAMWR, NULL, 0);
    }
    if (ret == ESP_OK) {
        ret = lcd_pixel_tx_color(pixels, color_bytes);
    }
    if (ret != ESP_OK) {
        portENTER_CRITICAL(&s_lcd_stats_mux);
        if (s_lcd_transfer_owner == owner &&
            s_lcd_transfer_owner_sequence == owner_sequence) {
            s_lcd_transfer_owner = LCD_TRANSFER_OWNER_NONE;
            s_lcd_transfer_owner_sequence = 0;
        }
        portEXIT_CRITICAL(&s_lcd_stats_mux);
        lcd_cs_deselect();
        xSemaphoreGive(s_lcd_color_idle);
        ESP_LOGE(TAG, "split-speed LCD draw failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static bool lvgl_flush_ready_cb(esp_lcd_panel_io_handle_t panel_io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    (void)user_ctx;
    BaseType_t task_awoken = pdFALSE;

    /* The pixel transaction owns CS until its ISR completion. */
    gpio_ll_set_level(&GPIO, BSP_LCD_SPI_CS_GPIO, 1);

    const int64_t done_us = esp_timer_get_time();
    bool lvgl_flush_done = false;
    bool video_frame_done = false;
    lcd_transfer_owner_t owner;
    uint32_t owner_sequence;

    /* Consume the owner before waking a task on the other core. Otherwise the
     * next submitter can overwrite the global state while this ISR still uses
     * it to decide which completion object to signal. */
    portENTER_CRITICAL_ISR(&s_lcd_stats_mux);
    owner = s_lcd_transfer_owner;
    owner_sequence = s_lcd_transfer_owner_sequence;
    s_lcd_transfer_owner = LCD_TRANSFER_OWNER_NONE;
    s_lcd_transfer_owner_sequence = 0;

    if (owner == LCD_TRANSFER_OWNER_LVGL) {
        if (s_lcd_pending_count > 0) {
            const lcd_flush_pending_t pending = s_lcd_pending[s_lcd_pending_head];
            s_lcd_pending_head = (s_lcd_pending_head + 1U) % LCD_STATS_PENDING_MAX;
            s_lcd_pending_count--;

            int64_t xfer_start_us = pending.submit_start_us;
            if (s_lcd_last_xfer_done_us > xfer_start_us) {
                xfer_start_us = s_lcd_last_xfer_done_us;
            }
            if (done_us >= xfer_start_us) {
                s_lcd_refresh_xfer_us += (uint32_t)(done_us - xfer_start_us);
            }
            s_lcd_last_xfer_done_us = done_us;

            if (pending.is_last && s_lcd_refresh_flush_count > 0) {
                s_lcd_refresh_ready.sequence++;
                s_lcd_refresh_ready.period_us = s_lcd_last_refresh_done_us > 0
                                                    ? (uint32_t)(done_us - s_lcd_last_refresh_done_us)
                                                    : 0;
                s_lcd_refresh_ready.total_us = (uint32_t)(done_us - s_lcd_refresh_start_us);
                s_lcd_refresh_ready.flush_count = s_lcd_refresh_flush_count;
                s_lcd_refresh_ready.pixel_count = s_lcd_refresh_pixel_count;
                s_lcd_refresh_ready.swap_us = s_lcd_refresh_swap_us;
                s_lcd_refresh_ready.submit_us = s_lcd_refresh_submit_us;
                s_lcd_refresh_ready.xfer_us = s_lcd_refresh_xfer_us;
                s_lcd_last_refresh_done_us = done_us;
                s_lcd_refresh_flush_count = 0;
                s_lcd_refresh_pixel_count = 0;
                s_lcd_refresh_swap_us = 0;
                s_lcd_refresh_submit_us = 0;
                s_lcd_refresh_xfer_us = 0;
            }
        } else {
            s_lcd_owner_mismatch_count++;
        }
        /* Owner is authoritative even if optional statistics bookkeeping was
         * inconsistent; never leave LVGL waiting forever for flush_ready. */
        lvgl_flush_done = true;
    } else if (owner == LCD_TRANSFER_OWNER_VIDEO) {
        if (!s_video_frame_inflight ||
            owner_sequence != s_video_frame_sequence) {
            s_lcd_owner_mismatch_count++;
        }
        s_video_frame_isr_done_us = done_us;
        s_video_frame_done_sequence = owner_sequence;
        s_video_frame_inflight = false;
        video_frame_done = true;
    }
    portEXIT_CRITICAL_ISR(&s_lcd_stats_mux);

    if (video_frame_done && s_video_frame_done) {
        xSemaphoreGiveFromISR(s_video_frame_done, &task_awoken);
    }
    if (lvgl_flush_done && s_lvgl_disp_drv) {
        lv_disp_flush_ready(s_lvgl_disp_drv);
    }
    /* Release the next submitter only after this ISR has consumed and cleared
     * the explicit owner. This makes the cross-core handoff deterministic. */
    if (s_lcd_color_idle) {
        xSemaphoreGiveFromISR(s_lcd_color_idle, &task_awoken);
    }
    return task_awoken == pdTRUE;
}

static void lcd_pixel_post_trans_cb(spi_transaction_t *trans)
{
    if (trans && trans->user == (void *)1) {
        (void)lvgl_flush_ready_cb(NULL, NULL, NULL);
    }
}

static esp_err_t lvgl_register_flush_ready_cb(lv_disp_drv_t *drv)
{
    ESP_RETURN_ON_FALSE(s_lcd_pixel_spi, ESP_ERR_INVALID_STATE, TAG,
                        "lcd pixel SPI device not ready");
    s_lvgl_disp_drv = drv;
    return ESP_OK;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                          lv_color_t *color_map)
{
    if (s_lcd_suspended || !s_lcd_ready) {
        lv_disp_flush_ready(drv);
        return;
    }
    /* Suppress autonomous LVGL transfers while video owns the panel. */
    lv_area_t clipped = *area;
    if (s_video_direct_owns_panel) {
        /* Only the frame-completion path may send LVGL pixels during video.
         * Keep the top part of a mixed chunk (e.g. rows 0..39), rather than
         * discarding the status strip along with the video rows. */
        if (!s_video_status_refresh || area->y1 >= (int32_t)s_video_region_y0) {
            lv_disp_flush_ready(drv);
            return;
        }
        if (clipped.y2 >= (int32_t)s_video_region_y0) {
            clipped.y2 = s_video_region_y0 - 1U;
        }
        area = &clipped;
    }
    const int64_t flush_start_us = esp_timer_get_time();
    uint32_t pixel_count = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    const bool is_last = lv_disp_flush_is_last(drv);

    portENTER_CRITICAL(&s_lcd_stats_mux);
    if (s_lcd_refresh_flush_count == 0) {
        s_lcd_refresh_start_us = flush_start_us;
    }
    s_lcd_refresh_flush_count++;
    s_lcd_refresh_pixel_count += pixel_count;
    portEXIT_CRITICAL(&s_lcd_stats_mux);

    const int64_t swap_done_us = esp_timer_get_time();
    const int64_t submit_start_us = swap_done_us;

    portENTER_CRITICAL(&s_lcd_stats_mux);
    s_lcd_refresh_swap_us += (uint32_t)(swap_done_us - flush_start_us);
    s_lcd_pending[s_lcd_pending_tail].submit_start_us = submit_start_us;
    s_lcd_pending[s_lcd_pending_tail].is_last = is_last;
    s_lcd_pending_tail = (s_lcd_pending_tail + 1U) % LCD_STATS_PENDING_MAX;
    s_lcd_pending_count++;
    portEXIT_CRITICAL(&s_lcd_stats_mux);

    esp_err_t err = lcd_draw_rgb565_bitmap(area->x1, area->y1,
                                           area->x2 + 1, area->y2 + 1,
                                           (const uint16_t *)color_map,
                                           LCD_TRANSFER_OWNER_LVGL, 0);
    const int64_t submit_done_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_lcd_stats_mux);
    s_lcd_refresh_submit_us += (uint32_t)(submit_done_us - submit_start_us);
    portEXIT_CRITICAL(&s_lcd_stats_mux);

    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_lcd_stats_mux);
        s_lcd_refresh_flush_count = 0;
        s_lcd_refresh_pixel_count = 0;
        s_lcd_refresh_swap_us = 0;
        s_lcd_refresh_submit_us = 0;
        s_lcd_refresh_xfer_us = 0;
        if (s_lcd_pending_count > 0) {
            s_lcd_pending_tail = (s_lcd_pending_tail + LCD_STATS_PENDING_MAX - 1U) %
                                 LCD_STATS_PENDING_MAX;
            s_lcd_pending_count--;
        }
        portEXIT_CRITICAL(&s_lcd_stats_mux);
        ESP_LOGE(TAG, "lvgl flush failed: %s", esp_err_to_name(err));
        lv_disp_flush_ready(drv);
        return;
    }
}

static bool lcd_refresh_stats_take(uint32_t *last_sequence,
                                   lcd_refresh_stats_t *stats)
{
    bool updated = false;
    portENTER_CRITICAL(&s_lcd_stats_mux);
    if (s_lcd_refresh_ready.sequence != *last_sequence) {
        *stats = s_lcd_refresh_ready;
        *last_sequence = stats->sequence;
        updated = true;
    }
    portEXIT_CRITICAL(&s_lcd_stats_mux);
    return updated;
}

static esp_err_t touch_reset(void)
{
    gpio_config_t intr = {
        .pin_bit_mask = 1ULL << BSP_LCD_TOUCH_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&intr), TAG, "touch int gpio");

    esp_err_t err = bsp_ioexp_set_pin(BSP_IO_EXP_TP_RST_PIN, false);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(bsp_ioexp_set_pin(BSP_IO_EXP_TP_RST_PIN, true),
                            TAG, "touch reset high");
        vTaskDelay(pdMS_TO_TICKS(80));
    } else {
        ESP_LOGW(TAG, "touch reset via IO expander unavailable: %s",
                 esp_err_to_name(err));
    }
    return ESP_OK;
}

static esp_err_t touch_attach(void)
{
    if (s_touch) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(touch_reset(), TAG, "touch reset");
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
        bus = bsp_i2c_bus();
    }
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }

    static const uint8_t candidates[] = {
        BSP_I2C_ADDR_TOUCH_FT6206,
        BSP_I2C_ADDR_TOUCH_CST816,
        BSP_I2C_ADDR_TOUCH_CST816_ALT,
    };
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        uint8_t addr = candidates[i];
        esp_err_t pe = i2c_master_probe(bus, addr, 50);
        if (pe != ESP_OK) {
            ESP_LOGI(TAG, "touch addr 0x%02X not present (%s)",
                     addr, esp_err_to_name(pe));
            continue;
        }

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = BSP_I2C0_FREQ_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_touch),
                            TAG, "touch add");
        s_touch_addr = addr;
        ESP_LOGI(TAG, "touch controller selected at 0x%02X (FT6206/CST816-compatible)",
                 s_touch_addr);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "no supported touch controller found on I2C");
    return ESP_ERR_NOT_FOUND;
}

static bool touch_read_point(lv_coord_t *x, lv_coord_t *y)
{
    uint8_t buf[5] = {};
    if (touch_read_reg(0x02, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    uint8_t points = buf[0] & 0x0f;
    if (points == 0 || points > 2) {
        return false;
    }

    uint16_t raw_x = ((uint16_t)(buf[1] & 0x0f) << 8) | buf[2];
    uint16_t raw_y = ((uint16_t)(buf[3] & 0x0f) << 8) | buf[4];
    int32_t adj_x = (int32_t)raw_x - APP_LCD_X_GAP;
    int32_t adj_y = (int32_t)raw_y - APP_LCD_Y_GAP;
    if (adj_x < 0 || adj_y < 0 ||
        adj_x >= APP_LCD_H_RES || adj_y >= APP_LCD_V_RES) {
        return false;
    }

    *x = (lv_coord_t)adj_x;
    *y = (lv_coord_t)adj_y;
    return true;
}

static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    static lv_coord_t last_x;
    static lv_coord_t last_y;

    if (s_touch && touch_read_point(&last_x, &last_y)) {
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->point.x = last_x;
    data->point.y = last_y;
}

static void lvgl_create_demo_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "AM36-DoorCam");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    lv_obj_t *subtitle = lv_label_create(scr);
    lv_label_set_text(subtitle, "ST7789T3 + LVGL + Touch");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xa7b0be), 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 42);

    lv_obj_t *bar = lv_bar_create(scr);
    lv_obj_set_size(bar, 190, 16);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 76);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, 72, LV_ANIM_OFF);

    lv_obj_t *slider = lv_slider_create(scr);
    lv_obj_set_width(slider, 190);
    lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 118);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 35, LV_ANIM_OFF);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 120, 42);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 158);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Touch");
    lv_obj_center(btn_label);

    lv_obj_t *status = lv_label_create(scr);
    lv_label_set_text(status, s_touch ? "Touch: ready" : "Touch: not found");
    lv_obj_set_style_text_color(status, lv_color_hex(0xd9e6f2), 0);
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -18);
}

static void camera_btn_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_capture_cb) {
        s_capture_cb(s_capture_user);
    }
}

static esp_err_t lvgl_create_camera_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);

    s_camera_status_label = lv_label_create(scr);
    lv_label_set_text(s_camera_status_label, s_touch ? "Touch capture to take a photo" : "Touch not found");
    lv_obj_set_style_text_color(s_camera_status_label, lv_color_hex(0xd9e6f2), 0);
    lv_obj_set_style_bg_color(s_camera_status_label, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(s_camera_status_label, LV_OPA_COVER, 0);
    lv_obj_set_size(s_camera_status_label, APP_LCD_H_RES - 16, 20);
    lv_label_set_long_mode(s_camera_status_label, LV_LABEL_LONG_DOT);
    lv_obj_align(s_camera_status_label, LV_ALIGN_TOP_MID, 0, 10);

    const size_t canvas_pixels = APP_LCD_H_RES * APP_LCD_PHOTO_PREVIEW_H;
    if (!s_camera_canvas_buf) {
        s_camera_canvas_buf = heap_caps_malloc(canvas_pixels * sizeof(lv_color_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_camera_canvas_buf) {
            s_camera_canvas_buf = heap_caps_malloc(canvas_pixels * sizeof(lv_color_t),
                                                   MALLOC_CAP_8BIT);
        }
        ESP_RETURN_ON_FALSE(s_camera_canvas_buf, ESP_ERR_NO_MEM, TAG,
                            "camera canvas alloc");
    }
    for (size_t i = 0; i < canvas_pixels; ++i) {
        s_camera_canvas_buf[i] = lv_color_hex(0x18232d);
    }

    s_camera_canvas = lv_canvas_create(scr);
    lv_canvas_set_buffer(s_camera_canvas, s_camera_canvas_buf,
                         APP_LCD_H_RES, APP_LCD_PHOTO_PREVIEW_H,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(s_camera_canvas, LV_ALIGN_TOP_MID, 0, 36);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 156, 42);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_add_event_cb(btn, camera_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Capture");
    lv_obj_center(btn_label);

    return ESP_OK;
}

static void lvgl_task(void *arg)
{
    (void)arg;
    uint32_t last_lcd_stats_sequence = 0;
    while (true) {
        if (s_lvgl_lock) {
            xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
        }
        lv_timer_handler();
        if (s_lvgl_lock) {
            xSemaphoreGiveRecursive(s_lvgl_lock);
        }

        lcd_refresh_stats_t stats = {0};
        if (lcd_refresh_stats_take(&last_lcd_stats_sequence, &stats) &&
            stats.pixel_count >= (APP_LCD_H_RES * APP_LCD_V_RES) / 2U) {
            const uint32_t idle_us = stats.total_us > stats.xfer_us
                                         ? stats.total_us - stats.xfer_us
                                         : 0;
            ESP_LOGI(TAG,
                     "[LCD] period=%luus total=%luus flushes=%lu pixels=%lu "
                     "swap=%luus submit=%luus xfer=%luus idle=%luus",
                     (unsigned long)stats.period_us,
                     (unsigned long)stats.total_us,
                     (unsigned long)stats.flush_count,
                     (unsigned long)stats.pixel_count,
                     (unsigned long)stats.swap_us,
                     (unsigned long)stats.submit_us,
                     (unsigned long)stats.xfer_us,
                     (unsigned long)idle_us);
        }
        vTaskDelay(pdMS_TO_TICKS(APP_LCD_LVGL_TASK_DELAY_MS));
    }
}

esp_err_t bsp_lcd_init(void)
{
    if (s_lcd_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    esp_err_t bl_err = bsp_ioexp_set_pin(BSP_IO_EXP_LCD_BL_PIN, false);
    if (bl_err != ESP_OK) {
        ESP_LOGW(TAG, "backlight off via IO expander unavailable: %s",
                 esp_err_to_name(bl_err));
    }

    if (!s_lcd_bus_ready) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = BSP_LCD_SPI_MOSI_GPIO,
            .miso_io_num = -1,
            .sclk_io_num = BSP_LCD_SPI_SCLK_GPIO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS * sizeof(uint16_t),
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                            TAG, "spi bus");
        s_lcd_bus_ready = true;
        ESP_LOGI(TAG, "SPI3 bus initialized: SCLK=%d MOSI=%d",
                 BSP_LCD_SPI_SCLK_GPIO, BSP_LCD_SPI_MOSI_GPIO);

        /* Video frames are read straight out of PSRAM by SPI3 TX GDMA. Give
         * that channel arbitration priority, otherwise 64-byte cache refills
         * from the other bus masters starve the pixel stream and it
         * underflows mid-frame. */
        esp_err_t dma_err =
            app_spi_tx_dma_prioritize_psram(BSP_LCD_SPI_HOST);
        if (dma_err != ESP_OK) {
            ESP_LOGW(TAG, "SPI3 TX DMA stays at IDF defaults: %s",
                     esp_err_to_name(dma_err));
        }
    }

    gpio_config_t cs_cfg = {
        .pin_bit_mask = 1ULL << BSP_LCD_SPI_CS_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cs_cfg), TAG, "lcd manual cs gpio");
    lcd_cs_deselect();

    if (!s_lcd_color_idle) {
        s_lcd_color_idle = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_lcd_color_idle, ESP_ERR_NO_MEM, TAG,
                            "lcd color semaphore");
    }
    (void)xSemaphoreTake(s_lcd_color_idle, 0);
    xSemaphoreGive(s_lcd_color_idle);

    const esp_lcd_panel_io_spi_config_t cmd_io_cfg = {
        .cs_gpio_num = -1,
        .dc_gpio_num = BSP_LCD_SPI_DC_GPIO,
        .spi_mode = 0,
        .pclk_hz = APP_LCD_SPI_CMD_PCLK_HZ,
        .trans_queue_depth = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(
                            (esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST,
                            &cmd_io_cfg, &s_lcd_io),
                        TAG, "low-speed command panel io");

    const spi_device_interface_config_t pixel_dev_cfg = {
        .mode = 0,
        .duty_cycle_pos = LCD_PIXEL_DUTY_CYCLE_POS,
        .clock_speed_hz = APP_LCD_SPI_PCLK_HZ,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = APP_LCD_SPI_QUEUE_DEPTH,
        .post_cb = lcd_pixel_post_trans_cb,
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_add_device(BSP_LCD_SPI_HOST, &pixel_dev_cfg,
                           &s_lcd_pixel_spi),
        TAG, "20 MHz 25-percent-duty pixel SPI device");
    ESP_RETURN_ON_ERROR(
        spi_bus_get_max_transaction_len(
            BSP_LCD_SPI_HOST, &s_lcd_pixel_max_transfer_bytes),
        TAG, "pixel max transaction length");

    /* spi_bus_get_max_transaction_len() does not report the size configured
     * above. spicommon_dma_desc_alloc() rounds max_transfer_sz up to a whole
     * number of DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED (4092) descriptors,
     * so 19200 comes back as 5 * 4092 = 20460.
     *
     * A PSRAM frame is split on that value, and 20460 is not a multiple of the
     * data cache line: every chunk after the first then starts 44 bytes into a
     * cache line. With 32-byte lines the GDMA block was 32 bytes and the AHB
     * GDMA v1 auto-alignment absorbed the smaller offset; at 64 bytes the block
     * and the misalignment both grow and the chunk boundaries no longer land
     * where the hardware realigns.
     *
     * Clamp back to the configured 19200 (exactly 40 lines, 64 * 300) and mask
     * to the cache line so the split stays aligned for any cache
     * configuration. The frame then divides evenly into 8 chunks. */
    const size_t configured_transfer_bytes =
        APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS * sizeof(uint16_t);
    if (s_lcd_pixel_max_transfer_bytes > configured_transfer_bytes) {
        s_lcd_pixel_max_transfer_bytes = configured_transfer_bytes;
    }
    s_lcd_pixel_max_transfer_bytes &=
        ~(size_t)(LCD_DMA_CACHE_ALIGNMENT - 1U);
    ESP_RETURN_ON_FALSE(s_lcd_pixel_max_transfer_bytes > 0,
                        ESP_ERR_INVALID_SIZE, TAG,
                        "pixel transfer size collapsed after alignment");
    s_lcd_pixel_inflight = 0;

    ESP_RETURN_ON_ERROR(
        gpio_set_drive_capability(BSP_LCD_SPI_SCLK_GPIO,
                                  (gpio_drive_cap_t)APP_LCD_SPI_SCLK_DRIVE_CAP),
        TAG, "lcd sclk drive capability");
    ESP_RETURN_ON_ERROR(
        gpio_set_drive_capability(BSP_LCD_SPI_MOSI_GPIO,
                                  (gpio_drive_cap_t)APP_LCD_SPI_MOSI_DRIVE_CAP),
        TAG, "lcd mosi drive capability");
    ESP_RETURN_ON_ERROR(
        gpio_set_drive_capability(BSP_LCD_SPI_CS_GPIO,
                                  (gpio_drive_cap_t)APP_LCD_SPI_CS_DRIVE_CAP),
        TAG, "lcd cs drive capability");
    ESP_RETURN_ON_ERROR(
        gpio_set_drive_capability(BSP_LCD_SPI_DC_GPIO,
                                  (gpio_drive_cap_t)APP_LCD_SPI_DC_DRIVE_CAP),
        TAG, "lcd dc drive capability");

    const size_t max_transfer_bytes = s_lcd_pixel_max_transfer_bytes;
    const size_t full_frame_bytes =
        APP_LCD_H_RES * APP_LCD_V_RES * sizeof(uint16_t);
    const size_t full_frame_chunks =
        (full_frame_bytes + max_transfer_bytes - 1U) / max_transfer_bytes;
    ESP_LOGI(TAG,
             "[LCD-SPI] split host=SPI3 mode=0 command_request=%uHz "
             "command_actual=%uHz pixel_request=%uHz pixel_actual=%uHz "
             "pixel_duty=%u/256 apb=%uHz queue=%u "
             "max_transfer=%uB frame=%uB chunks=%u",
             (unsigned)APP_LCD_SPI_CMD_PCLK_HZ,
             (unsigned)lcd_spi_actual_clock_hz(
                 APP_LCD_SPI_CMD_PCLK_HZ, 128U),
             (unsigned)APP_LCD_SPI_PCLK_HZ,
             (unsigned)lcd_spi_actual_clock_hz(
                 APP_LCD_SPI_PCLK_HZ,
                 LCD_PIXEL_DUTY_CYCLE_POS),
             (unsigned)LCD_PIXEL_DUTY_CYCLE_POS,
             (unsigned)APB_CLK_FREQ,
             (unsigned)APP_LCD_SPI_QUEUE_DEPTH,
             (unsigned)max_transfer_bytes, (unsigned)full_frame_bytes,
             (unsigned)full_frame_chunks);
    ESP_LOGI(TAG,
             "[LCD-SPI] split path: init/CASET/RASET/RAMWR=%uHz, "
             "RGB565-DMA=%uHz duty=%u/256, "
             "CS=GPIO%d manual hold until pixel ISR",
             (unsigned)APP_LCD_SPI_CMD_PCLK_HZ,
             (unsigned)APP_LCD_SPI_PCLK_HZ,
             (unsigned)LCD_PIXEL_DUTY_CYCLE_POS,
             BSP_LCD_SPI_CS_GPIO);
    ESP_LOGI(TAG,
             "[LCD-SPI] fallback TX staging: buffers=%u bytes_each=%u "
             "alignment=%u internal_dma=1",
             (unsigned)LCD_PSRAM_STAGE_BUFFER_COUNT,
             (unsigned)LCD_PSRAM_STAGE_BYTES,
             (unsigned)LCD_DMA_CACHE_ALIGNMENT);
    ESP_LOGI(TAG,
             "[LCD-SPI] drive_cap SCLK=%d MOSI=%d CS=%d DC=%d "
             "(0=weakest,3=strongest,-1=read_failed)",
             lcd_gpio_drive_capability(BSP_LCD_SPI_SCLK_GPIO),
             lcd_gpio_drive_capability(BSP_LCD_SPI_MOSI_GPIO),
             lcd_gpio_drive_capability(BSP_LCD_SPI_CS_GPIO),
             lcd_gpio_drive_capability(BSP_LCD_SPI_DC_GPIO));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(s_lcd_io, &panel_cfg, &s_lcd_panel),
                        TAG, "st7789 panel");

    esp_err_t ret;
    ret = lcd_reset_gpio();
    ESP_LOGI(TAG, "lcd_reset_gpio: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd hw reset");

    ret = lcd_panel_init_low_speed();
    ESP_LOGI(TAG, "lcd_panel_init_low_speed (%u Hz): %s",
             (unsigned)APP_LCD_SPI_CMD_PCLK_HZ, esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd low-speed init");

    vTaskDelay(pdMS_TO_TICKS(120));

    ret = esp_lcd_panel_set_gap(s_lcd_panel, APP_LCD_X_GAP, APP_LCD_Y_GAP);
    ESP_LOGI(TAG, "esp_lcd_panel_set_gap: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd gap");

    lcd_cs_select();
    ret = esp_lcd_panel_invert_color(s_lcd_panel, true);
    lcd_cs_deselect();
    ESP_LOGI(TAG, "esp_lcd_panel_invert_color (command clock): %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd invert");

    lcd_cs_select();
    ret = esp_lcd_panel_disp_on_off(s_lcd_panel, true);
    lcd_cs_deselect();
    ESP_LOGI(TAG, "esp_lcd_panel_disp_on_off (command clock): %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd on");
    bl_err = bsp_ioexp_set_pin(BSP_IO_EXP_LCD_BL_PIN, true);
    if (bl_err != ESP_OK) {
        ESP_LOGW(TAG, "backlight on via IO expander unavailable: %s",
                 esp_err_to_name(bl_err));
    }

    s_lcd_ready = true;
    ESP_LOGI(TAG, "ST7789V3 LCD ready: %ux%u, SPI3 sclk=%d mosi=%d dc=%d cs=%d te=%d bl=P%d rst=P%d",
             APP_LCD_H_RES, APP_LCD_V_RES, BSP_LCD_SPI_SCLK_GPIO,
             BSP_LCD_SPI_MOSI_GPIO, BSP_LCD_SPI_DC_GPIO, BSP_LCD_SPI_CS_GPIO,
             BSP_LCD_TE_GPIO, BSP_IO_EXP_LCD_BL_PIN, BSP_IO_EXP_LCD_RST_PIN);
    return ESP_OK;
}

esp_err_t bsp_lcd_release_for_camera(void)
{
    s_lcd_suspended = true;
    vTaskDelay(pdMS_TO_TICKS(40));

    if (s_lcd_color_idle) {
        ESP_RETURN_ON_FALSE(
            xSemaphoreTake(s_lcd_color_idle, pdMS_TO_TICKS(1000)) == pdTRUE,
            ESP_ERR_TIMEOUT, TAG, "LCD pixel transfer still active during release");
        xSemaphoreGive(s_lcd_color_idle);
    }

    if (s_lcd_panel) {
        lcd_cs_select();
        esp_lcd_panel_disp_on_off(s_lcd_panel, false);
        lcd_cs_deselect();
        esp_lcd_panel_del(s_lcd_panel);
        s_lcd_panel = NULL;
        s_lcd_ready = false;
    }
    if (s_lcd_pixel_spi) {
        esp_err_t err = lcd_pixel_recycle_completed();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "pixel transaction recycle failed: %s",
                     esp_err_to_name(err));
        }
        err = spi_bus_remove_device(s_lcd_pixel_spi);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "pixel SPI device removal failed: %s",
                     esp_err_to_name(err));
        } else {
            s_lcd_pixel_spi = NULL;
            s_lcd_pixel_max_transfer_bytes = 0;
            s_lcd_pixel_inflight = 0;
        }
        s_lcd_ready = false;
    }
    if (s_lcd_io) {
        esp_lcd_panel_io_del(s_lcd_io);
        s_lcd_io = NULL;
        s_lcd_ready = false;
    }
    lcd_cs_deselect();
    if (s_lcd_bus_ready) {
        esp_err_t err = spi_bus_free(BSP_LCD_SPI_HOST);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "spi bus free failed: %s", esp_err_to_name(err));
        } else {
            s_lcd_bus_ready = false;
        }
    }
    bsp_ioexp_set_pin(BSP_IO_EXP_LCD_BL_PIN, false);
    s_video_path_logged = false;
    s_split_draw_logged = false;
    return ESP_OK;
}

esp_err_t bsp_lcd_reinit_after_camera(void)
{
    ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd reinit");
    s_lcd_suspended = false;
    if (s_lvgl_lock) {
        xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
        lv_obj_invalidate(lv_scr_act());
        xSemaphoreGiveRecursive(s_lvgl_lock);
    }
    return ESP_OK;
}

esp_err_t bsp_lcd_show_test_pattern(void)
{
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    const size_t rows = APP_LCD_TEST_PATTERN_ROWS;
    const size_t pixels = APP_LCD_H_RES * rows;
    uint16_t *line = lcd_alloc_internal_dma(pixels * sizeof(uint16_t));
    if (!line) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "[LCD-DIAG] pattern buffer addr=%p bytes=%u aligned64=%d "
             "internal_dma=%d",
             (void *)line, (unsigned)(pixels * sizeof(uint16_t)),
             (((uintptr_t)line & (LCD_DMA_CACHE_ALIGNMENT - 1U)) == 0U) ? 1 : 0,
             esp_ptr_dma_capable(line) ? 1 : 0);

    static const uint16_t colors[] = {
        0xf800, 0x07e0, 0x001f, 0xffe0, 0x07ff, 0xf81f, 0xffff, 0x0000,
    };
    for (uint32_t y = 0; y < APP_LCD_V_RES; y += rows) {
        uint32_t draw_rows = APP_LCD_V_RES - y;
        if (draw_rows > rows) {
            draw_rows = rows;
        }
        for (uint32_t row = 0; row < draw_rows; ++row) {
            for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
                uint32_t band = (x * (sizeof(colors) / sizeof(colors[0]))) / APP_LCD_H_RES;
                line[row * APP_LCD_H_RES + x] = colors[band];
            }
        }
        esp_err_t err = lcd_draw_rgb565_bitmap(0, y, APP_LCD_H_RES,
                                               y + draw_rows, line,
                                               LCD_TRANSFER_OWNER_OTHER, 0);
        if (y == 0) {
            ESP_LOGI(TAG, "first draw_bitmap (y=0..%lu): %s", (unsigned long)draw_rows, esp_err_to_name(err));
        }
        if (err != ESP_OK) {
            heap_caps_free(line);
            ESP_RETURN_ON_ERROR(err, TAG, "draw test");
        }
    }

    if (xSemaphoreTake(s_lcd_color_idle, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "LCD test pattern final transfer timeout");
        return ESP_ERR_TIMEOUT;
    }
    xSemaphoreGive(s_lcd_color_idle);

    heap_caps_free(line);
    ESP_LOGI(TAG, "LCD test pattern drawn with split-speed SPI");
    return ESP_OK;
}

esp_err_t bsp_lcd_start_lvgl_demo(void)
{
    if (s_lvgl_started) {
        return ESP_OK;
    }
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    if (!s_lvgl_lock) {
        s_lvgl_lock = xSemaphoreCreateRecursiveMutex();
        ESP_RETURN_ON_FALSE(s_lvgl_lock, ESP_ERR_NO_MEM, TAG, "lvgl lock");
    }

    lv_init();

    const size_t pixels = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS;
    lv_color_t *buf1 = lcd_alloc_internal_dma(pixels * sizeof(lv_color_t));
    lv_color_t *buf2 = lcd_alloc_internal_dma(pixels * sizeof(lv_color_t));
    if (!buf1 || !buf2) {
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }
    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = APP_LCD_H_RES;
    disp_drv.ver_res = APP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    s_lvgl_disp_drv = &disp_drv;
    lv_disp_drv_register(&disp_drv);
    ESP_RETURN_ON_ERROR(lvgl_register_flush_ready_cb(&disp_drv), TAG,
                        "register LVGL flush callback");

    esp_err_t touch_err = touch_attach();
    if (touch_err == ESP_OK) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read_cb;
        lv_indev_drv_register(&indev_drv);
    }

    lvgl_create_demo_ui();

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG,
                        "lvgl tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer,
                                                 APP_LCD_LVGL_TICK_MS * 1000U),
                        TAG, "lvgl tick start");

    BaseType_t ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl",
                                            APP_LCD_LVGL_TASK_STACK_BYTES, NULL,
                                            APP_LCD_LVGL_TASK_PRIORITY, NULL,
                                            APP_LCD_LVGL_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_started = true;
    ESP_LOGI(TAG, "LVGL demo started%s", s_touch ? " with touch" : "");
    return ESP_OK;
}

esp_err_t bsp_lcd_start_camera_ui(bsp_lcd_capture_cb_t cb, void *user)
{
    if (s_lvgl_started) {
        s_capture_cb = cb;
        s_capture_user = user;
        if (s_lvgl_lock) {
            xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
            esp_err_t err = lvgl_create_camera_ui();
            xSemaphoreGiveRecursive(s_lvgl_lock);
            return err;
        }
        return ESP_OK;
    }
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    if (!s_lvgl_lock) {
        s_lvgl_lock = xSemaphoreCreateRecursiveMutex();
        ESP_RETURN_ON_FALSE(s_lvgl_lock, ESP_ERR_NO_MEM, TAG, "lvgl lock");
    }

    lv_init();

    const size_t pixels = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS;
    lv_color_t *buf1 = lcd_alloc_internal_dma(pixels * sizeof(lv_color_t));
    lv_color_t *buf2 = lcd_alloc_internal_dma(pixels * sizeof(lv_color_t));
    if (!buf1 || !buf2) {
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = APP_LCD_H_RES;
    disp_drv.ver_res = APP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    s_lvgl_disp_drv = &disp_drv;
    lv_disp_drv_register(&disp_drv);
    ESP_RETURN_ON_ERROR(lvgl_register_flush_ready_cb(&disp_drv), TAG,
                        "register LVGL flush callback");

    esp_err_t touch_err = touch_attach();
    if (touch_err == ESP_OK) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read_cb;
        lv_indev_drv_register(&indev_drv);
    }

    s_capture_cb = cb;
    s_capture_user = user;
    ESP_RETURN_ON_ERROR(lvgl_create_camera_ui(), TAG, "camera ui");

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG,
                        "lvgl tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer,
                                                 APP_LCD_LVGL_TICK_MS * 1000U),
                        TAG, "lvgl tick start");

    BaseType_t ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl",
                                            APP_LCD_LVGL_TASK_STACK_BYTES, NULL,
                                            APP_LCD_LVGL_TASK_PRIORITY, NULL,
                                            APP_LCD_LVGL_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_started = true;
    ESP_LOGI(TAG, "camera LVGL UI started%s", s_touch ? " with touch" : "");
    return ESP_OK;
}

esp_err_t bsp_lcd_start_gateway_ui(void)
{
    if (s_lvgl_started) {
        extern esp_err_t ui_gw_init(void);
        if (s_lvgl_lock) {
            xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
            esp_err_t err = ui_gw_init();
            xSemaphoreGiveRecursive(s_lvgl_lock);
            return err;
        }
        return ui_gw_init();
    }
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    if (!s_lvgl_lock) {
        s_lvgl_lock = xSemaphoreCreateRecursiveMutex();
        ESP_RETURN_ON_FALSE(s_lvgl_lock, ESP_ERR_NO_MEM, TAG, "lvgl lock");
    }

    lv_init();

    const size_t pixels = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS;
    lv_color_t *buf1 = lcd_alloc_internal_dma(pixels * sizeof(lv_color_t));
    lv_color_t *buf2 = lcd_alloc_internal_dma(pixels * sizeof(lv_color_t));
    if (!buf1 || !buf2) {
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "[LCD-SPI] gateway LVGL DMA buffers bytes_each=%u "
             "buf1=%p aligned64=%d buf2=%p aligned64=%d",
             (unsigned)(pixels * sizeof(lv_color_t)), (void *)buf1,
             (((uintptr_t)buf1 & (LCD_DMA_CACHE_ALIGNMENT - 1U)) == 0U) ? 1 : 0,
             (void *)buf2,
             (((uintptr_t)buf2 & (LCD_DMA_CACHE_ALIGNMENT - 1U)) == 0U) ? 1 : 0);

    static lv_disp_draw_buf_t draw_buf_gw;
    lv_disp_draw_buf_init(&draw_buf_gw, buf1, buf2, pixels);

    static lv_disp_drv_t disp_drv_gw;
    lv_disp_drv_init(&disp_drv_gw);
    disp_drv_gw.hor_res = APP_LCD_H_RES;
    disp_drv_gw.ver_res = APP_LCD_V_RES;
    disp_drv_gw.flush_cb = lvgl_flush_cb;
    disp_drv_gw.draw_buf = &draw_buf_gw;
    s_lvgl_disp_drv = &disp_drv_gw;
    lv_disp_drv_register(&disp_drv_gw);
    ESP_RETURN_ON_ERROR(lvgl_register_flush_ready_cb(&disp_drv_gw), TAG,
                        "register LVGL flush callback");

    esp_err_t touch_err = touch_attach();
    if (touch_err == ESP_OK) {
        static lv_indev_drv_t indev_drv_gw;
        lv_indev_drv_init(&indev_drv_gw);
        indev_drv_gw.type = LV_INDEV_TYPE_POINTER;
        indev_drv_gw.read_cb = lvgl_touch_read_cb;
        lv_indev_drv_register(&indev_drv_gw);
    }

    extern esp_err_t ui_gw_init(void);
    ESP_RETURN_ON_ERROR(ui_gw_init(), TAG, "gateway ui");

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG,
                        "lvgl tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer,
                                                 APP_LCD_LVGL_TICK_MS * 1000U),
                        TAG, "lvgl tick start");

    BaseType_t ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl",
                                            APP_LCD_LVGL_TASK_STACK_BYTES, NULL,
                                            APP_LCD_LVGL_TASK_PRIORITY, NULL,
                                            APP_LCD_LVGL_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_started = true;
    ESP_LOGI(TAG, "Gateway LVGL UI started");
    return ESP_OK;
}

SemaphoreHandle_t bsp_lcd_get_lvgl_lock(void)
{
    return s_lvgl_lock;
}

void bsp_lcd_set_video_direct_owner(bool owns_panel)
{
    s_video_direct_owns_panel = owns_panel;
}

void bsp_lcd_refresh_video_status(void)
{
    /* Caller holds the LVGL lock. The final DMA keeps normal LVGL ownership
     * until its ISR signals flush_ready; no radio task participates. */
    s_video_status_refresh = true;
    lv_refr_now(NULL);
    s_video_status_refresh = false;
}

esp_err_t bsp_lcd_present_video_frame(const uint16_t *rgb565,
                                      uint32_t width,
                                      uint32_t height,
                                      uint32_t y_offset)
{
    ESP_RETURN_ON_FALSE(s_lcd_ready && rgb565, ESP_ERR_INVALID_STATE, TAG,
                        "lcd not ready");
    ESP_RETURN_ON_FALSE(width == APP_LCD_H_RES && height > 0U &&
                            y_offset < APP_LCD_V_RES &&
                            height <= APP_LCD_V_RES - y_offset,
                        ESP_ERR_INVALID_ARG, TAG, "invalid video frame size");

    /* Publish the rows this frame owns so lvgl_flush_cb drops only the LVGL
     * output that would fight it. Set before the transfer, not after: LVGL runs
     * on its own task and may flush while this one is still in flight. */
    s_video_region_y0 = y_offset;

    if (!s_video_path_logged) {
        const bool external = esp_ptr_external_ram(rgb565);
        const bool staged = lcd_pixel_needs_staging(rgb565);
        const size_t frame_bytes = width * height * sizeof(uint16_t);
        const size_t chunk_bytes = staged ? LCD_PSRAM_STAGE_BYTES
                                          : s_lcd_pixel_max_transfer_bytes;
        const size_t chunks =
            (frame_bytes + chunk_bytes - 1U) / chunk_bytes;
        ESP_LOGI(TAG,
                 "[LCD-SPI] video_buffer=%s dma=%s addr=%p aligned64=%d "
                 "bytes=%u chunk_bytes=%u chunks=%u chunk_aligned64=%d",
                 external ? "PSRAM" : "internal",
                 staged ? "internal-stage"
                        : (external ? "PSRAM-direct" : "internal-direct"),
                 (const void *)rgb565,
                 (((uintptr_t)rgb565 & 63U) == 0U) ? 1 : 0,
                 (unsigned)frame_bytes, (unsigned)chunk_bytes,
                 (unsigned)chunks,
                 ((chunk_bytes & (LCD_DMA_CACHE_ALIGNMENT - 1U)) == 0U) ? 1 : 0);
        s_video_path_logged = true;
    }
    if (!s_video_frame_done) {
        s_video_frame_done = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_video_frame_done, ESP_ERR_NO_MEM, TAG,
                            "video frame semaphore");
    }
    bool already_inflight;
    lcd_transfer_owner_t active_owner;
    uint32_t active_sequence;
    portENTER_CRITICAL(&s_lcd_stats_mux);
    already_inflight = s_video_frame_inflight;
    active_owner = s_lcd_transfer_owner;
    active_sequence = s_lcd_transfer_owner_sequence;
    portEXIT_CRITICAL(&s_lcd_stats_mux);
    if (already_inflight) {
        ESP_LOGE(TAG,
                 "[LCD-TRANSITION] reject video: inflight=1 owner=%d owner_seq=%lu",
                 (int)active_owner, (unsigned long)active_sequence);
        return ESP_ERR_INVALID_STATE;
    }

    (void)xSemaphoreTake(s_video_frame_done, 0);
    uint32_t frame_sequence;
    uint32_t pending_before_submit;
    uint32_t mismatch_before_submit;
    portENTER_CRITICAL(&s_lcd_stats_mux);
    frame_sequence = ++s_video_frame_sequence;
    s_video_frame_done_sequence = 0;
    s_video_frame_isr_done_us = 0;
    s_video_frame_inflight = true;
    pending_before_submit = s_lcd_pending_count;
    mismatch_before_submit = s_lcd_owner_mismatch_count;
    portEXIT_CRITICAL(&s_lcd_stats_mux);
    if (frame_sequence <= 3U) {
        ESP_LOGW(TAG,
                 "[LCD-TRANSITION] submit video seq=%lu lvgl_pending=%lu owner_mismatch=%lu",
                 (unsigned long)frame_sequence,
                 (unsigned long)pending_before_submit,
                 (unsigned long)mismatch_before_submit);
    }

#if LCD_VIDEO_INTEGRITY_CHECK
    /* CPU view, through the cache: exactly what rotate_rgb565_to_canvas wrote. */
    const size_t integrity_pixels = (size_t)width * height;
    uint32_t fingerprint_before[LCD_INTEGRITY_SEGMENTS];
    lcd_video_fingerprint(rgb565, integrity_pixels, fingerprint_before);
#endif

    const int64_t start_us = esp_timer_get_time();
    /* lcd_draw_rgb565_bitmap takes END coordinates, not a width/height. The old
     * call read as (0, 0, width, height) only because an origin of (0,0) makes
     * the two forms identical; with a y_offset they are not. */
    esp_err_t err = lcd_draw_rgb565_bitmap(
        0, y_offset, width, y_offset + height, rgb565,
        LCD_TRANSFER_OWNER_VIDEO, frame_sequence);
    const int64_t submit_done_us = esp_timer_get_time();
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_lcd_stats_mux);
        if (s_video_frame_sequence == frame_sequence) {
            s_video_frame_inflight = false;
        }
        portEXIT_CRITICAL(&s_lcd_stats_mux);
        ESP_LOGE(TAG, "[LCD-TRANSITION] video submit failed seq=%lu: %s",
                 (unsigned long)frame_sequence, esp_err_to_name(err));
        return err;
    }

    if (xSemaphoreTake(s_video_frame_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
        const bool pixel_idle =
            xSemaphoreTake(s_lcd_color_idle, 0) == pdTRUE;
        if (pixel_idle) {
            xSemaphoreGive(s_lcd_color_idle);
        }

        bool recovered = false;
        uint32_t pending_on_timeout;
        uint32_t mismatch_on_timeout;
        portENTER_CRITICAL(&s_lcd_stats_mux);
        active_owner = s_lcd_transfer_owner;
        active_sequence = s_lcd_transfer_owner_sequence;
        pending_on_timeout = s_lcd_pending_count;
        mismatch_on_timeout = s_lcd_owner_mismatch_count;
        if (pixel_idle && s_video_frame_sequence == frame_sequence) {
            s_video_frame_inflight = false;
            if (s_lcd_transfer_owner == LCD_TRANSFER_OWNER_VIDEO &&
                s_lcd_transfer_owner_sequence == frame_sequence) {
                s_lcd_transfer_owner = LCD_TRANSFER_OWNER_NONE;
                s_lcd_transfer_owner_sequence = 0;
            }
            recovered = true;
        }
        portEXIT_CRITICAL(&s_lcd_stats_mux);

        ESP_LOGE(TAG,
                 "[LCD-TRANSITION] video timeout seq=%lu pixel_idle=%d "
                 "owner=%d owner_seq=%lu lvgl_pending=%lu owner_mismatch=%lu recovered=%d",
                 (unsigned long)frame_sequence, pixel_idle ? 1 : 0,
                 (int)active_owner, (unsigned long)active_sequence,
                 (unsigned long)pending_on_timeout,
                 (unsigned long)mismatch_on_timeout, recovered ? 1 : 0);
        return ESP_ERR_TIMEOUT;
    }

    /* The ISR callback reports that the last queued chunk completed, but the
     * per-transaction result queue is authoritative for PSRAM DMA underflow.
     * Drain this frame now so a failed chunk is reported to the UI immediately
     * and cannot poison the next frame's inflight accounting. */
    esp_err_t transfer_err = lcd_pixel_recycle_completed();
    if (transfer_err != ESP_OK) {
        ESP_LOGE(TAG, "[LCD-TRANSITION] video DMA failed seq=%lu: %s",
                 (unsigned long)frame_sequence,
                 esp_err_to_name(transfer_err));
        return transfer_err;
    }

#if LCD_VIDEO_INTEGRITY_CHECK
    /* Drop the cached copy so the second pass reads what actually sits in
     * PSRAM, i.e. the bytes GDMA fetched. The buffer is clean at this point:
     * spi_master ran a C2M writeback on every chunk when it was queued, so
     * invalidating cannot lose CPU data. */
    const size_t integrity_bytes = integrity_pixels * sizeof(uint16_t);
    /* No UNALIGNED flag: M2C rejects it, and the canvas is already 64-byte
     * aligned with a length that is a whole number of cache lines. */
    esp_err_t inv_err = esp_cache_msync((void *)rgb565, integrity_bytes,
                                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    if (inv_err != ESP_OK) {
        ESP_LOGW(TAG, "[LCD-INTEGRITY] invalidate failed: %s",
                 esp_err_to_name(inv_err));
    } else {
        uint32_t fingerprint_after[LCD_INTEGRITY_SEGMENTS];
        lcd_video_fingerprint(rgb565, integrity_pixels, fingerprint_after);

        uint32_t bad_mask = 0;
        for (uint32_t seg = 0; seg < LCD_INTEGRITY_SEGMENTS; ++seg) {
            if (fingerprint_after[seg] != fingerprint_before[seg]) {
                bad_mask |= (1U << seg);
            }
        }
        static uint32_t s_integrity_mismatch_count;
        if (bad_mask != 0) {
            const uint32_t first = (uint32_t)__builtin_ctz(bad_mask);
            ESP_LOGE(TAG,
                     "[LCD-INTEGRITY] PSRAM differs from CPU writes seq=%lu "
                     "segments=0x%02lx first=%lu rows=%lu..%lu "
                     "cpu=0x%08lx psram=0x%08lx mismatches=%lu addr=%p",
                     (unsigned long)frame_sequence, (unsigned long)bad_mask,
                     (unsigned long)first,
                     (unsigned long)(first * (height / LCD_INTEGRITY_SEGMENTS)),
                     (unsigned long)((first + 1U) *
                                     (height / LCD_INTEGRITY_SEGMENTS) - 1U),
                     (unsigned long)fingerprint_before[first],
                     (unsigned long)fingerprint_after[first],
                     (unsigned long)++s_integrity_mismatch_count,
                     (const void *)rgb565);
        }
    }
#endif

    const int64_t resume_us = esp_timer_get_time();
    int64_t isr_done_us;
    uint32_t done_sequence;
    portENTER_CRITICAL(&s_lcd_stats_mux);
    isr_done_us = s_video_frame_isr_done_us;
    done_sequence = s_video_frame_done_sequence;
    portEXIT_CRITICAL(&s_lcd_stats_mux);
    if (done_sequence != frame_sequence || isr_done_us <= 0) {
        ESP_LOGE(TAG, "video frame completion mismatch: seq=%lu done_seq=%lu isr=%lld",
                 (unsigned long)frame_sequence, (unsigned long)done_sequence,
                 isr_done_us);
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t submit_us = submit_done_us - start_us;
    const int64_t submit_to_isr_us = isr_done_us - start_us;
    const int64_t physical_us = isr_done_us > submit_done_us
                                    ? isr_done_us - submit_done_us
                                    : 0;
    const int64_t isr_to_resume_us = resume_us - isr_done_us;
    const int64_t total_us = resume_us - start_us;
    static int64_t last_isr_done_us;
    static int64_t last_resume_us;
    /* One line per displayed frame. The direct-video path is settled, so keep
     * the per-frame breakdown at DEBUG and read the display rate from the
     * [FRAME] line instead. */
    ESP_LOGD(TAG, "[LCD] direct seq=%lu submit=%lldus submit_to_isr=%lldus "
                  "physical=%lldus isr_to_resume=%lldus total=%lldus "
                  "isr_period=%lldus resume_period=%lldus pixels=%lu "
                  "dma_err=%lu",
             (unsigned long)frame_sequence, submit_us, submit_to_isr_us,
             physical_us, isr_to_resume_us, total_us,
             last_isr_done_us > 0 ? isr_done_us - last_isr_done_us : 0,
             last_resume_us > 0 ? resume_us - last_resume_us : 0,
             (unsigned long)(width * height),
             (unsigned long)s_lcd_pixel_dma_error_count);
    last_isr_done_us = isr_done_us;
    last_resume_us = resume_us;
    return ESP_OK;
}

esp_err_t bsp_lcd_set_camera_status(const char *text)
{
    if (!s_lvgl_started || !s_camera_status_label || !text) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    lv_label_set_text(s_camera_status_label, text);
    lv_obj_invalidate(s_camera_status_label);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

esp_err_t bsp_lcd_clear_camera_photo(void)
{
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    const size_t canvas_pixels = APP_LCD_H_RES * APP_LCD_PHOTO_PREVIEW_H;
    for (size_t i = 0; i < canvas_pixels; ++i) {
        s_camera_canvas_buf[i] = lv_color_hex(0x18232d);
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

esp_err_t bsp_lcd_show_gray_photo(const uint8_t *gray,
                                  uint32_t width,
                                  uint32_t height)
{
    ESP_RETURN_ON_FALSE(gray && width && height, ESP_ERR_INVALID_ARG, TAG,
                        "invalid gray frame");
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    for (uint32_t y = 0; y < APP_LCD_PHOTO_PREVIEW_H; ++y) {
        uint32_t src_y = (y * height) / APP_LCD_PHOTO_PREVIEW_H;
        const uint8_t *src = gray + src_y * width;
        lv_color_t *dst = s_camera_canvas_buf + y * APP_LCD_H_RES;
        for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
            uint32_t src_x = (x * width) / APP_LCD_H_RES;
            uint8_t v = src[src_x];
            dst[x] = lv_color_make(v, v, v);
        }
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

static inline uint8_t yuv_clamp(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static bool yuv422_get_pair(const uint8_t *p,
                            uint32_t pixelformat,
                            uint8_t *y0,
                            uint8_t *u,
                            uint8_t *y1,
                            uint8_t *v)
{
    switch (pixelformat) {
    case 0x56595559: // 'YUYV'
        *y0 = p[0]; *u = p[1]; *y1 = p[2]; *v = p[3];
        return true;
    case 0x59565955: // 'UYVY'
        *u = p[0]; *y0 = p[1]; *v = p[2]; *y1 = p[3];
        return true;
    case 0x55595659: // 'YVYU'
        *y0 = p[0]; *v = p[1]; *y1 = p[2]; *u = p[3];
        return true;
    case 0x59555956: // 'VYUY'
        *v = p[0]; *y0 = p[1]; *u = p[2]; *y1 = p[3];
        return true;
    default:
        return false;
    }
}

esp_err_t bsp_lcd_show_rgb565_photo(const uint16_t *rgb565,
                                    uint32_t width,
                                    uint32_t height)
{
    ESP_RETURN_ON_FALSE(rgb565 && width && height, ESP_ERR_INVALID_ARG, TAG,
                        "invalid rgb565 frame");
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    for (uint32_t y = 0; y < APP_LCD_PHOTO_PREVIEW_H; ++y) {
        uint32_t src_y = (y * height) / APP_LCD_PHOTO_PREVIEW_H;
        const uint16_t *src = rgb565 + src_y * width;
        lv_color_t *dst = s_camera_canvas_buf + y * APP_LCD_H_RES;
        for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
            uint32_t src_x = (x * width) / APP_LCD_H_RES;
            dst[x].full = src[src_x];
        }
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

esp_err_t bsp_lcd_show_yuv422_photo(const uint8_t *yuv422,
                                    uint32_t width,
                                    uint32_t height,
                                    uint32_t pixelformat)
{
    ESP_RETURN_ON_FALSE(yuv422 && width && height, ESP_ERR_INVALID_ARG, TAG,
                        "invalid yuv frame");
    ESP_RETURN_ON_FALSE(pixelformat == 0x56595559 || pixelformat == 0x59565955 ||
                        pixelformat == 0x55595659 || pixelformat == 0x59555956,
                        ESP_ERR_INVALID_ARG, TAG, "unsupported yuv fourcc");
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    for (uint32_t y = 0; y < APP_LCD_PHOTO_PREVIEW_H; ++y) {
        uint32_t src_y = (y * height) / APP_LCD_PHOTO_PREVIEW_H;
        const uint8_t *row = yuv422 + src_y * width * 2;
        lv_color_t *dst = s_camera_canvas_buf + y * APP_LCD_H_RES;
        for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
            uint32_t src_x = (x * width) / APP_LCD_H_RES;
            uint32_t pair = src_x & ~1u;
            uint8_t y0 = 0;
            uint8_t u = 0;
            uint8_t y1 = 0;
            uint8_t v = 0;
            yuv422_get_pair(row + pair * 2, pixelformat, &y0, &u, &y1, &v);
            uint8_t lum = (src_x & 1) ? y1 : y0;
            int c = lum - 16;
            int d = u - 128;
            int e = v - 128;
            uint8_t r = yuv_clamp((298 * c + 409 * e + 128) >> 8);
            uint8_t g = yuv_clamp((298 * c - 100 * d - 208 * e + 128) >> 8);
            uint8_t b = yuv_clamp((298 * c + 516 * d + 128) >> 8);
            dst[x] = lv_color_make(r, g, b);
        }
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

/*
 * ES8311 codec + I2S duplex + CST8302A PA control.
 *
 * Register sequence follows the ES8311 reference init used by Espressif BSPs:
 *   - I2S master on ESP side / slave on codec side
 *   - MCLK = 256 * Fs sourced from the ESP's MCLK pin
 *   - 16-bit Philips I2S
 *   - Mono analog mic on MIC1 (MIC_P/MIC_N, differential)
 *   - Mono line out on OUTP/OUTN routed through CST8302A to the speaker
 *
 * The PA's SD (mute) pin is wired to the TCA9554A expander P6 (active high
 * = PA enabled).
 */

#include "bsp.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2s_std.h"

static const char *TAG = "bsp_audio";

/* Upper bound on one blocking read/write. A running channel hands over a DMA
 * buffer every 15 ms, so this is only ever hit on a channel that was disabled
 * under the caller, where the driver would otherwise block for good. */
#define BSP_AUDIO_IO_TIMEOUT_MS     100U
/* How long a rebuild waits for such a caller to come back out. */
#define BSP_AUDIO_IO_DRAIN_MS       (3U * BSP_AUDIO_IO_TIMEOUT_MS)

/* ---- ES8311 register set we actually touch ----------------------------- */
#define ES8311_RESET_REG00          0x00
#define ES8311_CLK_MANAGER_REG01    0x01
#define ES8311_CLK_MANAGER_REG02    0x02
#define ES8311_CLK_MANAGER_REG03    0x03
#define ES8311_CLK_MANAGER_REG04    0x04
#define ES8311_CLK_MANAGER_REG05    0x05
#define ES8311_CLK_MANAGER_REG06    0x06
#define ES8311_CLK_MANAGER_REG07    0x07
#define ES8311_CLK_MANAGER_REG08    0x08
#define ES8311_SDPIN_REG09          0x09
#define ES8311_SDPOUT_REG0A         0x0A
#define ES8311_SYSTEM_REG0B         0x0B
#define ES8311_SYSTEM_REG0C         0x0C
#define ES8311_SYSTEM_REG0D         0x0D
#define ES8311_SYSTEM_REG0E         0x0E
#define ES8311_SYSTEM_REG0F         0x0F
#define ES8311_SYSTEM_REG10         0x10
#define ES8311_SYSTEM_REG11         0x11
#define ES8311_SYSTEM_REG12         0x12
#define ES8311_SYSTEM_REG13         0x13
#define ES8311_SYSTEM_REG14         0x14   /* MIC select + PGA gain         */
#define ES8311_ADC_REG15            0x15
#define ES8311_ADC_REG16            0x16
#define ES8311_ADC_REG17            0x17   /* ADC digital volume            */
#define ES8311_ADC_REG1B            0x1B
#define ES8311_ADC_REG1C            0x1C
#define ES8311_DAC_REG31            0x31
#define ES8311_DAC_REG32            0x32   /* DAC digital volume            */
#define ES8311_DAC_REG37            0x37
#define ES8311_GPIO_REG44           0x44
#define ES8311_GP_REG45             0x45
#define ES8311_CHD1_REGFD           0xFD
#define ES8311_CHD2_REGFE           0xFE
#define ES8311_CHVER_REGFF          0xFF

static i2c_master_dev_handle_t s_codec;
static i2s_chan_handle_t       s_tx, s_rx;
static bool                    s_audio_ready;
static volatile bool           s_i2s_enabled;
/* Callers currently inside i2s_channel_read/write. i2s_deinit() must not
 * delete a channel one of them is blocked on: the IDF deletes the semaphore
 * under the blocked task and it never runs again. */
static volatile int            s_io_users;
/* Serialises suspend/resume/depth changes, which come from different tasks. */
static SemaphoreHandle_t       s_reconfig_lock;
static uint32_t                s_sample_rate_hz;
static uint32_t                s_dma_desc_num = 24U;
/* Frames per DMA descriptor, taken from the channel config so the ring depth
 * can be reported in ms. Defaulted to the IDF value in case it is ever read
 * before a channel exists. */
static uint32_t                s_dma_frame_num = 240U;
/* Last value handed to bsp_audio_set_volume(), so a caller that raises the
 * level for one clip can put back exactly what was there. Codec register
 * reads are not used for this: the register is a dB code, not the level. */
static uint8_t                 s_volume_level;

/* ----------------------------------------------------------------------- */

static esp_err_t es_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_master_transmit(s_codec, buf, sizeof(buf), 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 write reg 0x%02X <= 0x%02X failed: %s",
                 reg, val, esp_err_to_name(err));
        return err;
    }

    uint8_t rb = 0;
    err = i2c_master_transmit_receive(s_codec, &reg, 1, &rb, 1, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ES8311 write reg 0x%02X <= 0x%02X, readback=0x%02X%s",
                 reg, val, rb, rb == val ? "" : " MISMATCH");
    } else {
        ESP_LOGW(TAG, "ES8311 write reg 0x%02X <= 0x%02X, readback failed: %s",
                 reg, val, esp_err_to_name(err));
    }
    return ESP_OK;
}

static esp_err_t es_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_codec, &reg, 1, val, 1, 100);
}

static void es_log_reg(uint8_t reg, const char *name)
{
    uint8_t val = 0;
    esp_err_t err = es_read(reg, &val);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ES8311 %-8s reg 0x%02X = 0x%02X", name, reg, val);
    } else {
        ESP_LOGW(TAG, "ES8311 %-8s reg 0x%02X read failed: %s",
                 name, reg, esp_err_to_name(err));
    }
}

static void es_dump_key_regs(const char *stage)
{
    ESP_LOGI(TAG, "ES8311 register snapshot: %s", stage);
    es_log_reg(ES8311_RESET_REG00,       "RESET");
    es_log_reg(ES8311_CLK_MANAGER_REG01, "CLK1");
    es_log_reg(ES8311_CLK_MANAGER_REG02, "CLK2");
    es_log_reg(ES8311_CLK_MANAGER_REG03, "CLK3");
    es_log_reg(ES8311_CLK_MANAGER_REG04, "CLK4");
    es_log_reg(ES8311_CLK_MANAGER_REG05, "CLK5");
    es_log_reg(ES8311_CLK_MANAGER_REG06, "BCLK");
    es_log_reg(ES8311_CLK_MANAGER_REG07, "LRCKH");
    es_log_reg(ES8311_CLK_MANAGER_REG08, "LRCKL");
    es_log_reg(ES8311_SDPIN_REG09,       "SDPIN");
    es_log_reg(ES8311_SDPOUT_REG0A,      "SDPOUT");
    es_log_reg(ES8311_SYSTEM_REG0D,      "PWR0D");
    es_log_reg(ES8311_SYSTEM_REG0E,      "PWR0E");
    es_log_reg(ES8311_SYSTEM_REG0F,      "SYS0F");
    es_log_reg(ES8311_SYSTEM_REG12,      "DAC12");
    es_log_reg(ES8311_SYSTEM_REG13,      "OUT13");
    es_log_reg(ES8311_SYSTEM_REG14,      "MIC14");
    es_log_reg(ES8311_ADC_REG16,         "ADC16");
    es_log_reg(ES8311_ADC_REG17,         "ADCVOL");
    es_log_reg(ES8311_ADC_REG1B,         "ADC1B");
    es_log_reg(ES8311_ADC_REG1C,         "ADC1C");
    es_log_reg(ES8311_DAC_REG31,         "DAC31");
    es_log_reg(ES8311_DAC_REG32,         "DACVOL");
    es_log_reg(ES8311_DAC_REG37,         "DAC37");
    es_log_reg(ES8311_GPIO_REG44,        "GPIO44");
    es_log_reg(ES8311_GP_REG45,          "GP45");
}

/* ES8311 analog init — only sample rate varies across the common 8/16/32/48 k
 * set; the clock tree is driven as 256 * Fs from the MCU's MCLK pin so the
 * codec's own divider programming stays the same. */
static esp_err_t es8311_program_regs(void)
{
    ESP_LOGI(TAG, "ES8311 init sequence begin");

    /* Soft reset + hold, then release. */
    ESP_RETURN_ON_ERROR(es_write(ES8311_RESET_REG00, 0x1F), TAG, "rst0");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(es_write(ES8311_RESET_REG00, 0x00), TAG, "rst1");

    /* Clock: MCLK from MCLK pin, MCLK = 256 * Fs, slave mode. */
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG01, 0x30), TAG, "clk1");
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG02, 0x00), TAG, "clk2");
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG03, 0x10), TAG, "clk3");
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG04, 0x20), TAG, "clk4");
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG05, 0x00), TAG, "clk5");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG0B,      0x00), TAG, "sys0b");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG0C,      0x00), TAG, "sys0c");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG10,      0x1F), TAG, "sys10");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG11,      0x7F), TAG, "sys11");
    ESP_RETURN_ON_ERROR(es_write(ES8311_RESET_REG00,       0x80), TAG, "start");

    /* BCLK divider 0x04 -> BCLK = MCLK / 4 = 64 * Fs, LRCK timing auto. */
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG06, 0x03), TAG, "bclkdiv");
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG07, 0x00), TAG, "lrckh");
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG08, 0xFF), TAG, "lrckl");

    /* Serial audio port: 16-bit I2S slave, both directions. */
    ESP_RETURN_ON_ERROR(es_write(ES8311_SDPIN_REG09,       0x0C), TAG, "sdpin");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SDPOUT_REG0A,      0x0C), TAG, "sdpout");

    /* Power up analog and output path using the vendor 3.3 V reference setup. */
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG10,      0x03), TAG, "sys10run");
    ESP_RETURN_ON_ERROR(es_write(ES8311_CLK_MANAGER_REG01, 0x3F), TAG, "clkall");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG0D,      0x01), TAG, "pwr");

    /* MIC path: MIC1 differential, PGA selected via bsp_audio_set_mic_gain_db(). */
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG14,      0x10), TAG, "mic");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG12,      0x28), TAG, "dacpwr");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG13,      0x00), TAG, "outdrv");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG0E,      0x02), TAG, "adcpwr");
    ESP_RETURN_ON_ERROR(es_write(ES8311_SYSTEM_REG0F,      0x44), TAG, "sys0f");
    ESP_RETURN_ON_ERROR(es_write(ES8311_ADC_REG15,         0x00), TAG, "adchpf");
    ESP_RETURN_ON_ERROR(es_write(ES8311_ADC_REG16,         0x24), TAG, "adcclk");
    ESP_RETURN_ON_ERROR(es_write(ES8311_ADC_REG1B,         0x0A), TAG, "adc1b");
    ESP_RETURN_ON_ERROR(es_write(ES8311_ADC_REG1C,         0x6A), TAG, "adceq");
    ESP_RETURN_ON_ERROR(es_write(ES8311_ADC_REG17,         0xBF), TAG, "adcvol");

    /* DAC path: unmute, DSP src = SDP, default volume ~0 dB. REG 0x37 is the
     * DAC ramp / mute register — bit 6 is DAC_RAM_CLR and MUST be 0, otherwise
     * the DAC output stays cleared and the speaker is silent. Use 0x08 to
     * enable soft ramp only, like the Espressif ES-ADF reference. */
    ESP_RETURN_ON_ERROR(es_write(ES8311_DAC_REG31,         0x00), TAG, "dacdsp");
    ESP_RETURN_ON_ERROR(es_write(ES8311_DAC_REG32,         0xBF), TAG, "dacvol");
    ESP_RETURN_ON_ERROR(es_write(ES8311_DAC_REG37,         0x08), TAG, "dacramp");
    ESP_RETURN_ON_ERROR(es_write(ES8311_GPIO_REG44,        0x00), TAG, "dacsrc");
    ESP_RETURN_ON_ERROR(es_write(ES8311_GP_REG45,          0x00), TAG, "gp");
    ESP_LOGI(TAG, "ES8311 init sequence done");
    return ESP_OK;
}

static esp_err_t i2s_init(uint32_t sample_rate_hz, uint32_t dma_desc_num)
{
    if (s_tx != NULL || s_rx != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // The node keeps a deeper DMA ring for capture/JPEG contention; the gateway
    // uses the default depth. At 16 kHz, 240 frames per descriptor is 15 ms.
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = dma_desc_num;
    /* On TX underrun send zeros, not the last descriptor again: a looped buffer
     * is a buzz and, during a call, an echo the AEC never saw as reference. */
    chan_cfg.auto_clear_after_cb = true;
    s_dma_frame_num = chan_cfg.dma_frame_num;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, &s_rx), TAG, "chan");

    ESP_LOGI(TAG, "I2S0 config: Fs=%" PRIu32 " desc=%" PRIu32
             " ring=%" PRIu32 "ms MCLK=%d BCLK=%d LRCK=%d DOUT=%d DIN=%d",
             sample_rate_hz, dma_desc_num,
             dma_desc_num * chan_cfg.dma_frame_num * 1000U / sample_rate_hz,
             BSP_I2S0_MCLK_GPIO, BSP_I2S0_BCLK_GPIO, BSP_I2S0_LRCK_GPIO,
             BSP_I2S0_DIN_GPIO, BSP_I2S0_DOUT_GPIO);

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = sample_rate_hz,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S0_MCLK_GPIO,
            .bclk = BSP_I2S0_BCLK_GPIO,
            .ws   = BSP_I2S0_LRCK_GPIO,
            .dout = BSP_I2S0_DIN_GPIO,    /* MCU -> codec DSDIN   */
            .din  = BSP_I2S0_DOUT_GPIO,   /* codec ASDOUT -> MCU  */
            .invert_flags = { 0 },
        },
    };
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &std_cfg), TAG, "std tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &std_cfg), TAG, "std rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "en tx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "en rx");
    s_i2s_enabled = true;
    return ESP_OK;
}

static esp_err_t i2s_init_tx_only(uint32_t sample_rate_hz)
{
    if (s_tx != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear_after_cb = true;   /* same underrun policy as the duplex ring */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, NULL), TAG, "chan tx-only");

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = sample_rate_hz,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S0_MCLK_GPIO,
            .bclk = BSP_I2S0_BCLK_GPIO,
            .ws   = BSP_I2S0_LRCK_GPIO,
            .dout = BSP_I2S0_DIN_GPIO,
            .din  = (gpio_num_t)-1,
            .invert_flags = { 0 },
        },
    };
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &std_cfg), TAG, "std tx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "en tx");
    s_i2s_enabled = true;
    return ESP_OK;
}

/* Tear the channels down in an order a concurrent reader/writer survives:
 *  1. clear s_i2s_enabled, so a new bsp_audio_read/write returns at once;
 *  2. disable, which wakes a caller blocked inside the driver (it returns
 *     short);
 *  3. wait for any caller that passed the flag just before step 1 - it is
 *     blocked on the disabled channel's semaphore and comes back when its
 *     bounded timeout expires;
 *  4. only then delete the handles.
 * Deleting first stranded the voice task forever, which showed up as a door
 * station whose microphone went dead at the second call. */
static esp_err_t i2s_deinit(void)
{
    esp_err_t ret = ESP_OK;
    const bool was_enabled = s_i2s_enabled;

    s_i2s_enabled = false;
    if (was_enabled) {
        esp_err_t err_rx = (s_rx != NULL) ? i2s_channel_disable(s_rx) : ESP_OK;
        esp_err_t err_tx = (s_tx != NULL) ? i2s_channel_disable(s_tx) : ESP_OK;
        if (err_rx != ESP_OK) {
            ESP_LOGW(TAG, "I2S RX disable failed: %s", esp_err_to_name(err_rx));
            ret = err_rx;
        }
        if (err_tx != ESP_OK) {
            ESP_LOGW(TAG, "I2S TX disable failed: %s", esp_err_to_name(err_tx));
            if (ret == ESP_OK) ret = err_tx;
        }
    }

    for (uint32_t waited = 0; s_io_users != 0 && waited < BSP_AUDIO_IO_DRAIN_MS;
         waited += 10U) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_io_users != 0) {
        /* Should not happen with the bounded I/O timeout; say so rather than
         * delete under a caller. */
        ESP_LOGE(TAG, "I2S teardown with %d caller(s) still inside the driver",
                 s_io_users);
    }

    if (s_rx != NULL) {
        esp_err_t err = i2s_del_channel(s_rx);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S RX delete failed: %s", esp_err_to_name(err));
            if (ret == ESP_OK) ret = err;
        } else {
            s_rx = NULL;
        }
    }

    if (s_tx != NULL) {
        esp_err_t err = i2s_del_channel(s_tx);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S TX delete failed: %s", esp_err_to_name(err));
            if (ret == ESP_OK) ret = err;
        } else {
            s_tx = NULL;
        }
    }

    return ret;
}

static esp_err_t bsp_audio_init_duplex(uint32_t sample_rate_hz,
                                       uint32_t dma_desc_num)
{
    if (s_audio_ready) return ESP_OK;

    if (s_reconfig_lock == NULL) {
        s_reconfig_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_reconfig_lock != NULL, ESP_ERR_NO_MEM, TAG, "reconfig lock");
    }
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    s_sample_rate_hz = sample_rate_hz;

    /* Register ES8311 on the shared I2C bus. */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_I2C_ADDR_ES8311,
        .scl_speed_hz    = BSP_I2C0_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bsp_i2c_bus(), &dev_cfg, &s_codec),
                        TAG, "add es8311");

    /* Quick presence check to fail early with a readable message. */
    uint8_t probe;
    esp_err_t err = es_read(ES8311_RESET_REG00, &probe);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 not responding on I2C addr 0x%02X (%s)",
                 BSP_I2C_ADDR_ES8311, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "ES8311 responded: REG00=0x%02X", probe);
    es_log_reg(ES8311_CHD1_REGFD,  "CHIPFD");
    es_log_reg(ES8311_CHD2_REGFE,  "CHIPFE");
    es_log_reg(ES8311_CHVER_REGFF, "CHIPFF");

    /* I2S must start clocking *before* the codec programming so the codec
     * can sync its internal PLL / clock manager. */
    ESP_RETURN_ON_ERROR(i2s_init(sample_rate_hz, dma_desc_num), TAG, "i2s");
    s_dma_desc_num = dma_desc_num;
    ESP_RETURN_ON_ERROR(es8311_program_regs(),    TAG, "es8311 regs");
    es_dump_key_regs("after codec init");

    bsp_audio_set_volume(84);       /* -6.5 dB */
    bsp_audio_set_mic_gain_db(24);
    es_dump_key_regs("after volume/mic gain");
    bsp_audio_pa_enable(false);     /* start muted to avoid startup pop */

    s_audio_ready = true;
    ESP_LOGI(TAG, "ES8311 + I2S up, Fs=%" PRIu32 " Hz", sample_rate_hz);
    return ESP_OK;
}

esp_err_t bsp_audio_init(uint32_t sample_rate_hz)
{
    return bsp_audio_init_duplex(sample_rate_hz, 24U);
}

esp_err_t bsp_audio_init_gateway_duplex(uint32_t sample_rate_hz)
{
    return bsp_audio_init_duplex(sample_rate_hz, 6U);
}

esp_err_t bsp_audio_init_playback_only(uint32_t sample_rate_hz)
{
    if (s_audio_ready) return ESP_OK;

    if (s_reconfig_lock == NULL) {
        s_reconfig_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_reconfig_lock != NULL, ESP_ERR_NO_MEM, TAG, "reconfig lock");
    }
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    s_sample_rate_hz = sample_rate_hz;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_I2C_ADDR_ES8311,
        .scl_speed_hz    = BSP_I2C0_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bsp_i2c_bus(), &dev_cfg, &s_codec),
                        TAG, "add es8311");

    uint8_t probe;
    esp_err_t err = es_read(ES8311_RESET_REG00, &probe);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 not responding on I2C addr 0x%02X (%s)",
                 BSP_I2C_ADDR_ES8311, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "ES8311 responded: REG00=0x%02X", probe);

    ESP_RETURN_ON_ERROR(i2s_init_tx_only(sample_rate_hz), TAG, "i2s tx-only");
    ESP_RETURN_ON_ERROR(es8311_program_regs(), TAG, "es8311 regs");

    bsp_audio_set_volume(84);       /* -6.5 dB */
    bsp_audio_pa_enable(false);

    s_audio_ready = true;
    ESP_LOGI(TAG, "ES8311 + I2S (playback-only) up, Fs=%" PRIu32 " Hz", sample_rate_hz);
    return ESP_OK;
}

esp_err_t bsp_audio_pa_enable(bool on)
{
    return bsp_ioexp_set_pin(BSP_IO_EXP_PA_MUTE_PIN, on);
}

uint32_t bsp_audio_tx_ring_ms(void)
{
    if (!s_audio_ready || s_sample_rate_hz == 0U) return 0U;
    return s_dma_desc_num * s_dma_frame_num * 1000U / s_sample_rate_hz;
}

static void reconfig_lock(void)
{
    if (s_reconfig_lock != NULL) xSemaphoreTake(s_reconfig_lock, portMAX_DELAY);
}

static void reconfig_unlock(void)
{
    if (s_reconfig_lock != NULL) xSemaphoreGive(s_reconfig_lock);
}

/* Build the duplex channels at the given depth from whatever is there now:
 * running, half-built (the IDF keeps the first channel when the second one
 * fails), or nothing. Caller holds the reconfig lock. */
static esp_err_t i2s_rebuild_locked(uint32_t dma_desc_num)
{
    if (s_tx != NULL || s_rx != NULL) {
        esp_err_t err = i2s_deinit();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S teardown before rebuild failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    esp_err_t err = i2s_init(s_sample_rate_hz, dma_desc_num);
    if (err != ESP_OK) {
        (void)i2s_deinit();
    }
    return err;
}

esp_err_t bsp_audio_suspend(void)
{
    if (!s_audio_ready) return ESP_OK;

    reconfig_lock();
    esp_err_t err = ESP_OK;
    if (s_tx != NULL || s_rx != NULL) {
        (void)bsp_audio_pa_enable(false);
        err = i2s_deinit();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2S released for camera capture");
        }
    }
    reconfig_unlock();
    return err;
}

esp_err_t bsp_audio_resume(void)
{
    if (!s_audio_ready) return ESP_OK;

    reconfig_lock();
    esp_err_t err = ESP_OK;
    if (!s_i2s_enabled) {
        err = i2s_rebuild_locked(s_dma_desc_num);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2S rebuilt at depth %" PRIu32, s_dma_desc_num);
        } else {
            ESP_LOGE(TAG, "I2S rebuild at depth %" PRIu32 " failed: %s",
                     s_dma_desc_num, esp_err_to_name(err));
        }
    }
    reconfig_unlock();
    return err;
}

esp_err_t bsp_audio_rebuild(void)
{
    if (!s_audio_ready) return ESP_ERR_INVALID_STATE;

    reconfig_lock();
    esp_err_t err = i2s_rebuild_locked(s_dma_desc_num);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "I2S force-rebuilt at depth %" PRIu32, s_dma_desc_num);
    } else {
        ESP_LOGE(TAG, "I2S force-rebuild at depth %" PRIu32 " failed: %s",
                 s_dma_desc_num, esp_err_to_name(err));
    }
    reconfig_unlock();
    return err;
}

esp_err_t bsp_audio_set_dma_desc_num(uint32_t dma_desc_num)
{
    if (!s_audio_ready) return ESP_ERR_INVALID_STATE;
    if (dma_desc_num == 0U) return ESP_ERR_INVALID_ARG;

    reconfig_lock();
    if (dma_desc_num == s_dma_desc_num) {
        reconfig_unlock();
        return ESP_OK;
    }

    /* I2S released (camera capture holds it): just record the new depth so the
     * next bsp_audio_resume() rebuilds at it, without touching hardware now. */
    if (!s_i2s_enabled) {
        s_dma_desc_num = dma_desc_num;
        ESP_LOGI(TAG, "I2S DMA depth deferred to %" PRIu32 " (I2S released)",
                 dma_desc_num);
        reconfig_unlock();
        return ESP_OK;
    }

    esp_err_t err = i2s_rebuild_locked(dma_desc_num);
    if (err != ESP_OK) {
        /* Usually ESP_ERR_NO_MEM: the deep ring is ~92 KB of internal DMA RAM.
         * Do not leave the channels torn down - nothing on the door station
         * would ever rebuild them and the microphone would stay dead until a
         * reboot. Go back to the depth that was just running. */
        ESP_LOGE(TAG, "I2S reinit at depth %" PRIu32 " failed: %s, restoring depth %" PRIu32,
                 dma_desc_num, esp_err_to_name(err), s_dma_desc_num);
        esp_err_t back = i2s_rebuild_locked(s_dma_desc_num);
        if (back != ESP_OK) {
            ESP_LOGE(TAG, "I2S restore at depth %" PRIu32 " failed too: %s",
                     s_dma_desc_num, esp_err_to_name(back));
        }
        reconfig_unlock();
        return err;
    }
    s_dma_desc_num = dma_desc_num;
    ESP_LOGI(TAG, "I2S DMA depth changed to %" PRIu32, dma_desc_num);
    reconfig_unlock();
    return ESP_OK;
}

esp_err_t bsp_audio_set_volume(uint8_t level)
{
    if (level > BSP_AUDIO_VOLUME_MAX) level = BSP_AUDIO_VOLUME_MAX;
    /* Reg 0x32 is a dB code, 0.5 dB per step: 0x00 = -95.5 dB, 0xBF = 0 dB,
     * 0xFF = +32 dB (digital gain, clips). 0..120 spans the whole code range,
     * so every 8 is 17 codes = 8.5 dB: 84 = -6.5 dB, 96 = +6.5 dB, 120 = +32 dB. */
    uint8_t v = (uint8_t)(level * 255 / BSP_AUDIO_VOLUME_MAX);
    esp_err_t err = es_write(ES8311_DAC_REG32, v);
    if (err == ESP_OK) s_volume_level = level;
    return err;
}

uint8_t bsp_audio_get_volume(void)
{
    return s_volume_level;
}

esp_err_t bsp_audio_set_mic_gain_db(uint8_t gain_db)
{
    /* Vendor reference uses REG14 low bits for analog PGA gain, while REG16 is
     * part of the sample-rate clock setup and must not be overwritten here. */
    uint8_t bits = gain_db / 3;
    if (bits > 10) bits = 10;
    return es_write(ES8311_SYSTEM_REG14, 0x10 | bits);
}

/* The user count is raised before the enabled check so i2s_deinit() sees a
 * caller that read the flag as true a moment before it was cleared. */
esp_err_t bsp_audio_write(const void *buf, size_t bytes, size_t *out_written)
{
    if (!s_audio_ready) return ESP_ERR_INVALID_STATE;
    esp_err_t err = ESP_ERR_INVALID_STATE;
    __atomic_add_fetch(&s_io_users, 1, __ATOMIC_SEQ_CST);
    if (s_i2s_enabled && s_tx != NULL) {
        err = i2s_channel_write(s_tx, buf, bytes, out_written, BSP_AUDIO_IO_TIMEOUT_MS);
    }
    __atomic_sub_fetch(&s_io_users, 1, __ATOMIC_SEQ_CST);
    return err;
}

esp_err_t bsp_audio_read(void *buf, size_t bytes, size_t *out_read)
{
    if (!s_audio_ready) return ESP_ERR_INVALID_STATE;
    esp_err_t err = ESP_ERR_INVALID_STATE;
    __atomic_add_fetch(&s_io_users, 1, __ATOMIC_SEQ_CST);
    if (s_i2s_enabled && s_rx != NULL) {
        err = i2s_channel_read(s_rx, buf, bytes, out_read, BSP_AUDIO_IO_TIMEOUT_MS);
    }
    __atomic_sub_fetch(&s_io_users, 1, __ATOMIC_SEQ_CST);
    return err;
}

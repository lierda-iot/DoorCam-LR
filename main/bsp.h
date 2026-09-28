#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Shared I2C0 bus + TCA9554A GPIO expander -------------------- */
esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_reinit(void);
i2c_master_bus_handle_t bsp_i2c_bus(void);

/* Probe each 7-bit address on I2C0 and log the ones that ACK. Handy for
 * bringing up a new board. Returns ESP_OK if the scan ran; the caller can
 * read the log output to see which addresses responded. */
esp_err_t bsp_i2c_scan(void);

typedef enum {
    BSP_CON6_PERIPHERAL_LCD_ST7789 = 0,
    BSP_CON6_PERIPHERAL_CAMERA,
} bsp_con6_peripheral_t;

/* Detect what is attached to CON6.  Camera has priority when its I2C address
 * ACKs; otherwise the connector is treated as the ST7789T3 LCD variant. */
esp_err_t bsp_con6_detect(bsp_con6_peripheral_t *out_peripheral);

/* Set a single pin on the TCA9554A expander (configures it as output if it
 * is not already). Pins used by the BSP: P0/P1/P2 = RGB LED, P3 = LCD reset,
 * P6 = PA enable. */
esp_err_t bsp_ioexp_set_pin(uint8_t pin, bool level);

/* ---------- LCD (ST7789V3 on V02 20-pin FPC) ---------------------------- */
typedef void (*bsp_lcd_capture_cb_t)(void *user);

esp_err_t bsp_lcd_init(void);
esp_err_t bsp_lcd_release_for_camera(void);
esp_err_t bsp_lcd_reinit_after_camera(void);
esp_err_t bsp_lcd_show_test_pattern(void);
esp_err_t bsp_lcd_start_lvgl_demo(void);
esp_err_t bsp_lcd_start_camera_ui(bsp_lcd_capture_cb_t cb, void *user);
esp_err_t bsp_lcd_start_gateway_ui(void);
SemaphoreHandle_t bsp_lcd_get_lvgl_lock(void);
/* Push one frame straight to the panel, bypassing LVGL. `y_offset` is the first
 * panel row the frame occupies, so the caller can reserve a strip above it (the
 * gateway keeps the top rows for its status bar). The occupied rows are recorded
 * and become the region bsp_lcd_set_video_direct_owner() protects. */
esp_err_t bsp_lcd_present_video_frame(const uint16_t *rgb565,
                                      uint32_t width,
                                      uint32_t height,
                                      uint32_t y_offset);
/* While a video stream owns the panel, LVGL redraws overlapping the video rows
 * are dropped (the next present overwrites them anyway); redraws entirely
 * outside them, such as the status strip, still go through. */
void bsp_lcd_set_video_direct_owner(bool owns_panel);
/* Flush the top status strip at frame completion. Caller holds the LVGL lock. */
void bsp_lcd_refresh_video_status(void);
esp_err_t bsp_lcd_set_camera_status(const char *text);
esp_err_t bsp_lcd_clear_camera_photo(void);
esp_err_t bsp_lcd_show_gray_photo(const uint8_t *gray,
                                  uint32_t width,
                                  uint32_t height);
esp_err_t bsp_lcd_show_yuv422_photo(const uint8_t *yuv422,
                                    uint32_t width,
                                    uint32_t height,
                                    uint32_t pixelformat);
esp_err_t bsp_lcd_show_rgb565_photo(const uint16_t *rgb565,
                                    uint32_t width,
                                    uint32_t height);

/* ---------- RGB LED ----------------------------------------------------- */
esp_err_t bsp_led_init(void);
void      bsp_led_set(bool r, bool g, bool b);

/* ---------- Audio (ES8311 codec + CST8302A PA + I2S) -------------------- */
esp_err_t bsp_audio_init(uint32_t sample_rate_hz);
esp_err_t bsp_audio_init_gateway_duplex(uint32_t sample_rate_hz);
esp_err_t bsp_audio_init_playback_only(uint32_t sample_rate_hz);
esp_err_t bsp_audio_pa_enable(bool on);                  /* PA enable via P6 */

/* Depth of the I2S TX DMA ring in ms (0 if audio is not up). bsp_audio_write()
 * returns once samples reach the ring, not the speaker, so push at least this
 * much silence before cutting the PA. The depth differs per board role. */
uint32_t  bsp_audio_tx_ring_ms(void);
/* Release the I2S (camera capture needs its DMA) and bring it back. resume()
 * also repairs a channel pair left half-built by a failed rebuild; it is a
 * no-op while I2S is up. rebuild() tears down whatever is there, even a
 * running pair, and builds it again at the current depth: the recovery for
 * a reader that keeps failing. All three may be called from any task; a
 * concurrent bsp_audio_read/write returns short and must be retried. */
esp_err_t bsp_audio_suspend(void);
esp_err_t bsp_audio_resume(void);
esp_err_t bsp_audio_rebuild(void);
/* Rebuild the duplex I2S at a new DMA ring depth (a shallow ring gives the
 * AEC a stable speaker-path latency during a call). No-op if the depth is
 * already active; if the I2S is released it is recorded for the next resume.
 * If the new depth cannot be built the old one is restored and the error
 * returned. */
esp_err_t bsp_audio_set_dma_desc_num(uint32_t dma_desc_num);
/* DAC output level: 0 mutes, BSP_AUDIO_VOLUME_MAX is the codec maximum. */
#define BSP_AUDIO_VOLUME_MAX  120
esp_err_t bsp_audio_set_volume(uint8_t level);           /* 0..120 for DAC  */
uint8_t   bsp_audio_get_volume(void);                    /* last value set  */
esp_err_t bsp_audio_set_mic_gain_db(uint8_t gain_db);    /* 0..42 PGA       */
esp_err_t bsp_audio_write(const void *buf, size_t bytes, size_t *out_written);
esp_err_t bsp_audio_read (void       *buf, size_t bytes, size_t *out_read);

/* ---------- Buttons ----------------------------------------------------- */
typedef enum {
    BSP_BTN_BOOT = 0,   /* Independent GPIO0 strap button */
    BSP_BTN_USER1,      /* K2 -> ~1.65 V on KEY_ADC    */
    BSP_BTN_VOL_DN,     /* K5 -> ~1.11 V               */
    BSP_BTN_PTT,        /* K1 -> ~0.82 V               */
    BSP_BTN_VOL_UP,     /* K4 -> ~2.41 V               */
    BSP_BTN_COUNT,
} bsp_btn_id_t;

typedef void (*bsp_btn_cb_t)(bsp_btn_id_t id, bool pressed, void *user);

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user);

/* Sample the KEY_ADC ladder once, right now, and return the key that is down
 * (BSP_BTN_COUNT if none / ambiguous). Bypasses the polling task's debounce,
 * so it is only for callers that already know a key edge happened — e.g.
 * identifying which key just woke the chip out of light sleep. */
bsp_btn_id_t bsp_button_sample_adc(void);

/* Arm / disarm the KEY_ADC line as a LOW_LEVEL light-sleep wake source. Hands
 * the pad from ADC1 to the digital GPIO peripheral; the caller must disarm
 * after waking, before the polling task's next ADC read.
 *
 * CAVEAT: outside the ESP32-S3 DC spec. V_IL is guaranteed only up to 0.25*VDD
 * (0.825 V) while the ladder steps sit at 0.82 V and above; the Schmitt input
 * does wake in practice, but this is measured, not guaranteed. Always confirm
 * with bsp_button_sample_adc() after the wake. */
esp_err_t bsp_button_arm_sleep_wakeup(void);
void bsp_button_disarm_sleep_wakeup(void);

/* ---------- Battery / supply voltage --------------------------------------
 * Sample VBAT_ADC (GPIO11 / ADC2_CH0) once and return the real supply voltage
 * in millivolts, i.e. the divider is already compensated (reading * 2). Returns
 * a negative esp_err_t on failure (e.g. ESP_ERR_TIMEOUT when Wi-Fi owns ADC2).
 */
int bsp_vbat_read_mv(void);

/* Return the most recent successful voltage reading cached by bsp_vbat_read_mv(),
 * or 0 if no valid reading has been taken yet. Safe to call from the radio path
 * without touching ADC2 (lock-free read). */
uint16_t bsp_vbat_get_cached(void);

/* Background task sampling the voltage every `period_ms` (0 = 15000 ms
 * default) to keep bsp_vbat_get_cached() fresh. */
esp_err_t bsp_vbat_monitor_start(uint32_t period_ms);

#ifdef __cplusplus
}
#endif

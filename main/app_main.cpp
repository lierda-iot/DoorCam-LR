#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "image_transfer.hpp"
#include "radio_ping.hpp"
#include "opus_codec.hpp"
#include "ui_gateway.h"
#include "visitor_store.h"

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_jpeg_common.h"
#include "esp_jpeg_dec.h"

#include <stdio.h>
#include <new>

#include "app_config.h"
#include "bsp.h"

#include <atomic>

volatile bool g_low_power_enabled = false;

namespace {
constexpr const char *TAG = "app";
constexpr const char *kNvsNs = "app";
constexpr const char *kModeKey = "mode";

void log_heap_state(const char *stage)
{
    constexpr uint32_t internal_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    constexpr uint32_t dma_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA |
                                  MALLOC_CAP_8BIT;
    ESP_LOGI(TAG,
             "[HEAP] %s internal free=%u largest=%u min=%u | dma free=%u largest=%u",
             stage,
             static_cast<unsigned>(heap_caps_get_free_size(internal_caps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(internal_caps)),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(internal_caps)),
             static_cast<unsigned>(heap_caps_get_free_size(dma_caps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(dma_caps)));
}

enum class AppMode : uint8_t {
    camera = 0,
    radio = 1,
};

AudioDiagnostics g_audio;
CameraUartStreamer g_camera_uart;
RadioPing g_radio;
std::atomic<bool> g_capture_busy{false};
std::atomic<uint16_t> g_active_capture_session{0};
AppMode g_app_mode = AppMode::camera;
bool g_radio_active = false;

#if APP_DOORBELL_ALSO_CAPTURE
// Session ids for the capture the doorbell key fires alongside its ring.
// 0x8000.. keeps clear of the gateway's request counter; the radio's PIR path
// has its own range at 0xC000.
uint16_t g_auto_session_id = 0x8000;
#endif

// K1 short/long press state
int64_t g_ptt_press_time_us = 0;
bool g_ptt_held_long = false;
esp_timer_handle_t g_ptt_timer = nullptr;

constexpr UBaseType_t kGatewayImageQueueLength = 2;
constexpr uint32_t kGatewayImageTaskStackBytes = 16384U;
constexpr UBaseType_t kGatewayImageTaskPriority = 3;
// Keep image decode on CPU0 below the radio task.
constexpr BaseType_t kGatewayImageTaskCore = 0;

struct GatewayImageFrame {
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    uint32_t transfer_ms = 0;
    uint32_t reassemble_ms = 0;
    uint32_t queued_ms = 0;
    uint16_t session_id = 0;
    uint16_t fragments = 0;
    bool from_intercom = false;  // in-call frame; bypass the stream-active gate
    bool from_playback = false;  // visitor record frame: gated on the playback page instead
    // The node's PIR snapshot, pushed unasked: decoded for its thumbnail and
    // filed in the visitor store, never put on the panel.
    bool from_snapshot = false;
};

QueueHandle_t g_gateway_image_queue = nullptr;
TaskHandle_t g_gateway_image_task_handle = nullptr;

// Live-intercom capture/JPEG is elastic work. Keep it below the radio task and
// its slot-tail burst while moving the load away from the saturated CPU1.
constexpr UBaseType_t kIntercomImageTaskPriority = 1;
constexpr BaseType_t kIntercomImageTaskCore = 0;
constexpr uint32_t kIntercomImageFramePeriodMs =
    APP_INTERCOM_IMAGE_REAL_CAPTURE_MS;

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
constexpr uint32_t kCpuStatsPeriodMs = 5000U;
constexpr uint32_t kCpuStatsTaskStackBytes = 3072U;
constexpr UBaseType_t kCpuStatsTaskPriority = 1;
constexpr BaseType_t kCpuStatsTaskCore = 0;

struct TaskRuntimeProbe {
    const char *name;
    uint32_t previous = 0;
    bool primed = false;
};

uint32_t read_task_runtime(TaskHandle_t handle)
{
    if (!handle) return 0;
    TaskStatus_t status = {};
    vTaskGetInfo(handle, &status, pdFALSE, eReady);
    return static_cast<uint32_t>(status.ulRunTimeCounter);
}

uint32_t sample_task_runtime(TaskHandle_t handle, uint32_t *previous, bool *primed)
{
    if (!handle) {
        *primed = false;
        return 0;
    }
    const uint32_t current = read_task_runtime(handle);
    if (!*primed) {
        *previous = current;
        *primed = true;
        return 0;
    }
    const uint32_t delta = current - *previous;
    *previous = current;
    return delta;
}

uint32_t runtime_percent_x10(uint32_t runtime_us, uint64_t elapsed_us)
{
    if (elapsed_us == 0) return 0;
    const uint64_t value = static_cast<uint64_t>(runtime_us) * 1000ULL / elapsed_us;
    return static_cast<uint32_t>(value > 1000ULL ? 1000ULL : value);
}

void cpu_stats_task(void *arg)
{
    (void)arg;
    TaskRuntimeProbe idle0 = {nullptr};
    TaskRuntimeProbe idle1 = {nullptr};
    TaskRuntimeProbe probes[] = {
        {"radio_ping"}, {"voice_tx"}, {"voice_play"},
        {"img_feed"}, {"gw_image"}, {"lvgl"},
    };

    const TaskHandle_t idle0_handle = xTaskGetIdleTaskHandleForCore(0);
    const TaskHandle_t idle1_handle = xTaskGetIdleTaskHandleForCore(1);
    (void)sample_task_runtime(idle0_handle, &idle0.previous, &idle0.primed);
    (void)sample_task_runtime(idle1_handle, &idle1.previous, &idle1.primed);
    for (auto &probe : probes) {
        (void)sample_task_runtime(xTaskGetHandle(probe.name),
                                  &probe.previous, &probe.primed);
    }
    int64_t previous_us = esp_timer_get_time();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kCpuStatsPeriodMs));
        const int64_t now_us = esp_timer_get_time();
        const uint64_t elapsed_us = static_cast<uint64_t>(now_us - previous_us);
        previous_us = now_us;

        const uint32_t idle0_delta = sample_task_runtime(
            idle0_handle, &idle0.previous, &idle0.primed);
        const uint32_t idle1_delta = sample_task_runtime(
            idle1_handle, &idle1.previous, &idle1.primed);
        uint32_t task_pct_x10[sizeof(probes) / sizeof(probes[0])] = {};
        for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); ++i) {
            const uint32_t delta = sample_task_runtime(
                xTaskGetHandle(probes[i].name),
                &probes[i].previous, &probes[i].primed);
            task_pct_x10[i] = runtime_percent_x10(delta, elapsed_us);
        }

        const uint32_t idle0_pct_x10 = runtime_percent_x10(idle0_delta, elapsed_us);
        const uint32_t idle1_pct_x10 = runtime_percent_x10(idle1_delta, elapsed_us);
        const uint32_t core0_busy_x10 = 1000U - idle0_pct_x10;
        const uint32_t core1_busy_x10 = 1000U - idle1_pct_x10;
        const uint32_t elapsed_ms = static_cast<uint32_t>(elapsed_us / 1000ULL);

        ESP_LOGI(TAG,
                 "[CPU] win=%lums core0=%lu.%01lu%% core1=%lu.%01lu%% | "
                 "radio=%lu.%01lu voice_tx=%lu.%01lu voice_play=%lu.%01lu "
                 "img_feed=%lu.%01lu gw_image=%lu.%01lu lvgl=%lu.%01lu",
                 static_cast<unsigned long>(elapsed_ms),
                 static_cast<unsigned long>(core0_busy_x10 / 10U),
                 static_cast<unsigned long>(core0_busy_x10 % 10U),
                 static_cast<unsigned long>(core1_busy_x10 / 10U),
                 static_cast<unsigned long>(core1_busy_x10 % 10U),
                 static_cast<unsigned long>(task_pct_x10[0] / 10U),
                 static_cast<unsigned long>(task_pct_x10[0] % 10U),
                 static_cast<unsigned long>(task_pct_x10[1] / 10U),
                 static_cast<unsigned long>(task_pct_x10[1] % 10U),
                 static_cast<unsigned long>(task_pct_x10[2] / 10U),
                 static_cast<unsigned long>(task_pct_x10[2] % 10U),
                 static_cast<unsigned long>(task_pct_x10[3] / 10U),
                 static_cast<unsigned long>(task_pct_x10[3] % 10U),
                 static_cast<unsigned long>(task_pct_x10[4] / 10U),
                 static_cast<unsigned long>(task_pct_x10[4] % 10U),
                 static_cast<unsigned long>(task_pct_x10[5] / 10U),
                 static_cast<unsigned long>(task_pct_x10[5] % 10U));
    }
}
#endif

const char *mode_name(AppMode mode)
{
    return mode == AppMode::radio ? "radio" : "camera";
}

const char *short_error_name(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_NO_MEM:
        return "NO_MEM";
    case ESP_ERR_TIMEOUT:
        return "TIMEOUT";
    case ESP_ERR_INVALID_SIZE:
        return "BAD_SIZE";
    case ESP_ERR_NOT_SUPPORTED:
        return "NOT_SUP";
    default:
        return nullptr;
    }
}

void init_nvs()
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
}

AppMode load_app_mode()
{
    nvs_handle_t nvs;
    uint8_t value = static_cast<uint8_t>(AppMode::camera);
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u8(nvs, kModeKey, &value);
        nvs_close(nvs);
    }
    return value == static_cast<uint8_t>(AppMode::radio) ? AppMode::radio : AppMode::camera;
}

void save_app_mode(AppMode mode)
{
    nvs_handle_t nvs;
    esp_err_t e = nvs_open(kNvsNs, NVS_READWRITE, &nvs);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "open mode nvs: %s", esp_err_to_name(e));
        return;
    }
    e = nvs_set_u8(nvs, kModeKey, static_cast<uint8_t>(mode));
    if (e == ESP_OK) e = nvs_commit(nvs);
    nvs_close(nvs);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "save mode nvs: %s", esp_err_to_name(e));
    }
}

void save_config_u8(const char *key, uint8_t val)
{
    nvs_handle_t nvs;
    if (nvs_open(kNvsNs, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u8(nvs, key, val);
    nvs_commit(nvs);
    nvs_close(nvs);
}

uint8_t load_config_u8(const char *key, uint8_t def)
{
    nvs_handle_t nvs;
    uint8_t val = def;
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u8(nvs, key, &val);
        nvs_close(nvs);
    }
    return val;
}

bool on_image_capture_request(uint16_t session_id);
esp_err_t gateway_image_pipeline_init();
void on_gw_rx_abort(void);
bool gateway_image_queue_push(const GatewayImageFrame &frame);
void gateway_image_frame_free(GatewayImageFrame *frame);

// One-shot timer that re-arms the PIR trigger after a detection. Must be an
// esp_timer, not a FreeRTOS xTimer: the FreeRTOS tick is frozen during light
// sleep, while esp_timer keeps counting wall-clock time.
static esp_timer_handle_t g_pir_rearm_timer = nullptr;

// PIR uses GPIO_INTR_HIGH_LEVEL (the sensor holds the pin high for seconds,
// and level mode is the only ESP32-S3 light-sleep GPIO wake mode). A level
// trigger left enabled would storm the interrupt watchdog, so the ISR disables
// the source and the one-shot timer re-arms it; that delay is also the capture
// cooldown.
static void IRAM_ATTR pir_isr_handler(void *arg)
{
    // Kill the level trigger NOW so it can't re-fire while GPIO12 stays high.
    gpio_intr_disable(APP_PIR_GPIO);
    static_cast<RadioPing *>(arg)->set_pir_armed(false);
    static_cast<RadioPing *>(arg)->pir_trigger();
    if (g_pir_rearm_timer != nullptr) {
        // esp_timer_start_once is ISR-safe; esp_timer_stop is not, so the
        // disabled trigger above is what prevents re-entry.
        esp_timer_start_once(g_pir_rearm_timer,
                             (uint64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000ULL);
    }
}

// Re-arm the PIR level trigger 15s after the last detection.
static void pir_rearm_timer_cb(void *arg)
{
    gpio_intr_disable(APP_PIR_GPIO);
    gpio_set_intr_type(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
    static_cast<RadioPing *>(arg)->set_pir_armed(true);
    gpio_intr_enable(APP_PIR_GPIO);
    ESP_LOGI(TAG, "PIR: GPIO%d high-level re-armed", APP_PIR_GPIO);
}

void pir_arm_timer_cb(void *arg)
{
    if (g_pir_rearm_timer == nullptr) {
        const esp_timer_create_args_t rearm_args = {
            .callback = pir_rearm_timer_cb,
            .arg = arg,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pir_rearm",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&rearm_args, &g_pir_rearm_timer);
    }
    gpio_intr_disable(APP_PIR_GPIO);
    gpio_isr_handler_add(APP_PIR_GPIO, pir_isr_handler, arg);
    gpio_set_intr_type(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
    static_cast<RadioPing *>(arg)->set_pir_armed(true);
    gpio_intr_enable(APP_PIR_GPIO);
    ESP_LOGI(TAG, "PIR: GPIO%d high-level armed", APP_PIR_GPIO);
}

void on_config_received(uint8_t key, uint32_t value)
{
    if (key == APP_CFG_KEY_INTER_PACKET) {
        g_radio.set_inter_packet_us(value);
        ESP_LOGI(TAG, "config: inter_packet=%luus", static_cast<unsigned long>(value));
        char buf[16];
        snprintf(buf, sizeof(buf), "%luus", static_cast<unsigned long>(value));
        bsp_lcd_set_camera_status(buf);
    } else if (key == APP_CFG_KEY_PIR_TRIGGER) {
        g_radio.set_pir_enabled(value != 0);
        save_config_u8("pir", value ? 1 : 0);
        ESP_LOGI(TAG, "config: pir_trigger=%s", value ? "on" : "off");
    } else if (key == APP_CFG_KEY_LOW_POWER) {
        g_low_power_enabled = (value != 0);
        save_config_u8("lowpwr", value ? 1 : 0);
        ESP_LOGI(TAG, "config: low_power=%s", value ? "on" : "off");
    } else if (key == APP_CFG_KEY_INTERCOM) {
        ESP_LOGI(TAG, "config: intercom prepare session=%lu",
                 static_cast<unsigned long>(value));
    } else if (key == APP_CFG_KEY_JPEG_QUALITY) {
        // Clamped by the setter; store what was applied, not what was sent.
        g_radio.image_xfer().set_jpeg_quality(
            static_cast<uint8_t>(value > 255U ? 255U : value));
        const uint8_t applied = g_radio.image_xfer().jpeg_quality();
        save_config_u8("jpegq", applied);
        ESP_LOGI(TAG, "config: jpeg_quality=%u", applied);
    } else {
        // Ignore unsupported configuration keys.
        ESP_LOGW(TAG, "config: key=%u not supported, ignored", key);
    }
}

// Node low power: called by the radio when entering CAD sleep standby. Release
// the camera to save power; capture_frame() rebuilds it on the next capture.
bool on_low_power_standby(bool entering)
{
    if (!entering) return true;
    // Never release mid-capture (DVP and its DMA buffers are in use); the
    // radio asks again on the next CAD cycle.
    if (g_capture_busy) return false;
    g_camera_uart.low_power_standby();
    return true;
}

// Gateway visitor session: press -> ring + show the door for
// APP_DOORBELL_RING_MS -> answered with the capture key or timed out. The
// radio task opens it on the packet and closes it at the deadline; the answer
// or a navigation key closes it early through abort_image_rx(). A press while
// it is open is ignored. Node audio in the stream is recorded, not played.
enum class DoorbellSession : uint8_t { idle, ringing };
static std::atomic<DoorbellSession> s_doorbell_session{DoorbellSession::idle};

// Gateway, radio task: a doorbell packet passed the de-dup window. Hands the
// ring to the audio task and queues the stream start.
void on_doorbell_ring()
{
    if (g_app_mode != AppMode::radio) return;

    if (s_doorbell_session.load(std::memory_order_acquire) != DoorbellSession::idle) {
        ESP_LOGI(TAG, "doorbell: press ignored, visitor session in progress");
        return;
    }

    // A ring while a stream is up replaces it: two writers on one I2S path is
    // noise, and the stream's playback timeout would cut the PA mid-chime.
    const bool preempted = ui_gw_doorbell_preempt();
    const uint32_t ring_delay_ms =
        preempted ? APP_DOORBELL_PREEMPT_CHIME_DELAY_MS : 0U;

    ESP_LOGI(TAG, "doorbell: ringing%s", preempted ? " (stream pre-empted)" : "");
    if (!g_audio.request_doorbell_ring(APP_DOORBELL_RING_MS, ring_delay_ms)) {
        return;   // a call owns the speaker, or a ring is still sounding
    }
    s_doorbell_session.store(DoorbellSession::ringing, std::memory_order_release);
    ui_gw_note_event();
    // Open the visitor record before the first frame can arrive. A ring that
    // gets no memory simply goes unrecorded (logged by the store).
    (void)visitor_store_begin();
    g_radio.doorbell_stream_request(APP_DOORBELL_RING_MS);
}

// Gateway (radio task): request the ring-time picture through the UI.
bool on_doorbell_stream_start(void)
{
    const bool ok = ui_gw_doorbell_stream_begin();
    ESP_LOGI(TAG, "doorbell: visitor stream %s", ok ? "started" : "NOT started");
    return ok;
}

// Gateway: the visitor session is over. On expiry this stops the stream via
// the UI path; otherwise the user already left or answered.
void on_doorbell_stream_end(bool expired)
{
    g_audio.stop_doorbell_ring();
    if (expired) {
        ESP_LOGI(TAG, "doorbell: not answered, ring time is up");
        // Unanswered: the ring becomes a visitor record, committed before the
        // stream is torn down.
        if (visitor_store_commit()) {
            ui_gw_note_unseen();
            ui_gw_records_changed();
        }
        ui_gw_doorbell_ring_end();
    } else {
        // Answered, or dismissed with a key: the visitor was dealt with, so
        // nothing is kept (user decision, step 3).
        visitor_store_discard();
    }
    s_doorbell_session.store(DoorbellSession::idle, std::memory_order_release);
}

// Visitor record playback (store task -> live pipeline). The decode queue
// frees what it is given, so it gets a copy.
void on_visitor_play_frame(const uint8_t *jpeg, size_t len)
{
    if (!jpeg || len == 0 || !ui_gw_playback_active()) return;
    uint8_t *copy = static_cast<uint8_t *>(
        heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!copy) {
        ESP_LOGW(TAG, "playback: no memory for a %u byte frame copy",
                 static_cast<unsigned>(len));
        return;
    }
    memcpy(copy, jpeg, len);
    GatewayImageFrame frame = {};
    frame.jpeg = copy;
    frame.jpeg_len = len;
    frame.from_playback = true;
    frame.queued_ms = static_cast<uint32_t>(esp_log_timestamp());
    if (!gateway_image_queue_push(frame)) {
        gateway_image_frame_free(&frame);
    }
}

void on_visitor_play_audio(const uint8_t *blob, size_t len, uint16_t first_seq)
{
    if (!ui_gw_playback_active()) return;
    g_radio.play_stored_audio(blob, len, first_seq);
}

void on_visitor_play_done(bool completed)
{
    // Stopped mid-way: drop what is still queued for the speaker. A record
    // that ran to its end drains its last frames.
    if (!completed) {
        g_radio.flush_playback_audio();
    }
    ui_gw_playback_ended();
}

void on_intercom_state(bool active)
{
    g_audio.set_intercom_active(active);
    if (g_app_mode == AppMode::radio) {
        // Gateway already runs the shallow 6-desc duplex ring; only refresh UI.
        ui_gw_set_intercom_active(active);
    } else {
        // Node: shrink the DMA ring for the call so speaker-path latency is
        // stable enough for the AEC to lock; restore the deep ring on hangup.
        const uint32_t desc = active ? APP_INTERCOM_NODE_DMA_DESC_NUM
                                     : APP_AUDIO_NODE_DEFAULT_DMA_DESC_NUM;
        esp_err_t e = bsp_audio_set_dma_desc_num(desc);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "intercom DMA depth -> %u failed: %s",
                     static_cast<unsigned>(desc), esp_err_to_name(e));
        }
    }
}

void switch_mode_and_restart()
{
    AppMode next = g_app_mode == AppMode::camera ? AppMode::radio : AppMode::camera;
    ESP_LOGW(TAG, "mode switch: %s -> %s, restarting",
             mode_name(g_app_mode), mode_name(next));
    save_app_mode(next);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

// Gateway K1 long press selects the door-station role on release.
void ptt_long_press_cb(void *arg)
{
    (void)arg;
    g_ptt_held_long = true;
}

// Door-station capture task, started after an ImageCmd request.
struct ImageCaptureCtx {
    uint16_t session_id;
};

// --- Frame prefetch (pipelining) --------------------------------------------
// Capture + JPEG (~100 ms, CPU) and the radio push (~140 ms, SPI + air) use
// independent hardware, so frame N+1 is captured and encoded into this cache
// while frame N is transmitted, and sent as soon as the next ImageCmd arrives.
static SemaphoreHandle_t g_prefetch_mutex = nullptr;
static uint8_t *g_prefetch_jpeg = nullptr;   // heap_caps blob, owned by cache
static size_t g_prefetch_jpeg_len = 0;
static uint32_t g_prefetch_ready_ms = 0;     // esp_log_timestamp() when produced
// A prefetched frame older than this is stale (stream stopped / manual capture
// gap): discard it and capture fresh so we never send a second-old image.
#define APP_PREFETCH_MAX_AGE_MS 800U

static void prefetch_cache_init()
{
    if (!g_prefetch_mutex) {
        g_prefetch_mutex = xSemaphoreCreateMutex();
    }
}

// Drop any cached prefetch blob. Caller must NOT hold g_prefetch_mutex.
static void prefetch_discard()
{
    if (!g_prefetch_mutex) return;
    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    if (g_prefetch_jpeg) {
        heap_caps_free(g_prefetch_jpeg);
        g_prefetch_jpeg = nullptr;
        g_prefetch_jpeg_len = 0;
        g_prefetch_ready_ms = 0;
    }
    xSemaphoreGive(g_prefetch_mutex);
}

// Capture one frame and JPEG-encode it into a heap_caps blob the caller owns.
// Returns nullptr on failure. Assumes the camera is powered.
static uint8_t *prepare_jpeg_blob(size_t *out_len)
{
    *out_len = 0;
    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0, height = 0, pixfmt = 0;
    const uint32_t prepare_start_ms = static_cast<uint32_t>(esp_log_timestamp());
    esp_err_t capture_e = g_camera_uart.capture_jpeg_input(
        &frame, &len, &width, &height, &pixfmt);
    const uint32_t capture_done_ms = static_cast<uint32_t>(esp_log_timestamp());
    if (capture_e != ESP_OK || !frame) {
        ESP_LOGW(TAG, "[PREFETCH] capture failed after %lums: %s",
                 static_cast<unsigned long>(capture_done_ms - prepare_start_ms),
                 esp_err_to_name(capture_e));
        return nullptr;
    }
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    esp_err_t e = g_radio.image_xfer().encode_prepared_frame(
        frame, len, width, height, pixfmt, &jpeg, &jpeg_len);
    const uint32_t encode_done_ms = static_cast<uint32_t>(esp_log_timestamp());
    heap_caps_free(frame);
    if (e != ESP_OK || !jpeg) {
        ESP_LOGW(TAG, "[PREFETCH] encode failed: capture=%lums encode=%lums error=%s",
                 static_cast<unsigned long>(capture_done_ms - prepare_start_ms),
                 static_cast<unsigned long>(encode_done_ms - capture_done_ms),
                 esp_err_to_name(e));
        return nullptr;
    }
    ESP_LOGD(TAG,
             "[PREFETCH] capture=%lums encode=%lums total=%lums raw=%u jpeg=%u",
             static_cast<unsigned long>(capture_done_ms - prepare_start_ms),
             static_cast<unsigned long>(encode_done_ms - capture_done_ms),
             static_cast<unsigned long>(encode_done_ms - prepare_start_ms),
             static_cast<unsigned>(len), static_cast<unsigned>(jpeg_len));
    *out_len = jpeg_len;
    return jpeg;
}

// Producer, called from the TX-wait loop: capture + encode the next frame into
// the cache. At most once per capture task; skipped if one is already cached.
static void prefetch_produce()
{
    if (!g_prefetch_mutex) return;
    // Already have one cached? Don't stack.
    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    bool have = (g_prefetch_jpeg != nullptr);
    xSemaphoreGive(g_prefetch_mutex);
    if (have) return;

    size_t jpeg_len = 0;
    uint8_t *jpeg = prepare_jpeg_blob(&jpeg_len);
    if (!jpeg) return;

    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    if (g_prefetch_jpeg) {
        // Raced with another producer (shouldn't happen — single capture task) —
        // keep the existing one, drop ours.
        heap_caps_free(jpeg);
    } else {
        g_prefetch_jpeg = jpeg;
        g_prefetch_jpeg_len = jpeg_len;
        g_prefetch_ready_ms = static_cast<uint32_t>(esp_log_timestamp());
    }
    xSemaphoreGive(g_prefetch_mutex);
}

// Consumer: try to take a fresh cached prefetch blob. Returns the blob (caller
// owns / frees) or nullptr if none / stale. On success *out_len is set.
static uint8_t *prefetch_take(size_t *out_len)
{
    *out_len = 0;
    if (!g_prefetch_mutex) return nullptr;
    xSemaphoreTake(g_prefetch_mutex, portMAX_DELAY);
    uint8_t *blob = nullptr;
    if (g_prefetch_jpeg) {
        uint32_t age = static_cast<uint32_t>(esp_log_timestamp()) - g_prefetch_ready_ms;
        if (age <= APP_PREFETCH_MAX_AGE_MS) {
            blob = g_prefetch_jpeg;
            *out_len = g_prefetch_jpeg_len;
        } else {
            heap_caps_free(g_prefetch_jpeg);  // stale, drop
        }
        g_prefetch_jpeg = nullptr;
        g_prefetch_jpeg_len = 0;
        g_prefetch_ready_ms = 0;
    }
    xSemaphoreGive(g_prefetch_mutex);
    return blob;
}

void finish_image_capture_task()
{
    // Clear the session before publishing idle, so a new request can never see
    // an idle capture slot still carrying the previous session identifier.
    g_active_capture_session.store(0, std::memory_order_release);
    g_capture_busy.store(false, std::memory_order_release);
    vTaskDelete(nullptr);
}

#if APP_INTERCOM_IMAGE_CAPTURE_PROBE_MS > 0
// Optional capture-and-encode diagnostic during an active call.
static void intercom_capture_probe_task(void *arg)
{
    (void)arg;
    uint32_t attempts = 0;
    uint32_t fails = 0;
    for (;;) {
        // Only while a call is up; otherwise poll cheaply and wait.
        if (!g_radio.intercom_active()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        // Respect the shared capture guard so we never race any other capture.
        bool expected_idle = false;
        if (!g_capture_busy.compare_exchange_strong(expected_idle, true,
                                                    std::memory_order_acq_rel)) {
            vTaskDelay(pdMS_TO_TICKS(APP_INTERCOM_IMAGE_CAPTURE_PROBE_MS));
            continue;
        }

        const uint32_t t0 = static_cast<uint32_t>(esp_log_timestamp());
        size_t jpeg_len = 0;
        uint8_t *jpeg = prepare_jpeg_blob(&jpeg_len);  // capture + encode, no suspend
        const uint32_t dt = static_cast<uint32_t>(esp_log_timestamp()) - t0;
        attempts++;
        if (jpeg) {
            heap_caps_free(jpeg);  // probe only: measure, then discard
        } else {
            fails++;
        }
        g_capture_busy.store(false, std::memory_order_release);

        ESP_LOGI(TAG,
                 "[IMG PROBE] capture+encode=%lums jpeg=%u bytes ok=%d "
                 "attempts=%lu fails=%lu (no suspend, not sent)",
                 static_cast<unsigned long>(dt),
                 static_cast<unsigned>(jpeg_len), jpeg ? 1 : 0,
                 static_cast<unsigned long>(attempts),
                 static_cast<unsigned long>(fails));

        vTaskDelay(pdMS_TO_TICKS(APP_INTERCOM_IMAGE_CAPTURE_PROBE_MS));
    }
}
#endif

#if APP_INTERCOM_IMAGE_REAL_CAPTURE_MS > 0
// Publish periodic JPEG frames to the in-call slot-tail transport.
static void intercom_capture_feed_task(void *arg)
{
    (void)arg;
    uint32_t attempts = 0;
    uint32_t fails = 0;
    const TickType_t frame_period_ticks =
        pdMS_TO_TICKS(kIntercomImageFramePeriodMs);
    TickType_t last_frame_tick = xTaskGetTickCount();
    for (;;) {
        if (!g_radio.intercom_active()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            // Do not catch up frames accumulated while the call was inactive.
            last_frame_tick = xTaskGetTickCount();
            continue;
        }
        bool expected_idle = false;
        if (!g_capture_busy.compare_exchange_strong(expected_idle, true,
                                                    std::memory_order_acq_rel)) {
            vTaskDelayUntil(&last_frame_tick, frame_period_ticks);
            continue;
        }

        const uint32_t t0 = static_cast<uint32_t>(esp_log_timestamp());
        size_t jpeg_len = 0;
        uint8_t *jpeg = prepare_jpeg_blob(&jpeg_len);  // capture + encode, no suspend
        const uint32_t dt = static_cast<uint32_t>(esp_log_timestamp()) - t0;
        g_capture_busy.store(false, std::memory_order_release);
        attempts++;

        if (jpeg && jpeg_len > 0) {
            // Ownership transfers to the radio (freed on adopt or replace).
            g_radio.intercom_image_publish(jpeg, jpeg_len);
        } else {
            if (jpeg) heap_caps_free(jpeg);
            fails++;
        }

        // Per-frame timing: verbose at the feed rate, so keep it at DEBUG and
        // read the aggregate from the periodic [CPU] / [IMG RX] lines instead.
        ESP_LOGD(TAG,
                 "[IMG FEED] capture+encode=%lums jpeg=%u bytes ok=%d "
                 "attempts=%lu fails=%lu",
                 static_cast<unsigned long>(dt),
                 static_cast<unsigned>(jpeg_len), (jpeg && jpeg_len > 0) ? 1 : 0,
                 static_cast<unsigned long>(attempts),
                 static_cast<unsigned long>(fails));

        // Keep capture starts on an absolute cadence. Capture/encode time is part
        // of the frame period instead of being added as a post-work delay.
        vTaskDelayUntil(&last_frame_tick, frame_period_ticks);
    }
}
#endif

void image_capture_task(void *arg)
{
    auto *ctx = static_cast<ImageCaptureCtx *>(arg);
    uint16_t session_id = ctx->session_id;
    delete ctx;

    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;

    uint32_t t_cmd = static_cast<uint32_t>(esp_log_timestamp());
    // Per-frame capture step timings are DEBUG (they fire mid-capture, on the hot
    // path); only the post-send "[TIMING] total" summary stays at INFO.
    ESP_LOGD(TAG, "[TIMING] cmd received t=0ms");

    bsp_lcd_set_camera_status("Remote capture...");

    esp_err_t e = ESP_OK;
    (void)e;
#if APP_CAMERA_NODE_LCD_ENABLE
    e = bsp_lcd_release_for_camera();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "release lcd for image capture: %s", esp_err_to_name(e));
        bsp_lcd_reinit_after_camera();
        bsp_lcd_set_camera_status("LCD release failed");
        finish_image_capture_task();
        return;
    }
#endif

    // JPEG for this frame. Use a fresh prefetched frame when available;
    // otherwise capture and encode on demand.
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    uint32_t t_jpeg_done;
    jpeg = prefetch_take(&jpeg_len);
    if (jpeg) {
        t_jpeg_done = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] prefetch hit +%lums (jpeg=%u bytes)",
                 static_cast<unsigned long>(t_jpeg_done - t_cmd),
                 static_cast<unsigned>(jpeg_len));
    } else {
        uint32_t t_cam_start = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] camera init start +%lums", static_cast<unsigned long>(t_cam_start - t_cmd));

        esp_err_t capture_e = g_camera_uart.capture_jpeg_input(
            &frame, &len, &width, &height, &pixfmt);

        uint32_t t_cam_done = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] capture done +%lums (camera=%lums) %lux%lu %u bytes",
                 static_cast<unsigned long>(t_cam_done - t_cmd),
                 static_cast<unsigned long>(t_cam_done - t_cam_start),
                 static_cast<unsigned long>(width),
                 static_cast<unsigned long>(height),
                 static_cast<unsigned>(len));

        // LCD reinit is deferred until after transmission.
        if (capture_e != ESP_OK || !frame) {
#if APP_CAMERA_NODE_LCD_ENABLE
            bsp_lcd_reinit_after_camera();
#endif
            bsp_lcd_set_camera_status("Capture failed");
            finish_image_capture_task();
            return;
        }

        // JPEG encode
        uint32_t t_jpeg_start = static_cast<uint32_t>(esp_log_timestamp());
        e = g_radio.image_xfer().encode_prepared_frame(
            frame, len, width, height, pixfmt, &jpeg, &jpeg_len);
        heap_caps_free(frame);
        t_jpeg_done = static_cast<uint32_t>(esp_log_timestamp());
        ESP_LOGD(TAG, "[TIMING] JPEG done +%lums (jpeg=%lums) %u bytes",
                 static_cast<unsigned long>(t_jpeg_done - t_cmd),
                 static_cast<unsigned long>(t_jpeg_done - t_jpeg_start),
                 static_cast<unsigned>(jpeg_len));

        if (e != ESP_OK || !jpeg) {
#if APP_CAMERA_NODE_LCD_ENABLE
            bsp_lcd_reinit_after_camera();
#endif
            bsp_lcd_set_camera_status("JPEG encode failed");
            finish_image_capture_task();
            return;
        }
    }

    uint16_t total_frags = static_cast<uint16_t>(
        (jpeg_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
    ESP_LOGD(TAG, "sending JPEG: %u pkts, %u bytes",
             total_frags, static_cast<unsigned>(jpeg_len));

    g_radio.send_image(jpeg, jpeg_len, session_id);

    // Capture the next frame while the current frame is sent on the independent
    // radio SPI bus. Unused prefetch data expires in prefetch_take().
    prefetch_produce();

    // image_tx_task owns and frees jpeg after send_image(). This wait only
    // sequences capture cleanup after transmission.
    bool tx_done = false;
    uint32_t wait_start = xTaskGetTickCount();
    while (xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(30000)) {
        if (!g_radio.image_tx_busy()) {
            tx_done = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint32_t t_tx_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] total +%lums | img_prep=%lu tx=%lums",
             static_cast<unsigned long>(t_tx_done - t_cmd),
             static_cast<unsigned long>(t_jpeg_done - t_cmd),
             static_cast<unsigned long>(t_tx_done - t_jpeg_done));

    if (!tx_done) {
        // The TX task still owns the JPEG and will finish radio cleanup.
        // Skip post-TX work that could contend with the active transfer.
        ESP_LOGW(TAG, "image tx still active after 30s; skipping post-tx cleanup (tx task finishes on its own)");
        finish_image_capture_task();
        return;
    }

    // Reinit LCD now that transmission is done
#if APP_CAMERA_NODE_LCD_ENABLE
    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after tx: %s", esp_err_to_name(lcd_e));
    }
#endif
    char status[48];
    snprintf(status, sizeof(status), "Done %u pkts", total_frags);
    bsp_lcd_set_camera_status(status);

    finish_image_capture_task();
}

// Start a door-station capture. Duplicate active sessions are ignored; completed
// session IDs may be reused after the gateway restarts.
bool on_image_capture_request(uint16_t session_id)
{
    if (g_radio.intercom_active()) {
        ESP_LOGD(TAG, "capture ignored while intercom is active");
        g_radio.notify_capture_dropped();
        return false;
    }
    if (g_app_mode != AppMode::camera) {
        ESP_LOGW(TAG, "ImageCmd received outside the door-station role");
        g_radio.notify_capture_dropped();
        return false;
    }

    uint16_t active_session = g_active_capture_session.load(std::memory_order_acquire);
    if (g_capture_busy.load(std::memory_order_acquire)) {
        if (active_session != 0 && session_id == active_session) {
            // The gateway did not hear the earlier ACK. Re-ACK it without
            // spawning another capture or clearing the active wake guard.
            return true;
        }
        ESP_LOGD(TAG, "capture busy: queued session=%u active=%u",
                 session_id, active_session);
        return false;
    }

    bool expected_idle = false;
    if (!g_capture_busy.compare_exchange_strong(expected_idle, true,
                                                std::memory_order_acq_rel)) {
        active_session = g_active_capture_session.load(std::memory_order_acquire);
        return active_session != 0 && session_id == active_session;
    }
    g_active_capture_session.store(session_id, std::memory_order_release);

    // Low power: hold the node awake through the capture + push, or CAD light
    // sleep starves the camera/JPEG task.
    g_radio.notify_capture_starting();

    auto *ctx = new (std::nothrow) ImageCaptureCtx{ session_id };
    if (!ctx) {
        g_active_capture_session.store(0, std::memory_order_release);
        g_capture_busy.store(false, std::memory_order_release);
        g_radio.notify_capture_dropped();
        return false;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(image_capture_task, "img_cap",
                                            APP_IMAGE_TASK_STACK_BYTES, ctx,
                                            APP_IMAGE_TASK_PRIORITY, nullptr,
                                            APP_IMAGE_TASK_CORE);
    if (ok != pdPASS) {
        delete ctx;
        g_active_capture_session.store(0, std::memory_order_release);
        g_capture_busy.store(false, std::memory_order_release);
        g_radio.notify_capture_dropped();
        ESP_LOGE(TAG, "image capture task create failed");
        return false;
    }
    return true;
}

void gateway_image_frame_free(GatewayImageFrame *frame)
{
    if (!frame || !frame->jpeg) return;
    heap_caps_free(frame->jpeg);
    frame->jpeg = nullptr;
}

bool gateway_image_queue_push(const GatewayImageFrame &frame)
{
    if (!g_gateway_image_queue) return false;
    if (xQueueSend(g_gateway_image_queue, &frame, 0) == pdTRUE) return true;

    GatewayImageFrame stale = {};
    if (xQueueReceive(g_gateway_image_queue, &stale, 0) == pdTRUE) {
        ESP_LOGW(TAG, "image queue full: drop stale session=%u", stale.session_id);
        gateway_image_frame_free(&stale);
    }
    return xQueueSend(g_gateway_image_queue, &frame, 0) == pdTRUE;
}

void gateway_image_queue_discard_pending()
{
    if (!g_gateway_image_queue) return;
    GatewayImageFrame frame = {};
    while (xQueueReceive(g_gateway_image_queue, &frame, 0) == pdTRUE) {
        gateway_image_frame_free(&frame);
    }
}

void gateway_image_task(void *arg)
{
    (void)arg;
    uint8_t *rgb565 = nullptr;
    int rgb565_capacity = 0;
    uint32_t last_frame_ms = 0;

    while (true) {
        GatewayImageFrame frame = {};
        if (xQueueReceive(g_gateway_image_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // In-call frames remain visible during a call. Playback frames require
        // the playback page, while PIR snapshots are decoded only for storage.
        const bool live_gate_ok =
            frame.from_intercom ||
            (ui_gw_stream_active() && !g_radio.intercom_active());
        const bool playback_gate_ok = frame.from_playback && ui_gw_playback_active();
        const bool wanted = frame.from_snapshot ||
                            (frame.from_playback ? playback_gate_ok : live_gate_ok);
        if (g_app_mode != AppMode::radio || !wanted) {
            gateway_image_frame_free(&frame);
            continue;
        }

        const uint32_t task_start_ms = static_cast<uint32_t>(esp_log_timestamp());
        const uint32_t queue_wait_ms = task_start_ms - frame.queued_ms;
        uint32_t decode_ms = 0;
        uint32_t compose_ms = 0;
        bool submitted = false;

        jpeg_dec_config_t dec_cfg = DEFAULT_JPEG_DEC_CONFIG();
        dec_cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;

        jpeg_dec_handle_t decoder = nullptr;
        jpeg_error_t jerr = jpeg_dec_open(&dec_cfg, &decoder);
        if (jerr == JPEG_ERR_OK && decoder) {
            jpeg_dec_io_t io = {};
            io.inbuf = frame.jpeg;
            io.inbuf_len = static_cast<int>(frame.jpeg_len);

            jpeg_dec_header_info_t header = {};
            jerr = jpeg_dec_parse_header(decoder, &io, &header);
            int outbuf_len = 0;
            if (jerr == JPEG_ERR_OK) {
                jerr = jpeg_dec_get_outbuf_len(decoder, &outbuf_len);
            }
            if (jerr == JPEG_ERR_OK && outbuf_len > 0) {
                if (outbuf_len > rgb565_capacity) {
                    if (rgb565) jpeg_free_align(rgb565);
                    /* Own complete PSRAM cache lines so another DMA buffer's
                     * invalidation cannot discard this image's edge bytes. */
                    rgb565 = static_cast<uint8_t *>(jpeg_calloc_align(
                        outbuf_len, CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE));
                    rgb565_capacity = rgb565 ? outbuf_len : 0;
                }
                if (rgb565) {
                    io.outbuf = rgb565;
                    jerr = jpeg_dec_process(decoder, &io);
                    decode_ms = static_cast<uint32_t>(esp_log_timestamp()) - task_start_ms;
                    // The first live frame decoded during a visitor record
                    // becomes its thumbnail; the store ignores the rest.
                    if (jerr == JPEG_ERR_OK && !frame.from_playback &&
                        !frame.from_snapshot && visitor_store_recording()) {
                        visitor_store_offer_thumbnail(
                            reinterpret_cast<const uint16_t *>(rgb565),
                            header.width, header.height);
                    }
                    if (jerr == JPEG_ERR_OK && frame.from_snapshot) {
                        // Filed, not shown; the store copies the JPEG.
                        if (visitor_store_snapshot_add(
                                frame.jpeg, frame.jpeg_len,
                                reinterpret_cast<const uint16_t *>(rgb565),
                                header.width, header.height)) {
                            ui_gw_note_unseen();
                            ui_gw_records_changed();
                        }
                    } else if (jerr == JPEG_ERR_OK &&
                        (frame.from_intercom || frame.from_playback ||
                         ui_gw_stream_active())) {
                        const uint32_t compose_start_ms =
                            static_cast<uint32_t>(esp_log_timestamp());
                        ui_gw_rx_complete(reinterpret_cast<const uint16_t *>(rgb565),
                                          header.width, header.height,
                                          static_cast<uint32_t>(frame.jpeg_len),
                                          frame.transfer_ms);
                        compose_ms = static_cast<uint32_t>(esp_log_timestamp()) -
                                     compose_start_ms;
                        submitted = true;
                    }
                } else {
                    ESP_LOGE(TAG, "gateway RGB565 alloc failed: %d bytes", outbuf_len);
                }
            }
            jpeg_dec_close(decoder);
        }

        if (jerr != JPEG_ERR_OK) {
            ESP_LOGE(TAG, "gateway JPEG decode failed: %d session=%u", jerr,
                     frame.session_id);
        }
        gateway_image_frame_free(&frame);

        if (submitted) {
            const uint32_t now_ms = static_cast<uint32_t>(esp_log_timestamp());
            const uint32_t period_ms = last_frame_ms ? now_ms - last_frame_ms : 0;
            last_frame_ms = now_ms;
            const uint32_t consume_ms = frame.reassemble_ms + decode_ms + compose_ms;
            if (period_ms > 0) {
                ESP_LOGI(TAG,
                         "[FRAME] period=%lums (%lu.%lu fps) | transfer=%lums "
                         "consume=%lums (queue=%lu reassemble=%lu decode=%lu compose=%lu) "
                         "jpeg=%u frags=%u",
                         (unsigned long)period_ms,
                         (unsigned long)(1000U / period_ms),
                         (unsigned long)((10000U / period_ms) % 10U),
                         (unsigned long)frame.transfer_ms,
                         (unsigned long)consume_ms,
                         (unsigned long)queue_wait_ms,
                         (unsigned long)frame.reassemble_ms,
                         (unsigned long)decode_ms,
                         (unsigned long)compose_ms,
                         static_cast<unsigned>(frame.jpeg_len), frame.fragments);
            }
        }
    }
}

esp_err_t gateway_image_pipeline_init()
{
    if (g_gateway_image_queue && g_gateway_image_task_handle) return ESP_OK;

    g_gateway_image_queue = xQueueCreate(kGatewayImageQueueLength,
                                         sizeof(GatewayImageFrame));
    if (!g_gateway_image_queue) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreatePinnedToCore(
        gateway_image_task, "gw_image", kGatewayImageTaskStackBytes, nullptr,
        kGatewayImageTaskPriority, &g_gateway_image_task_handle,
        kGatewayImageTaskCore);
    if (ok != pdPASS) {
        vQueueDelete(g_gateway_image_queue);
        g_gateway_image_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "gateway image pipeline: queue=%u core=%d priority=%u",
             static_cast<unsigned>(kGatewayImageQueueLength),
             static_cast<int>(kGatewayImageTaskCore),
             static_cast<unsigned>(kGatewayImageTaskPriority));
    return ESP_OK;
}

void on_image_rx_complete(ImageTransfer *xfer)
{
    if (!xfer || !xfer->rx_complete()) return;

    const uint32_t reassemble_start_ms = static_cast<uint32_t>(esp_log_timestamp());
    GatewayImageFrame frame = {};
    frame.transfer_ms = g_radio.last_transfer_ms();
    frame.session_id = xfer->rx_session_id();
    frame.fragments = xfer->rx_total_count();
    frame.from_snapshot = g_radio.image_rx_unsolicited();

    esp_err_t e = xfer->rx_reassemble(&frame.jpeg, &frame.jpeg_len);
    if (e != ESP_OK || !frame.jpeg) {
        ESP_LOGE(TAG, "rx_reassemble failed: %d", e);
        xfer->rx_reset();
        return;
    }

    // A/V stream: the payload is [JPEG][audio]. Hand the tail to the speaker
    // and shorten the JPEG length; the buffer stays over-allocated.
    const uint16_t av_audio_len = g_radio.image_rx_audio_len();
    if (av_audio_len > 0 && av_audio_len < frame.jpeg_len) {
        frame.jpeg_len -= av_audio_len;
        if (!frame.from_snapshot) {
            g_radio.deliver_image_audio(frame.jpeg + frame.jpeg_len, av_audio_len);
        }
    } else if (av_audio_len > 0) {
        ESP_LOGW(TAG, "A/V audio_len=%u exceeds payload %u: dropping malformed frame",
                 av_audio_len, static_cast<unsigned>(frame.jpeg_len));
        gateway_image_frame_free(&frame);
        xfer->rx_reset();
        return;
    }

    xfer->rx_reset();
    // Visitor record in progress: keep JPEG and audio as they came, before
    // the decoder or speaker get their copies.
    if (!frame.from_snapshot && visitor_store_recording()) {
        visitor_store_push(frame.jpeg, frame.jpeg_len,
                           av_audio_len > 0 ? frame.jpeg + frame.jpeg_len : nullptr,
                           av_audio_len, g_radio.image_rx_audio_seq());
    }
    // A delivered picture stamps the status bar's age field.
    if (g_app_mode == AppMode::radio) {
        ui_gw_note_event();
    }
    frame.reassemble_ms = static_cast<uint32_t>(esp_log_timestamp()) -
                          reassemble_start_ms;
    frame.queued_ms = static_cast<uint32_t>(esp_log_timestamp());

    if (!gateway_image_queue_push(frame)) {
        ESP_LOGE(TAG, "gateway image queue unavailable: drop session=%u",
                 frame.session_id);
        gateway_image_frame_free(&frame);
    }
}

#if APP_INTERCOM_IMAGE_REAL_CAPTURE_MS > 0
// Takes ownership of each reassembled in-call JPEG and forwards it to the
// decode/display queue. Early exits release the buffer here.
void on_intercom_image_frame(uint8_t *jpeg, size_t len, uint16_t session,
                              uint16_t fragments, uint32_t transfer_ms,
                              uint32_t reassemble_ms, void *ctx)
{
    (void)ctx;
    if (!jpeg || len == 0) {
        if (jpeg) heap_caps_free(jpeg);
        return;
    }
    GatewayImageFrame frame = {};
    frame.jpeg = jpeg;
    frame.jpeg_len = len;
    frame.session_id = session;
    frame.from_intercom = true;
    frame.transfer_ms = transfer_ms;
    frame.reassemble_ms = reassemble_ms;
    frame.fragments = fragments;
    frame.queued_ms = static_cast<uint32_t>(esp_log_timestamp());
    if (!gateway_image_queue_push(frame)) {
        ESP_LOGW(TAG, "intercom image queue unavailable: drop session=%u", session);
        gateway_image_frame_free(&frame);
    }
}
#endif

void camera_capture_task(void *arg)
{
    (void)arg;
    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;

#if APP_AUDIO_FEATURES_ENABLE
#if APP_RADIO_FEATURES_ENABLE
    if (g_radio_active) {
        g_radio.suspend();
    }
#endif
    esp_err_t audio_e = bsp_audio_suspend();
    if (audio_e != ESP_OK) {
        ESP_LOGW(TAG, "audio suspend before camera: %s", esp_err_to_name(audio_e));
    }
#endif
    bsp_lcd_set_camera_status("Preparing camera...");
    esp_err_t e = bsp_lcd_release_for_camera();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "release lcd for camera: %s", esp_err_to_name(e));
        bsp_lcd_reinit_after_camera();
        bsp_lcd_set_camera_status("LCD release failed");
#if APP_AUDIO_FEATURES_ENABLE
        if ((e = bsp_audio_resume()) != ESP_OK) {
            ESP_LOGW(TAG, "audio resume after LCD release failure: %s", esp_err_to_name(e));
        }
#if APP_RADIO_FEATURES_ENABLE
        if (g_radio_active) {
            g_radio.resume();
        }
#endif
#endif
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);
    ESP_LOGI(TAG, "capture result=%s frame=%p len=%u %lux%lu fourcc=0x%08lx",
             esp_err_to_name(capture_e), frame, static_cast<unsigned>(len),
             static_cast<unsigned long>(width),
             static_cast<unsigned long>(height),
             static_cast<unsigned long>(pixfmt));

    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after camera: %s", esp_err_to_name(lcd_e));
    }
#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_audio_resume()) != ESP_OK) {
        ESP_LOGW(TAG, "audio resume after camera: %s", esp_err_to_name(e));
    }
#if APP_RADIO_FEATURES_ENABLE
    if (g_radio_active) {
        g_radio.resume();
    }
#endif
#endif

    if (capture_e == ESP_OK && (pixfmt == 0x56595559 || pixfmt == 0x59565955 ||
                                pixfmt == 0x55595659 || pixfmt == 0x59555956)) {
        if (bsp_lcd_show_yuv422_photo(frame, width, height, pixfmt) == ESP_OK) {
            bsp_lcd_set_camera_status("Captured. Touch capture to retake");
        } else {
            bsp_lcd_set_camera_status("Display photo failed");
        }
    } else if (capture_e == ESP_OK && pixfmt == 0x59455247) { // 'GREY'
        if (bsp_lcd_show_gray_photo(frame, width, height) == ESP_OK) {
            bsp_lcd_set_camera_status("Captured. Touch capture to retake");
        } else {
            bsp_lcd_set_camera_status("Display photo failed");
        }
    } else if (capture_e == ESP_OK) {
        char status[64];
        snprintf(status, sizeof(status), "Unsupported pixel 0x%08lx",
                 static_cast<unsigned long>(pixfmt));
        bsp_lcd_set_camera_status(status);
    } else {
        bsp_lcd_clear_camera_photo();
        char status[64];
        const char *short_name = short_error_name(capture_e);
        if (short_name) {
            snprintf(status, sizeof(status), "Fail:%s", short_name);
        } else {
            snprintf(status, sizeof(status), "Fail:0x%lx",
                     static_cast<unsigned long>(capture_e));
        }
        bsp_lcd_set_camera_status(status);
    }

    heap_caps_free(frame);
    g_capture_busy = false;
    vTaskDelete(nullptr);
}

void on_lcd_capture(void *user)
{
    (void)user;
    if (g_app_mode == AppMode::radio) {
        bsp_lcd_set_camera_status("Gateway role. Hold K1 to switch role");
        return;
    }
    if (g_capture_busy) {
        bsp_lcd_set_camera_status("Capture already running");
        return;
    }
    g_capture_busy = true;
    BaseType_t ok = xTaskCreatePinnedToCore(camera_capture_task,
                                            "touch_capture",
                                            APP_CAMERA_TASK_STACK_BYTES,
                                            nullptr,
                                            APP_CAMERA_TASK_PRIORITY + 3,
                                            nullptr,
                                            APP_CAMERA_TASK_CORE);
    if (ok != pdPASS) {
        g_capture_busy = false;
        bsp_lcd_set_camera_status("Capture task start failed");
    }
}

// Gateway image-reception progress callback.
void on_image_rx_progress(uint16_t received, uint16_t total, int16_t rssi)
{
    if (g_app_mode == AppMode::radio) {
        if (received == 0) {
            ui_gw_rx_begin(0, total);
            return;
        }
        ui_gw_rx_progress(received, total, rssi);
    }
}

// Forward the door-station battery voltage to the gateway UI.
void on_vbat_received(uint16_t vbat_mv)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_update_vbat(vbat_mv);
    }
}

// Forward completed-frame link statistics to the gateway UI.
void on_link_stats(uint32_t bytes_per_s, int16_t rssi, bool rssi_valid)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_update_link_stats(bytes_per_s, rssi, rssi_valid);
    }
}

void on_image_rx_eot_nack(uint16_t missing_count, bool is_first_eot)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_rx_eot_nack(missing_count, is_first_eot);
    }
}

// Gateway UI capture callback — triggers remote photo via radio

bool on_gw_capture(void)
{
    if (g_radio.intercom_active()) return false;
    ESP_LOGI(TAG, "UI capture: trigger remote photo");
    g_radio.trigger_image_capture();
    return true;
}

bool on_gw_intercom_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI intercom: %s", enable ? "on" : "off");
    if (enable) {
        gateway_image_queue_discard_pending();
        // Let the stream run out at a frame boundary so the node is idle in
        // RX when the CONFIG goes out.
        g_radio.quiesce_image_stream(APP_STREAM_QUIESCE_MS);
    }
    const uint32_t start_ms = static_cast<uint32_t>(esp_log_timestamp());
    const bool ok = g_radio.set_intercom(enable != 0);
    ESP_LOGI(TAG, "UI intercom result: requested=%s ok=%d elapsed=%lums active=%d",
             enable ? "on" : "off", ok ? 1 : 0,
             static_cast<unsigned long>(static_cast<uint32_t>(esp_log_timestamp()) -
                                        start_ms),
             g_radio.intercom_active() ? 1 : 0);
    return ok;
}

// Gateway UI: user left the transfer page. Abort the RX (the node's TX
// self-aborts once ACKs stop) and drop any partial store-side transfer.
void on_gw_rx_abort(void)
{
    ESP_LOGI(TAG, "UI: left transfer page, aborting image RX");
    gateway_image_queue_discard_pending();
    g_radio.abort_image_rx();
}

// Gateway UI: the PIR switch. What the node does on a detection is fixed - one
// capture, pushed here as a snapshot (see APP_VISITOR_SNAPSHOTS).
bool on_gw_pir_trigger_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI PIR trigger: %s", enable ? "on" : "off");
    return g_radio.send_config(APP_CFG_KEY_PIR_TRIGGER, enable);
}

bool on_gw_low_power_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI low power: %s", enable ? "on" : "off");
    // Send first, commit local state only on success.
    bool ok = g_radio.send_config(APP_CFG_KEY_LOW_POWER, enable);
    if (ok) {
        g_low_power_enabled = (enable != 0);
    }
    return ok;
}

// Gateway UI: JPEG quality for the door station's encoder. Nothing to keep
// here; the node stores it and the UI keeps its own copy for the badge.
bool on_gw_jpeg_quality_change(uint32_t quality)
{
    ESP_LOGI(TAG, "UI JPEG quality: %lu", static_cast<unsigned long>(quality));
    return g_radio.send_config(APP_CFG_KEY_JPEG_QUALITY, quality);
}

void on_button(bsp_btn_id_t id, bool pressed, void *user)
{
    (void)user;

    // Route gateway keys to the UI.
    if (g_app_mode == AppMode::radio) {
        // K1 long press (>1.5 s) selects the door-station role.
        if (id == BSP_BTN_PTT) {
            if (pressed) {
                g_ptt_press_time_us = esp_timer_get_time();
                g_ptt_held_long = false;
                if (g_ptt_timer) {
                    esp_timer_start_once(g_ptt_timer, 1500000); // 1.5s for mode switch
                }
            } else {
                if (g_ptt_timer) {
                    esp_timer_stop(g_ptt_timer);
                }
                if (g_ptt_held_long) {
                    switch_mode_and_restart();
                } else {
                    ui_gw_key_event(id, true);
                }
                g_ptt_held_long = false;
            }
            return;
        }
        ui_gw_key_event(id, pressed);
        return;
    }

    // Door station: K2 switches role and K5 rings; K1, K4, and BOOT are unused.
    if (id == BSP_BTN_USER1) {
        if (pressed) {
            switch_mode_and_restart();
        }
        return;
    }
#if APP_DOORBELL_ENABLE
    // Awake and low-power wake paths share doorbell_trigger() de-duplication.
    if (id == APP_DOORBELL_KEY) {
        if (pressed) {
            g_radio.doorbell_trigger();
#if APP_DOORBELL_ALSO_CAPTURE
            if (!g_capture_busy) {
                on_image_capture_request(g_auto_session_id++);
            }
#endif
        }
        return;
    }
#endif
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Lierda L-LRMAM36-FANN4-DK01 booting");
    prefetch_cache_init();
    esp_log_level_set("RALF_LR20XX", ESP_LOG_WARN);

    esp_err_t e;
    init_nvs();
    g_app_mode = load_app_mode();
    ESP_LOGI(TAG, "app mode: %s", mode_name(g_app_mode));

    ESP_ERROR_CHECK(bsp_i2c_init());

    printf("PSRAM free: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    printf("PSRAM total: %d\n", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    log_heap_state("startup");

#if APP_CAMERA_LCD_BRINGUP
    bsp_i2c_scan();
    if ((e = g_camera_uart.init()) != ESP_OK) {
        ESP_LOGE(TAG, "camera init: %s", esp_err_to_name(e));
        return;
    }
    if ((e = bsp_lcd_init()) != ESP_OK) {
        ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
        return;
    }
    if ((e = bsp_lcd_show_test_pattern()) != ESP_OK) {
        ESP_LOGE(TAG, "lcd test pattern: %s", esp_err_to_name(e));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(800));
    if ((e = bsp_lcd_start_camera_ui(on_lcd_capture, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "camera ui start: %s", esp_err_to_name(e));
        return;
    }
    ESP_LOGI(TAG, "V02 camera/LCD validation UI ready: ST7789V3 %ux%u, SP0A39 DVP %ux%u",
             APP_LCD_H_RES, APP_LCD_V_RES,
             APP_CAMERA_SENSOR_WIDTH, APP_CAMERA_SENSOR_HEIGHT);
    return;
#endif

#if APP_CAMERA_ONLY_BRINGUP
    ESP_LOGW(TAG, "camera diagnostic mode: skipping CON6 detect, LED, audio, LR2021 radio, buttons, LCD, chime");
#if APP_CAMERA_UART_ENABLE
    if ((e = g_camera_uart.start()) != ESP_OK) {
        ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "camera diagnostic SP0A39 one-frame capture: MCLK GPIO%d, PWDN IOEXP P%d",
             BSP_SP0A39_MCLK_GPIO, BSP_SP0A39_PWDN_IOEXP_PIN);
#else
    ESP_LOGW(TAG, "APP_CAMERA_UART_ENABLE is disabled");
#endif
    return;
#endif

    bsp_i2c_scan();

#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_led_init()) != ESP_OK) {
        ESP_LOGE(TAG, "led init: %s", esp_err_to_name(e));
    }
    if (g_app_mode == AppMode::radio) {
        if ((e = bsp_audio_init_gateway_duplex(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
            ESP_LOGE(TAG, "audio init (duplex): %s", esp_err_to_name(e));
        }
    } else {
        if ((e = bsp_audio_init(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
            ESP_LOGE(TAG, "audio init: %s", esp_err_to_name(e));
        }
    }
    if ((e = g_audio.init()) != ESP_OK) {
        ESP_LOGE(TAG, "audio diagnostics init: %s", esp_err_to_name(e));
    }
    log_heap_state("after audio");
#if APP_RADIO_FEATURES_ENABLE
    {
        // Driver 0.0.7 loads LR2021 PRAM during ral_init(). Configure NRST as a
        // driven-high output before the first ral_reset(), otherwise the initial
        // reset pulse can be lost and leave BUSY stuck high on a cold start.
        gpio_config_t nrst_conf = {};
        nrst_conf.pin_bit_mask = 1ULL << CONFIG_LR2021_NRST_GPIO;
        nrst_conf.mode         = GPIO_MODE_OUTPUT;
        nrst_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
        nrst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        nrst_conf.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&nrst_conf);
        gpio_set_level((gpio_num_t)CONFIG_LR2021_NRST_GPIO, 1);

        bool radio_ok = true;
        if (g_app_mode == AppMode::radio) {
            if ((e = g_radio.init_gateway()) != ESP_OK) {
                ESP_LOGE(TAG, "radio init (gateway): %s", esp_err_to_name(e));
                radio_ok = false;
            }
        } else {
            if ((e = g_radio.init()) != ESP_OK) {
                ESP_LOGE(TAG, "radio init: %s", esp_err_to_name(e));
                radio_ok = false;
            }
        }
        if (radio_ok) {
            g_radio.set_image_capture_cb(on_image_capture_request);
            g_radio.set_image_rx_complete_cb(on_image_rx_complete);
            g_radio.set_image_rx_progress_cb(on_image_rx_progress);
            g_radio.set_vbat_received_cb(on_vbat_received);
            g_radio.set_image_rx_eot_cb(on_image_rx_eot_nack);
            g_radio.set_config_received_cb(on_config_received);
            g_radio.set_low_power_standby_cb(on_low_power_standby);
            g_radio.set_intercom_state_cb(on_intercom_state);
            g_radio.set_doorbell_cb(on_doorbell_ring);
            g_radio.set_doorbell_stream_cbs(on_doorbell_stream_start,
                                            on_doorbell_stream_end);
            g_radio.set_link_stats_cb(on_link_stats);
#if APP_INTERCOM_IMAGE_REAL_CAPTURE_MS > 0
            // Gateway: route in-call reassembled JPEG frames to decode+display.
            // Harmless on the node (it never fires this callback).
            g_radio.set_intercom_image_frame_cb(on_intercom_image_frame, nullptr);
#endif
        }
        if (g_app_mode == AppMode::radio && radio_ok) {
#if APP_RADIO_TASKS_ENABLE
            if ((e = g_radio.start_gateway()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start (gateway): %s", esp_err_to_name(e));
            } else {
                g_radio_active = true;
                nvs_handle_t gw_nvs;
                if (nvs_open("ui_gw", NVS_READONLY, &gw_nvs) == ESP_OK) {
                    uint8_t lp = 0;
                    nvs_get_u8(gw_nvs, "lowpwr", &lp);
                    g_low_power_enabled = (lp != 0);
                    nvs_close(gw_nvs);
                }

            }
#else
            ESP_LOGW(TAG, "radio tasks/RX disabled for camera diagnostic isolation");
#endif
        } else if (g_app_mode == AppMode::camera && radio_ok) {
            if ((e = g_radio.start()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start (door-station role): %s", esp_err_to_name(e));
            }
            ESP_LOGI(TAG, "door-station role: radio initialized for image transfer");
            g_radio.set_pir_enabled(load_config_u8("pir", 0) != 0);
            g_low_power_enabled = load_config_u8("lowpwr", 0) != 0;
            g_radio.image_xfer().set_jpeg_quality(
                load_config_u8("jpegq", APP_IMAGE_JPEG_QUALITY));
            ESP_LOGI(TAG, "NVS: pir=%d lowpwr=%d jpegq=%u",
                     load_config_u8("pir", 0), g_low_power_enabled,
                     g_radio.image_xfer().jpeg_quality());

#if APP_INTERCOM_IMAGE_CAPTURE_PROBE_MS > 0
            // Optional capture-and-encode diagnostic below voice priority.
            if (xTaskCreatePinnedToCore(intercom_capture_probe_task, "img_probe",
                                        APP_IMAGE_TASK_STACK_BYTES, nullptr,
                                        APP_IMAGE_TASK_PRIORITY, nullptr,
                                        APP_IMAGE_TASK_CORE) != pdPASS) {
                ESP_LOGW(TAG, "intercom capture probe task create failed");
            }
#endif
#if APP_INTERCOM_IMAGE_REAL_CAPTURE_MS > 0
            // Live-call capture/JPEG uses CPU0 idle time below radio/burst priority.
            if (xTaskCreatePinnedToCore(intercom_capture_feed_task, "img_feed",
                                        APP_IMAGE_TASK_STACK_BYTES, nullptr,
                                        kIntercomImageTaskPriority, nullptr,
                                        kIntercomImageTaskCore) != pdPASS) {
                ESP_LOGW(TAG, "intercom capture feed task create failed");
            }
#endif

            // PIR sensor on GPIO12: delay 5s then arm high-level trigger
            // (allow residual touch IC signals to settle after power-on)
            gpio_config_t pir_cfg = {
                .pin_bit_mask = 1ULL << APP_PIR_GPIO,
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&pir_cfg);
            gpio_install_isr_service(0); // OK if already installed
            ESP_LOGI(TAG, "PIR: GPIO%d configured, arming in 5s...", APP_PIR_GPIO);
            const esp_timer_create_args_t pir_arm_args = {
                .callback = pir_arm_timer_cb,
                .arg = &g_radio,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "pir_arm",
                .skip_unhandled_events = true,
            };
            esp_timer_handle_t pir_arm_timer = nullptr;
            esp_timer_create(&pir_arm_args, &pir_arm_timer);
            esp_timer_start_once(pir_arm_timer, 5000000ULL); // 5s
        }
    }
    log_heap_state("after radio");
#else
    ESP_LOGW(TAG, "radio feature disabled for camera/audio isolation");
#endif
#endif
    if (g_app_mode == AppMode::camera) {
        if ((e = g_camera_uart.init()) != ESP_OK) {
            ESP_LOGE(TAG, "camera init: %s", esp_err_to_name(e));
        }
#if APP_CAMERA_UART_ENABLE
        if ((e = g_camera_uart.start()) != ESP_OK) {
            ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
        }
#endif
    }
    if (g_app_mode != AppMode::camera) {
        if ((e = bsp_lcd_init()) != ESP_OK) {
            ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
        } else if (g_app_mode == AppMode::radio) {
            if ((e = bsp_lcd_start_gateway_ui()) != ESP_OK) {
                ESP_LOGE(TAG, "gateway ui start: %s", esp_err_to_name(e));
            } else {
                e = gateway_image_pipeline_init();
                if (e != ESP_OK) {
                    ESP_LOGE(TAG, "gateway image pipeline init: %s", esp_err_to_name(e));
                } else {
                    ui_gw_set_capture_cb(on_gw_capture);
                }
                if ((e = visitor_store_init()) != ESP_OK) {
                    ESP_LOGE(TAG, "visitor store init: %s", esp_err_to_name(e));
                } else {
                    visitor_store_set_thumb_fn(ui_gw_make_thumbnail);
                    visitor_store_set_playback_sinks(on_visitor_play_frame,
                                                     on_visitor_play_audio,
                                                     on_visitor_play_done);
                }
                ui_gw_set_pir_trigger_cb(on_gw_pir_trigger_change);
                ui_gw_set_low_power_cb(on_gw_low_power_change);
                ui_gw_set_jpeg_quality_cb(on_gw_jpeg_quality_change);
                ui_gw_set_intercom_cb(on_gw_intercom_change);
                ui_gw_set_rx_abort_cb(on_gw_rx_abort);
            }
        }
    }
#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_button_init(on_button, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "btn init: %s", esp_err_to_name(e));
    }

    // Create the gateway K1 role-switch timer.
    const esp_timer_create_args_t ptt_timer_args = {
        .callback = ptt_long_press_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ptt_long",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&ptt_timer_args, &g_ptt_timer);

    // Supply voltage monitor; the radio path embeds the cached value in
    // ImageStart and Vbat packets.
    if ((e = bsp_vbat_monitor_start(0)) != ESP_OK) {
        ESP_LOGW(TAG, "vbat monitor start: %s", esp_err_to_name(e));
    }

    g_audio.play_startup_chime();

    ESP_LOGI(TAG, "audio config: %u Hz", APP_AUDIO_SAMPLE_RATE_HZ);
#if APP_RADIO_FEATURES_ENABLE
#if APP_RADIO_TASKS_ENABLE
    ESP_LOGI(TAG, "voice config: Opus %u Hz, %u ms, %d bps CBR; FLRC %lu Hz, %lu bps",
             APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_FRAME_MS, APP_OPUS_BITRATE_BPS,
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS);
#else
    ESP_LOGW(TAG, "FLRC radio init only; RX/TX tasks disabled in this build");
#endif
#else
    ESP_LOGW(TAG, "FLRC voice disabled in this build");
#endif
#else
    ESP_LOGW(TAG, "audio/radio/button features disabled");
#endif
    ESP_LOGI(TAG, "display: ST7789T3 %ux%u camera capture UI",
             APP_LCD_H_RES, APP_LCD_V_RES);
    ESP_LOGI(TAG, "buttons: RST=reset, BOOT=download, side K3=hardware power");
    ESP_LOGI(TAG, "buttons: gateway K1=Latest/hold role switch, K2=Settings, "
                  "K4=Visitors, K5=session");
    ESP_LOGI(TAG, "buttons: door station K2=role switch, K5=doorbell; K1/K4 unbound");
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    if (xTaskCreatePinnedToCore(cpu_stats_task, "cpu_stats",
                                kCpuStatsTaskStackBytes, nullptr,
                                kCpuStatsTaskPriority, nullptr,
                                kCpuStatsTaskCore) != pdPASS) {
        ESP_LOGW(TAG, "CPU stats task create failed");
    }
#endif
}

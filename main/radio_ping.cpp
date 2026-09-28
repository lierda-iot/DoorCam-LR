#include "radio_ping.hpp"

#include <cstring>
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"
#include "bsp.h"
#include "spi_psram_dma_wrap.h"

#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_system.h"

extern volatile bool g_low_power_enabled;

namespace {
constexpr const char *TAG = "radio_ping";

ral_flrc_pkt_params_t flrc_packet_params(uint16_t payload_len)
{
    ral_flrc_pkt_params_t params = {};
    params.preamble_len = APP_FLRC_PREAMBLE_LEN;
    params.sync_word_len = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    params.tx_syncword = RAL_FLRC_TX_SYNCWORD_1;
    params.match_sync_word = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    params.pld_is_fix = false;
    params.pld_len_in_bytes = payload_len;
    // FIFO IRQ status cannot identify a corrupt packet within a burst.
    params.crc_type = RAL_FLRC_CRC_OFF;
    return params;
}

// Caller holds the radio API lock and has stopped the previous RX/TX.
ral_status_t set_flrc_payload_length(const ral_t *radio, uint16_t payload_len)
{
    if (payload_len < 6 || payload_len > APP_FLRC_MAX_PAYLOAD_BYTES) {
        ESP_LOGE(TAG, "Invalid FLRC payload length: %u", payload_len);
        return RAL_STATUS_ERROR;
    }
    const auto params = flrc_packet_params(payload_len);
    const auto status = ral_set_flrc_pkt_params(radio, &params);
    if (status != RAL_STATUS_OK) {
        ESP_LOGW(TAG, "FLRC payload length %u failed: %d", payload_len, status);
    }
    return status;
}

constexpr uint8_t kSyncWord[4] = {
    APP_FLRC_SYNC_WORD_0,
    APP_FLRC_SYNC_WORD_1,
    APP_FLRC_SYNC_WORD_2,
    APP_FLRC_SYNC_WORD_3,
};
constexpr uint8_t kMagic[4] = { 'L', 'R', 'P', '1' };
constexpr uint8_t kPacketTypePing = 1;
constexpr uint8_t kPacketTypeVoice = 2;
constexpr uint8_t kPacketTypeImageCmd = 3;
constexpr uint8_t kPacketTypeImageData = 4;
constexpr uint8_t kPacketTypeImageNack = 5;
constexpr uint8_t kPacketTypeImageDone = 6;
constexpr uint8_t kPacketTypeImageEOT = 7;
constexpr uint8_t kPacketTypeImageStart = 8;
constexpr uint8_t kPacketTypeConfig = 9;
constexpr uint8_t kPacketTypeConfigAck = 10;
constexpr uint8_t kPacketTypeImageCmdAck = 11;
constexpr uint8_t kPacketTypeVbat = 12;
/* Optional padded diagnostic packet for in-call slot timing. */
constexpr uint8_t kPacketTypeIntercomProbe = 13;
/* Doorbell press at the node: unacknowledged short burst, same shape as
 * kPacketTypeVbat (header + payload + CRC32). */
constexpr uint8_t kPacketTypeDoorbell = 14;
constexpr uint16_t kHeaderSize = 14;
constexpr uint8_t kImageStartAvVersion = 2;
constexpr size_t kImageStartAvSize = 24;
constexpr size_t kRxStreamCapacity = 2048;
constexpr size_t kRxPacketMalformed = static_cast<size_t>(-1);
constexpr uint8_t kVoiceFlagMaster = 0x01;
constexpr uint8_t kVoiceFlagNodeReply = 0x02;
constexpr uint8_t kVoiceFlagStart = 0x04;
constexpr uint8_t kVoiceFlagStop = 0x08;
constexpr uint8_t kVoiceFlagStopAck = 0x10;
constexpr uint8_t kVoiceFlagMask = kVoiceFlagMaster | kVoiceFlagNodeReply |
    kVoiceFlagStart | kVoiceFlagStop | kVoiceFlagStopAck;

// Worst-case cost of one 510-byte FLRC fragment (~1.7 ms on air at 2.6 Mbps
// plus FIFO write, command, IRQ and task wake-up). Do not start a fragment
// unless this much remains before the slot-tail deadline.
constexpr int64_t kIntercomImageFragmentStartBudgetUs = 3000;

// Continuous-stream fast path. The gateway places the next session in bytes
// 12..13 of the previous frame's final empty-missing ACK. The node holds that
// session until the active capture task releases its busy state.
std::atomic<bool> s_image_stream_active{false};
std::atomic<uint16_t> s_chained_capture_session{0};
std::atomic<uint16_t> s_image_tx_session{0};
uint16_t s_image_rx_done_next_session = 0;

/* Low-power node battery voltage maintenance cadence and broadcast interval. */
constexpr uint32_t kVbatLowPowerSampleIntervalMs = 60000;     /* 60 s */
constexpr uint32_t kVbatBroadcastIntervalMs = 300000;         /* 5 min */
constexpr uint32_t kConfigAckTimeoutMs = 500;

int32_t abs16(int16_t v)
{
    return v < 0 ? -static_cast<int32_t>(v) : v;
}

void apply_intercom_playback_gain(int16_t *pcm, size_t samples)
{
    const int32_t gain_percent =
        static_cast<int32_t>(APP_INTERCOM_PLAYBACK_PERCENT);
    for (size_t i = 0; i < samples; ++i) {
        const int32_t scaled = static_cast<int32_t>(pcm[i]) * gain_percent;
        pcm[i] = static_cast<int16_t>(scaled / 100);
    }
}

void apply_av_playback_gain(int16_t *pcm, size_t samples)
{
    const int32_t gain_percent = static_cast<int32_t>(APP_AV_PLAYBACK_PERCENT);
    for (size_t i = 0; i < samples; ++i) {
        const int32_t scaled = static_cast<int32_t>(pcm[i]) * gain_percent;
        pcm[i] = static_cast<int16_t>(scaled / 100);
    }
}

void log_intercom_heap(const char *stage)
{
    constexpr uint32_t kInternalCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    constexpr uint32_t kDmaCaps =
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
    ESP_LOGI(TAG,
             "intercom heap %s: internal free=%u largest=%u | "
             "dma free=%u largest=%u",
             stage,
             static_cast<unsigned>(heap_caps_get_free_size(kInternalCaps)),
             static_cast<unsigned>(
                 heap_caps_get_largest_free_block(kInternalCaps)),
             static_cast<unsigned>(heap_caps_get_free_size(kDmaCaps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(kDmaCaps)));
}

void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

uint16_t get_u16_le(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t get_u32_le(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

uint32_t crc32_ieee(const uint8_t *data, size_t len);

size_t rx_packet_size_from_header(const uint8_t *data, size_t available)
{
    if (available < kHeaderSize) {
        return 0;
    }

    switch (data[4]) {
    case kPacketTypeVoice: {
        const uint8_t frame_count = data[12];
        if (frame_count > APP_FLRC_OPUS_FRAMES_PER_PACKET) {
            return kRxPacketMalformed;
        }
        size_t offset = kHeaderSize;
        for (uint8_t i = 0; i < frame_count; ++i) {
            if (offset >= available) {
                return 0;
            }
            const uint8_t opus_len = data[offset++];
            if (opus_len == 0 || opus_len > APP_OPUS_MAX_PACKET_BYTES ||
                offset + opus_len > APP_FLRC_VOICE_MAX_PAYLOAD_BYTES) {
                return kRxPacketMalformed;
            }
            if (offset + opus_len > available) {
                return 0;
            }
            offset += opus_len;
        }
        return offset + 4U; // Software CRC32 follows the voice payload.
    }
    case kPacketTypeImageData:
    case kPacketTypeIntercomProbe:
        // Both paths are padded to one fixed physical FLRC payload.
        return APP_FLRC_BURST_PAYLOAD_LEN;
    case kPacketTypeImageNack: {
        const uint16_t missing_count = get_u16_le(&data[8]);
        if (missing_count > APP_IMAGE_NACK_MAX_INDICES) {
            return kRxPacketMalformed;
        }
        return kHeaderSize + static_cast<size_t>(missing_count) * 2U + 4U;
    }
    case kPacketTypeImageStart: {
        if (data[5] == kImageStartAvVersion) return kImageStartAvSize;
        if (data[5] != 1) return kRxPacketMalformed;
        /* Version 1 was used for all three lengths; the CRC finds their
         * boundary. Newer senders carry an explicit version. */
        const size_t legacy_sizes[] = {18U, 20U, 24U};
        for (size_t size : legacy_sizes) {
            if (available < size) return 0;
            if (get_u32_le(data + size - 4U) == crc32_ieee(data, size - 4U)) {
                return size;
            }
        }
        return kRxPacketMalformed;
    }
    case kPacketTypeVbat:
    case kPacketTypeDoorbell:
        return kHeaderSize + 6U;
    case kPacketTypePing:
    case kPacketTypeImageCmd:
    case kPacketTypeImageDone:
    case kPacketTypeImageEOT:
    case kPacketTypeConfig:
    case kPacketTypeConfigAck:
    case kPacketTypeImageCmdAck:
        return kHeaderSize + 4U;
    default:
        return kRxPacketMalformed;
    }
}

TickType_t ms_to_ticks_min_1(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : (crc >> 1);
        }
    }
    return ~crc;
}

// These packet builders predate software integrity protection. Voice appends
// its CRC in build_voice_packet; control packets append it in send_single_packet.
bool needs_crc32_trailer(uint8_t type)
{
    switch (type) {
    case kPacketTypeVoice:
    case kPacketTypePing:
    case kPacketTypeImageCmd:
    case kPacketTypeImageNack:
    case kPacketTypeImageDone:
    case kPacketTypeImageEOT:
    case kPacketTypeConfig:
    case kPacketTypeConfigAck:
    case kPacketTypeImageCmdAck:
        return true;
    default:
        return false;
    }
}

bool rx_packet_crc_valid(const uint8_t *data, size_t size)
{
    if (size < kHeaderSize) return false;
    if (data[4] == kPacketTypeImageData) {
        const size_t frag_len = get_u16_le(data + 12);
        if (frag_len == 0 || frag_len > APP_IMAGE_FRAGMENT_DATA_SIZE ||
            kHeaderSize + frag_len + 2U > size) return false;
        // Magic is checked by the stream parser; CRC16 covers type through data.
        return get_u16_le(data + kHeaderSize + frag_len) ==
               crc16_ccitt(data + 4, kHeaderSize - 4 + frag_len);
    }
    // All other accepted packet types carry CRC32, including the padded probe.
    return size >= kHeaderSize + 4U &&
           get_u32_le(data + size - 4U) == crc32_ieee(data, size - 4U);
}

} // namespace

RadioPing *RadioPing::instance_ = nullptr;

esp_err_t RadioPing::init_rx_stream()
{
    if (rx_stream_buf_ != nullptr) {
        rx_stream_size_ = 0;
        return ESP_OK;
    }
    // Prefer internal RAM: this buffer is on the RX hot path (FIFO memcpy,
    // magic scan, memmove per packet) and would compete with the LCD/camera
    // streams for MSPI in PSRAM. Fall back to PSRAM rather than fail.
    rx_stream_buf_ = static_cast<uint8_t *>(
        heap_caps_malloc(kRxStreamCapacity, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    const bool internal = rx_stream_buf_ != nullptr;
    if (!internal) {
        rx_stream_buf_ = static_cast<uint8_t *>(
            heap_caps_malloc(kRxStreamCapacity,
                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    }
    if (rx_stream_buf_ == nullptr) {
        ESP_LOGE(TAG, "RX stream alloc failed: %u bytes",
                 static_cast<unsigned>(kRxStreamCapacity));
        return ESP_ERR_NO_MEM;
    }
    rx_stream_size_ = 0;
    ESP_LOGI(TAG, "RX stream buffer: %u bytes in %s",
             static_cast<unsigned>(kRxStreamCapacity),
             internal ? "internal RAM" : "PSRAM (internal heap full)");
    return ESP_OK;
}

esp_err_t RadioPing::init()
{
    instance_ = this;

    esp_err_t err = codec_.init();
    if (err != ESP_OK) {
        return err;
    }
    codec_lock_ = xSemaphoreCreateMutex();
    if (codec_lock_ == nullptr) {
        ESP_LOGE(TAG, "codec lock alloc failed");
        return ESP_ERR_NO_MEM;
    }

    voice_queue_ = xQueueCreate(APP_VOICE_RX_QUEUE_LEN, sizeof(VoicePacket));
    if (voice_queue_ == nullptr) {
        ESP_LOGE(TAG, "voice queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    tx_queue_ = xQueueCreate(APP_VOICE_TX_QUEUE_LEN, sizeof(TxFrame));
    if (tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    image_tx_queue_ = xQueueCreate(1, sizeof(ImageTxRequest));
    if (image_tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "image tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    err = init_rx_stream();
    if (err != ESP_OK) {
        return err;
    }

#if !APP_RADIO_HW_INIT_ENABLE
    ESP_LOGW(TAG, "LR2021 hardware init disabled for camera isolation");
    return ESP_OK;
#endif

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_reset(&radio_.ral);
    if (status == RAL_STATUS_OK) status = ral_init(&radio_.ral);
    if (status == RAL_STATUS_OK &&
        app_spi_polling_dma_prepare(static_cast<spi_host_device_t>(
            CONFIG_LR2021_RADIO_SPI_ID)) != ESP_OK) {
        status = RAL_STATUS_ERROR;
    }
    if (status == RAL_STATUS_OK) {
        status = ral_set_rx_tx_fallback_mode(&radio_.ral, RAL_FALLBACK_STDBY_XOSC);
    }
    if (status == RAL_STATUS_OK) status = ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    if (status == RAL_STATUS_OK && !configure_flrc()) status = RAL_STATUS_ERROR;
    if (status == RAL_STATUS_OK) {
        status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    }
    smtc_modem_hal_irq_config_radio_irq(&RadioPing::irq_callback, this);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "direct RAL init failed: %d", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LR2021 direct RAL initialized: FLRC rf=%lu Hz br=%lu bps bw=%lu Hz",
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS, APP_FLRC_BANDWIDTH_HZ);
#if APP_RADIO_AUTO_RX_ENABLE
    schedule_rx();
#else
    ESP_LOGW(TAG, "LR2021 auto RX disabled for camera isolation");
#endif
    return ESP_OK;
}

esp_err_t RadioPing::init_gateway()
{
    instance_ = this;
    is_gateway_ = true;

    esp_err_t err = codec_.init();
    if (err != ESP_OK) {
        return err;
    }
    codec_lock_ = xSemaphoreCreateMutex();
    if (codec_lock_ == nullptr) {
        ESP_LOGE(TAG, "codec lock alloc failed");
        return ESP_ERR_NO_MEM;
    }

    voice_queue_ = xQueueCreate(APP_VOICE_RX_QUEUE_LEN, sizeof(VoicePacket));
    if (voice_queue_ == nullptr) {
        ESP_LOGE(TAG, "voice queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    tx_queue_ = xQueueCreate(APP_VOICE_TX_QUEUE_LEN, sizeof(TxFrame));
    if (tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    err = init_rx_stream();
    if (err != ESP_OK) {
        return err;
    }

#if !APP_RADIO_HW_INIT_ENABLE
    ESP_LOGW(TAG, "LR2021 hardware init disabled");
    return ESP_OK;
#endif

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_reset(&radio_.ral);
    if (status == RAL_STATUS_OK) status = ral_init(&radio_.ral);
    if (status == RAL_STATUS_OK &&
        app_spi_polling_dma_prepare(static_cast<spi_host_device_t>(
            CONFIG_LR2021_RADIO_SPI_ID)) != ESP_OK) {
        status = RAL_STATUS_ERROR;
    }
    if (status == RAL_STATUS_OK) {
        status = ral_set_rx_tx_fallback_mode(&radio_.ral, RAL_FALLBACK_STDBY_XOSC);
    }
    if (status == RAL_STATUS_OK) status = ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    if (status == RAL_STATUS_OK && !configure_flrc()) status = RAL_STATUS_ERROR;
    if (status == RAL_STATUS_OK) {
        status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    }
    smtc_modem_hal_irq_config_radio_irq(&RadioPing::irq_callback, this);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "direct RAL init failed: %d", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LR2021 gateway role (RX-only): FLRC rf=%lu Hz br=%lu bps bw=%lu Hz",
             APP_FLRC_FREQUENCY_HZ, APP_FLRC_BITRATE_BPS, APP_FLRC_BANDWIDTH_HZ);
#if APP_RADIO_AUTO_RX_ENABLE
    schedule_rx();
#endif
    return ESP_OK;
}

esp_err_t RadioPing::start_gateway()
{
    BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "radio_ping",
                                            APP_RADIO_TASK_STACK_BYTES, this,
                                            APP_RADIO_TASK_PRIORITY, &task_handle_,
                                            APP_RADIO_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(play_task_trampoline, "voice_play",
                                 APP_VOICE_PLAY_TASK_STACK_BYTES, this,
                                 APP_VOICE_PLAY_TASK_PRIORITY, nullptr,
                                 APP_VOICE_PLAY_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(tx_task_trampoline, "voice_tx",
                                 APP_VOICE_TX_TASK_STACK_BYTES, this,
                                 APP_VOICE_TX_TASK_PRIORITY, nullptr,
                                 APP_VOICE_TX_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::start_intercom_local(uint16_t session)
{
    intercom_session_ = session;
    intercom_prepared_session_ = session;
    intercom_start_confirmed_ = false;
    intercom_stop_confirmed_ = false;
    intercom_stop_requested_ = false;
    intercom_stop_reply_ = false;
    intercom_reply_pending_ = false;
    intercom_last_sync_ms_ = smtc_modem_hal_get_time_in_ms();
    intercom_tx_slots_ = 0;
    intercom_rx_slots_ = 0;
    intercom_missed_slots_ = 0;
    // Keep diagnostic counters scoped to one call.
    intercom_probe_tx_ = 0;
    intercom_probe_rx_ = 0;
    intercom_probe_sent_ = 0;
    intercom_probe_deadline_stops_ = 0;
    intercom_img_rearm_target_us_ = 0;
    intercom_img_rearm_min_slack_us_ = 0;
    intercom_img_rearm_slack_valid_ = false;
    intercom_img_rearm_late_ = 0;
    intercom_masters_tx_ = 0;
    intercom_voice_rx_ = 0;
    intercom_rearm_after_rx_ = 0;
    intercom_img_cursor_ = 0;
    intercom_img_frames_tx_ = 0;
    intercom_img_frames_rx_ = 0;
    intercom_img_frags_rx_ = 0;
    intercom_img_frames_adopted_ = 0;
    intercom_img_frag_tx_attempts_ = 0;
    intercom_img_frag_tx_done_ = 0;
    intercom_img_frag_tx_timeouts_ = 0;
    intercom_img_sessions_seen_ = 0;
    intercom_img_sessions_unseen_ = 0;
    intercom_img_sessions_incomplete_ = 0;
    intercom_img_missing_unique_ = 0;
    intercom_img_frags_unique_ = 0;
    intercom_img_frags_duplicate_ = 0;
    intercom_img_frag_seq_lost_ = 0;
    intercom_img_crc_errors_ = 0;
    intercom_img_malformed_ = 0;
    intercom_img_alloc_failures_ = 0;
    intercom_img_reassemble_failures_ = 0;
    intercom_img_rx_session_ = 0;
    intercom_img_rx_active_ = false;
    intercom_img_expected_frag_ = 0;
    intercom_img_have_expected_frag_ = false;
    intercom_img_current_complete_ = false;
    intercom_img_rx_frame_start_ms_ = 0;
    intercom_img_rate_ms_ = smtc_modem_hal_get_time_in_ms();
    // Gateway per-frame display de-dup state: clear it so the first frame of THIS
    // call is always pushed even if a prior call ended on the same session id.
    intercom_img_shown_valid_ = false;
    intercom_img_shown_session_ = 0;
    rx_crc_errors_ = 0;
    rx_hdr_errors_ = 0;
    rx_unknown_packets_ = 0;
    intercom_mic_frames_ = 0;
    intercom_play_frames_ = 0;
    intercom_aec_us_total_ = 0;
    intercom_aec_us_max_ = 0;
    intercom_input_clip_samples_ = 0;
    intercom_howl_.reset();
    intercom_mic_clip_max_ = 0;
    intercom_mic_rms_max_ = 0;
    intercom_next_slot_us_ = esp_timer_get_time() + 20000;
    ptt_active_ = false;
    tx_burst_active_ = false;
    tx_flush_pending_ = false;
    pir_triggered_ = false;
    if (tx_queue_) xQueueReset(tx_queue_);
    if (voice_queue_) xQueueReset(voice_queue_);
    // Under the codec lock: voice_tx / voice_play may be inside Opus on the
    // other core right now.
    codec_lock();
    codec_.reset_encoder();
    codec_.reset_decoder();
    audio_proc_.reset();
    codec_unlock();
    echo_canceller_.reset();
    have_expected_rx_seq_ = false;
    have_expected_play_seq_ = false;
    playback_active_ = false;
    intercom_active_ = true;
    if (!is_gateway_) {
        prepare_intercom_image();  // node: prepare the one-shot image path
        // A call supersedes the ring-time stream. Stop it at the next safe
        // transfer checkpoint and release its frame.
        if (image_tx_active_) {
            ESP_LOGI(TAG, "intercom start: preempting the running image push");
            image_tx_preempt_req_ = true;
        }
    }
    if (intercom_state_cb_) intercom_state_cb_(true);
    cad_wakeup_ms_ = 0;
    // Give a low-power call a fresh hard deadline; the timer ends the call and
    // returns the node to CAD standby.
    arm_lp_window_timer();
    pir_push_wake_ = false;
    if (task_handle_) xTaskNotifyGive(task_handle_);
    ESP_LOGI(TAG, "intercom local start session=%u role=%s", session,
             is_gateway_ ? "gateway" : "node");
    ESP_LOGI(TAG,
             "intercom diagnostics: aec=esp-sr-direct-fd-low-cost ready=%d "
             "low_power=%d cad_active=%d mode=%u tx_core=%u tx_prio=%u "
             "play_core=%u play_prio=%u tx_gain=%u play_gain=%u%%",
             echo_canceller_.ready() ? 1 : 0, g_low_power_enabled ? 1 : 0,
             low_power_cad_active_ ? 1 : 0, static_cast<unsigned>(mode_),
             static_cast<unsigned>(APP_VOICE_TX_TASK_CORE),
             static_cast<unsigned>(APP_VOICE_TX_TASK_PRIORITY),
             static_cast<unsigned>(APP_VOICE_PLAY_TASK_CORE),
             static_cast<unsigned>(APP_VOICE_PLAY_TASK_PRIORITY),
             static_cast<unsigned>(APP_INTERCOM_INPUT_GAIN),
             static_cast<unsigned>(APP_INTERCOM_PLAYBACK_PERCENT));
}

void RadioPing::stop_intercom_local()
{
    // Settle the trailing gateway session before taking the final snapshot so a
    // call that ends mid-JPEG is reported as incomplete instead of disappearing.
    if (is_gateway_) {
        finalize_intercom_image_rx_session();
    }
    log_intercom_image_stats(true);
    intercom_last_stopped_session_ = intercom_session_;
    intercom_active_ = false;
    // Its own mutex serialises this against a process_capture() in flight.
    // Before the state callback: on the node that callback rebuilds the I2S
    // ring at the deep depth (~92 KB of internal DMA RAM), which must not
    // compete with the AEC's internal buffers; a failed rebuild left the
    // microphone dead until the node was rebooted.
    echo_canceller_.deinit();
    if (intercom_state_cb_) intercom_state_cb_(false);
    intercom_reply_pending_ = false;
    intercom_stop_requested_ = false;
    intercom_stop_reply_ = false;
    if (tx_queue_) xQueueReset(tx_queue_);
    if (voice_queue_) xQueueReset(voice_queue_);
    set_playback_pa(false);
    playback_active_ = false;
    // Clear in-call image state before the next ordinary image transfer.
    if (intercom_img_buf_) {
        heap_caps_free(intercom_img_buf_);
        intercom_img_buf_ = nullptr;
        intercom_img_len_ = 0;
    }
    intercom_img_total_frags_ = 0;
    intercom_img_cursor_ = 0;
    if (is_gateway_ && !image_rx_pending_ && image_xfer_.rx_active()) {
        image_xfer_.rx_reset();
    }
    ESP_LOGI(TAG, "intercom local stop session=%u", intercom_session_);
}

bool RadioPing::set_intercom(bool enable)
{
    if (!is_gateway_) return false;
    ESP_LOGI(TAG,
             "intercom request: enable=%d active=%d started=%d stop_req=%d "
             "image_req=%d image_rx=%d suspended=%d mode=%u",
             enable ? 1 : 0, intercom_active_ ? 1 : 0,
             intercom_start_confirmed_ ? 1 : 0, intercom_stop_requested_ ? 1 : 0,
             image_req_active_ ? 1 : 0, image_rx_pending_ ? 1 : 0,
             suspended_ ? 1 : 0, static_cast<unsigned>(mode_));
    if (enable == intercom_active_) {
        ESP_LOGI(TAG, "intercom request already in local state enable=%d", enable ? 1 : 0);
        return true;
    }

    if (enable) {
        // No ImageStart ready-ACK until the call is up (see handle_image_start).
        // Raised before the abort and AEC init so none can slip in.
        intercom_dialing_ = true;
        abort_image_rx();
        const uint32_t abort_start = smtc_modem_hal_get_time_in_ms();
        while (image_rx_abort_req_.load(std::memory_order_acquire) &&
               smtc_modem_hal_get_time_in_ms() - abort_start < 100U) {
            vTaskDelay(ms_to_ticks_min_1(2));
        }
        if (image_rx_abort_req_.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "image abort not consumed before intercom CONFIG");
        }
#if APP_INTERCOM_AEC_ENABLE
        log_intercom_heap("before AEC");
        if (!echo_canceller_.ready() &&
            !echo_canceller_.init()) {
            ESP_LOGE(TAG,
                     "intercom start rejected: gateway ESP-SR direct AEC init failed");
            intercom_dialing_ = false;
            return false;
        }
        log_intercom_heap("after AEC");
#endif
        uint16_t session = static_cast<uint16_t>(intercom_session_ + 1U);
        if (session == 0) session = 1;
        if (!send_config(APP_CFG_KEY_INTERCOM, session)) {
            echo_canceller_.deinit();
            intercom_dialing_ = false;
            return false;
        }
        start_intercom_local(session);

        const uint32_t start = smtc_modem_hal_get_time_in_ms();
        while (!intercom_start_confirmed_ &&
               smtc_modem_hal_get_time_in_ms() - start < APP_INTERCOM_START_TIMEOUT_MS) {
            vTaskDelay(ms_to_ticks_min_1(2));
        }
        intercom_dialing_ = false;
        if (!intercom_start_confirmed_) {
            ESP_LOGW(TAG, "intercom start handshake timeout session=%u", session);
            intercom_stop_requested_ = true;
            if (task_handle_) xTaskNotifyGive(task_handle_);
            const uint32_t stop_start = smtc_modem_hal_get_time_in_ms();
            while (!intercom_stop_confirmed_ &&
                   smtc_modem_hal_get_time_in_ms() - stop_start <
                       APP_INTERCOM_STOP_TIMEOUT_MS) {
                vTaskDelay(ms_to_ticks_min_1(2));
            }
            stop_intercom_local();
            schedule_rx();
            return false;
        }
        ESP_LOGI(TAG, "intercom START confirmed session=%u in %lums", session,
                 static_cast<unsigned long>(smtc_modem_hal_get_time_in_ms() - start));
        return true;
    }

    intercom_stop_confirmed_ = false;
    intercom_stop_requested_ = true;
    if (task_handle_) xTaskNotifyGive(task_handle_);
    const uint32_t start = smtc_modem_hal_get_time_in_ms();
    while (!intercom_stop_confirmed_ &&
           smtc_modem_hal_get_time_in_ms() - start < APP_INTERCOM_STOP_TIMEOUT_MS) {
        vTaskDelay(ms_to_ticks_min_1(2));
    }
    const bool acknowledged = intercom_stop_confirmed_;
    stop_intercom_local();
    schedule_rx();
    if (!acknowledged) ESP_LOGW(TAG, "intercom stop ACK timeout");
    ESP_LOGI(TAG, "intercom local close complete remote_ack=%d", acknowledged ? 1 : 0);
    // Local cleanup is complete even if the final ACK was lost; the node has
    // its own link-loss failsafe.
    return true;
}

esp_err_t RadioPing::start()
{
    BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "radio_ping",
                                            APP_RADIO_TASK_STACK_BYTES, this,
                                            APP_RADIO_TASK_PRIORITY, &task_handle_,
                                            APP_RADIO_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(tx_task_trampoline, "voice_tx",
                                 APP_VOICE_TX_TASK_STACK_BYTES, this,
                                 APP_VOICE_TX_TASK_PRIORITY, nullptr,
                                 APP_VOICE_TX_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(play_task_trampoline, "voice_play",
                                 APP_VOICE_PLAY_TASK_STACK_BYTES, this,
                                 APP_VOICE_PLAY_TASK_PRIORITY, nullptr,
                                 APP_VOICE_PLAY_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(image_tx_task_trampoline, "img_tx",
                                 APP_IMAGE_TASK_STACK_BYTES, this,
                                 APP_IMAGE_TX_TASK_PRIORITY, nullptr,
                                 APP_IMAGE_TX_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (id != APP_PTT_BUTTON) return;
    if (suspended_ || intercom_active_) return;

    ptt_active_ = pressed;
    ESP_LOGI(TAG, "voice button %s -> FLRC voice %s", pressed ? "down" : "up",
             pressed ? "TX" : "RX");

    if (pressed) {
        bool new_burst = !tx_burst_active_;
        tx_burst_active_ = true;
        tx_flush_pending_ = false;

        if (new_burst) {
            set_playback_pa(false);
            playback_active_ = false;
            have_expected_play_seq_ = false;
            codec_lock();
            codec_.reset_encoder();
            audio_proc_.reset();
            codec_unlock();
            if (voice_queue_ != nullptr) {
                xQueueReset(voice_queue_);
            }
            if (tx_queue_ != nullptr) {
                xQueueReset(tx_queue_);
            }
        }

        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }
    } else {
        tx_flush_pending_ = tx_burst_active_;
        if (mode_ == Mode::idle) {
            schedule_tx();
        }
    }
}

void RadioPing::suspend()
{
    suspended_ = true;
    ptt_active_ = false;
    tx_burst_active_ = false;
    tx_flush_pending_ = false;
    irq_pending_ = false;
    rx_stream_size_ = 0;

    if (tx_queue_ != nullptr) {
        xQueueReset(tx_queue_);
    }
    if (voice_queue_ != nullptr) {
        xQueueReset(voice_queue_);
    }

    set_playback_pa(false);
    playback_active_ = false;
    have_expected_play_seq_ = false;

    smtc_modem_hal_protect_api_call();
    (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::idle;
    ESP_LOGI(TAG, "radio suspended");
}

void RadioPing::resume()
{
    smtc_modem_hal_protect_api_call();
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::idle;
    rx_stream_size_ = 0;
    suspended_ = false;
    ESP_LOGI(TAG, "radio resumed");
}

void RadioPing::task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->task();
}

void RadioPing::tx_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->tx_task();
}

void RadioPing::play_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->play_task();
}

void RadioPing::task()
{
    while (true) {
        if (!suspended_) {
            poll_once();
            update_playback_timeout();
            // Consume control-plane aborts before intercom takes the packet plane.
            check_image_rx_abort();
            check_image_rx_quiesce();
            if (intercom_active_) {
                service_intercom();
                ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
                continue;
            }
            service_doorbell_stream();
            check_image_rx_timeout();
            check_image_capture_request();

            uint16_t chained_session =
                s_chained_capture_session.load(std::memory_order_acquire);
            if (chained_session != 0 && !image_tx_active_ && image_capture_cb_ &&
                image_capture_cb_(chained_session)) {
                uint16_t expected = chained_session;
                (void)s_chained_capture_session.compare_exchange_strong(
                    expected, 0, std::memory_order_acq_rel);
                ESP_LOGD(TAG, "chained capture accepted: session=%u", chained_session);
            }

            check_image_req_retry();
        }
        ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
    }
}

void RadioPing::tx_task()
{
    if (is_gateway_) {
        ESP_LOGI(TAG, "gateway mic task parked until intercom starts");
    }
    while (true) {
        // A/V stream (node): keep sampling through an image transfer, or the
        // audio timeline gets a hole as wide as every transfer. This task is on
        // core 1, the transfer on core 0. Any other suspension still parks.
        if (suspended_ && !(av_audio_capture_due() && image_tx_active_)) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        // The gateway has no microphone consumer before a call.
        if (is_gateway_ && !intercom_active_) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        // Low power (node): no mic sampling during CAD standby except while an
        // A/V stream is running (av_audio_capture_due() lapses on its own). The
        // PIR trigger is still handled here after a GPIO wake.
        if (g_low_power_enabled && !is_gateway_ && !intercom_active_ &&
            !av_audio_capture_due()) {
            if (pir_triggered_) {
                pir_triggered_ = false;
                bool dispatched = false;
                if (pir_enabled_) {
                    int64_t now = esp_timer_get_time();
                    if ((now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                        last_trigger_us_ = now;
                        ESP_LOGI(TAG, "PIR trigger! (low power)");
                        if (image_capture_cb_) {
                            image_capture_cb_(trigger_session_id_++);
                            dispatched = true;
                        }
                    }
                }
                // Not dispatched: end the keep-awake guard now instead of
                // waiting for the safety timeout.
                if (!dispatched) {
                    pir_push_wake_ = false;
                }
            }
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        if (!read_mono_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES)) {
            note_mic_read_failure();
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }
        mic_read_fail_since_us_ = 0;

        if (intercom_active_) {
            uint32_t aec_us = 0;
            uint32_t raw_sum_abs = 0;
            int32_t raw_peak = 0;
            for (size_t i = 0; i < APP_AUDIO_FRAME_SAMPLES; ++i) {
                const int32_t level = abs16(tx_pcm_[i]);
                raw_sum_abs += static_cast<uint32_t>(level);
                if (level > raw_peak) raw_peak = level;
            }
#if APP_INTERCOM_AEC_ENABLE
            const int64_t aec_start_us = esp_timer_get_time();
            echo_canceller_.process_capture(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);
            aec_us = static_cast<uint32_t>(esp_timer_get_time() - aec_start_us);
#endif
            uint32_t sum_abs = 0;
            int32_t peak = 0;
            for (size_t i = 0; i < APP_AUDIO_FRAME_SAMPLES; ++i) {
                int32_t level = abs16(tx_pcm_[i]);
                sum_abs += static_cast<uint32_t>(level);
                if (level > peak) peak = level;
                int32_t scaled = static_cast<int32_t>(tx_pcm_[i]) *
                    APP_INTERCOM_INPUT_GAIN;
                if (scaled > 32767) {
                    scaled = 32767;
                    intercom_input_clip_samples_++;
                }
                if (scaled < -32768) {
                    scaled = -32768;
                    intercom_input_clip_samples_++;
                }
                tx_pcm_[i] = static_cast<int16_t>(scaled);
            }
#if APP_INTERCOM_HOWL_ENABLE
            // Feedback killer on the AEC output. Both boxes run it, so a howl
            // opens the loop at whichever end sees it first; the far end then
            // stops hearing it one round trip later and unmutes on its own.
            // See APP_INTERCOM_HOWL_ENABLE.
            {
                const bool was_muted = intercom_howl_.muted();
                const bool mute = intercom_howl_.process(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);
                const HowlSuppressor::Features &f = intercom_howl_.last();
                if (f.clip_percent > intercom_mic_clip_max_) intercom_mic_clip_max_ = f.clip_percent;
                if (f.rms > intercom_mic_rms_max_) intercom_mic_rms_max_ = f.rms;
                if (mute) {
                    std::memset(tx_pcm_, 0, sizeof(tx_pcm_));
                    if (!was_muted) {
                        ESP_LOGW(TAG, "call howl: muting microphone clip=%lu%% rms=%lu events=%lu",
                                 static_cast<unsigned long>(f.clip_percent),
                                 static_cast<unsigned long>(f.rms),
                                 static_cast<unsigned long>(intercom_howl_.mute_events()));
#if APP_INTERCOM_AEC_ENABLE
                        // Whatever got through was more than the steady NLP
                        // could hold; give the filter a warm-up window again.
                        echo_canceller_.rearm_nlp();
#endif
                    }
                } else if (was_muted) {
                    ESP_LOGI(TAG, "call howl: microphone restored after %lu frames%s",
                             static_cast<unsigned long>(intercom_howl_.muted_frames()),
                             intercom_howl_.hit_cap() ? " (cap)" : "");
                }
            }
#endif
            intercom_mic_frames_++;
            intercom_aec_us_total_ += aec_us;
            if (aec_us > intercom_aec_us_max_) intercom_aec_us_max_ = aec_us;
            if (intercom_mic_frames_ == 100U ||
                (intercom_mic_frames_ % 500U) == 0U) {
                // ref_lag: reference-to-echo delay. The filter is causal, so a
                // reference that lags the echo cancels nothing. Healthy is a
                // 0..50 ms sawtooth (one FLRC packet carries 5 x 10 ms frames).
                // clip_max/rms_max: window maxima of the AEC output, the same
                // features the howl killer trips on (a howl shows here first).
                EchoCanceller::ReferenceStats ref;
#if APP_INTERCOM_AEC_ENABLE
                ref = echo_canceller_.reference_stats(true);
#endif
                ESP_LOGI(TAG,
                         "intercom mic: frames=%lu raw_avg=%lu raw_peak=%ld "
                         "aec_avg_abs=%lu aec_peak=%ld "
                         "clip_total=%lu aec_avg=%luus aec_max=%luus "
                         "ref_lag=%lums ref_peak=%lums ref_trim=%lu "
                         "ref_under=%lu ref_over=%lu "
                         "clip_max=%lu%% rms_max=%lu howl_events=%lu howl_muted=%d",
                         static_cast<unsigned long>(intercom_mic_frames_),
                         static_cast<unsigned long>(
                             raw_sum_abs / APP_AUDIO_FRAME_SAMPLES),
                         static_cast<long>(raw_peak),
                         static_cast<unsigned long>(sum_abs / APP_AUDIO_FRAME_SAMPLES),
                         static_cast<long>(peak),
                         static_cast<unsigned long>(intercom_input_clip_samples_),
                         static_cast<unsigned long>(intercom_aec_us_total_ /
                                                    intercom_mic_frames_),
                         static_cast<unsigned long>(intercom_aec_us_max_),
                         static_cast<unsigned long>(
                             ref.level * 1000U / APP_AUDIO_SAMPLE_RATE_HZ),
                         static_cast<unsigned long>(
                             ref.peak * 1000U / APP_AUDIO_SAMPLE_RATE_HZ),
                         static_cast<unsigned long>(ref.trims),
                         static_cast<unsigned long>(ref.underflows),
                         static_cast<unsigned long>(ref.overflows),
                         static_cast<unsigned long>(intercom_mic_clip_max_),
                         static_cast<unsigned long>(intercom_mic_rms_max_),
                         static_cast<unsigned long>(intercom_howl_.mute_events()),
                         intercom_howl_.muted() ? 1 : 0);
                intercom_mic_clip_max_ = 0;
                intercom_mic_rms_max_ = 0;
            }
            uint8_t encoded[APP_OPUS_MAX_PACKET_BYTES];
            codec_lock();
            const int encoded_len = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                                   encoded,
                                                   APP_OPUS_MAX_PACKET_BYTES);
            codec_unlock();
            if (encoded_len > 0 && encoded_len <= 255) {
                enqueue_voice_frame(encoded, static_cast<uint16_t>(encoded_len));
            }

            // The blocking i2s_channel_read() paces this loop to the 10 ms
            // frame and yields the core; an extra vTaskDelay would backlog the
            // I2S RX DMA ring and drift the AEC reference alignment.
            continue;
        }

        // A/V stream (node): encode and queue for the next image payload. No
        // AEC: nothing is playing on this box in this mode.
        if (av_audio_capture_due()) {
            uint8_t encoded[APP_OPUS_MAX_PACKET_BYTES];
            codec_lock();
            const int encoded_len = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                                  encoded, APP_OPUS_MAX_PACKET_BYTES);
            codec_unlock();
            if (encoded_len > 0 && encoded_len <= APP_OPUS_MAX_PACKET_BYTES) {
                av_audio_push(encoded, static_cast<uint8_t>(encoded_len));
            }
        }

        // The capture path is single-flight; do not race an in-flight transfer.
        if (!image_tx_active_ && pir_triggered_) {
            pir_triggered_ = false;
            if (pir_enabled_) {
                int64_t now = esp_timer_get_time();
                if ((now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                    last_trigger_us_ = now;
                    ESP_LOGI(TAG, "PIR trigger!");
                    if (image_capture_cb_) {
                        image_capture_cb_(trigger_session_id_++);
                    }
                }
            }
        }
    }
}

void RadioPing::play_task()
{
    VoicePacket packet;

    while (true) {
        // In a call, wait only until the ring is down to the guard; if nothing
        // arrived by then, top it up instead (APP_INTERCOM_PLAYOUT_GUARD_MS).
        const TickType_t wait = intercom_fill_wait();
        if (xQueueReceive(voice_queue_, &packet, wait) != pdTRUE) {
            if (wait != portMAX_DELAY && intercom_fill_due()) {
                intercom_fill_frame();
            }
            continue;
        }

        if (suspended_) {
            continue;
        }

        if (!playback_active_) {
            codec_lock();
            codec_.reset_decoder();
            codec_unlock();
            // A restart is a fresh timeline for the howl detector too: whatever
            // it was muting is gone with the ring it was written into.
            av_howl_.reset();
#if APP_INTERCOM_AEC_ENABLE
            // A restart rebuilds the reference timeline from scratch; put the
            // aggressive startup NLP back while the alignment settles.
            echo_canceller_.rearm_nlp();
#endif
        }
        wait_for_jitter_buffer();
        conceal_missing_frames(packet.seq, packet.av_stream);

        codec_lock();
        int decoded = codec_.decode(packet.payload, packet.len, rx_pcm_, APP_AUDIO_FRAME_SAMPLES);
        codec_unlock();
        if (decoded <= 0) {
            ESP_LOGW(TAG, "Opus decode failed: %d", decoded);
            continue;
        }

        if (intercom_active_) {
            apply_intercom_playback_gain(rx_pcm_, static_cast<size_t>(decoded));
        } else if (packet.av_stream) {
            // Raw-encoded on the node: no de-emphasis to undo.
            av_playback_process(rx_pcm_, static_cast<size_t>(decoded), false);
        } else {
            audio_proc_.process_rx_frame(rx_pcm_, static_cast<size_t>(decoded));
        }
        if (intercom_active_) {
            uint32_t sum_abs = 0;
            int32_t peak = 0;
            for (int i = 0; i < decoded; ++i) {
                int32_t level = abs16(rx_pcm_[i]);
                sum_abs += static_cast<uint32_t>(level);
                if (level > peak) peak = level;
            }
            intercom_play_frames_++;
            if (intercom_play_frames_ == 100U ||
                (intercom_play_frames_ % 500U) == 0U) {
                ESP_LOGI(TAG, "intercom playback: frames=%lu avg_abs=%lu peak=%ld",
                         static_cast<unsigned long>(intercom_play_frames_),
                         static_cast<unsigned long>(sum_abs / static_cast<uint32_t>(decoded)),
                         static_cast<long>(peak));
            }
        }
        const bool written = play_mono_frame(rx_pcm_, static_cast<size_t>(decoded));
        intercom_fill_run_ = 0;
#if APP_INTERCOM_AEC_ENABLE
        // Reference after the write, and only if it landed in the DMA ring:
        // every frame must enter both the reference FIFO and the TX ring or
        // neither, or the AEC alignment shifts in the direction the causal
        // filter cannot follow.
        if (written && intercom_active_) {
            echo_canceller_.push_reference(rx_pcm_, static_cast<size_t>(decoded));
        }
#else
        (void)written;
#endif
        last_rx_audio_ms_ = smtc_modem_hal_get_time_in_ms();
        playback_active_ = true;
    }
}

void RadioPing::poll_once()
{
    if (irq_pending_) {
        irq_pending_ = false;
        ral_irq_t irq = RAL_IRQ_NONE;
        smtc_modem_hal_protect_api_call();
        ral_status_t status = ral_get_and_clear_irq_status(&radio_.ral, &irq);
        smtc_modem_hal_unprotect_api_call();
        if (status == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
            handle_irq(irq);
        }
    }

    // CAD watchdog: a lost CAD_DONE IRQ would leave mode_ cad_pending forever.
    // After 2 s force standby + idle so the next pass re-arms.
    if (mode_ == Mode::cad_pending && cad_pending_ms_ != 0 &&
        (smtc_modem_hal_get_time_in_ms() - cad_pending_ms_) >= 2000) {
        ESP_LOGW(TAG, "CAD watchdog: no CAD_DONE in 2s, resetting to idle");
        smtc_modem_hal_protect_api_call();
        ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        smtc_modem_hal_unprotect_api_call();
        cad_pending_ms_ = 0;
        mode_ = Mode::idle;
    }

    // Before the mode branches: the doorbell burst breaks out of RX/CAD itself
    // and leaves the radio in FLRC idle for whichever branch runs next.
    service_doorbell();

    if (mode_ == Mode::idle) {
        if (g_low_power_enabled && !is_gateway_ && !intercom_active_) {
            // CAD sleep is node-only; the gateway stays in continuous FLRC RX
            // so an unsolicited PIR push can reach it.
            if (lp_window_expired_) {
                // Hard deadline fired: everything else unwinds at its own
                // checkpoint. Drop the RX window and PIR guard and sleep.
                if (pir_push_wake_) {
                    ESP_LOGW(TAG, "comm window expired during capture/push, dropping it");
                    pir_push_wake_ = false;
                }
                cad_wakeup_ms_ = 0;
                vbat_maintenance_tick();
                enter_low_power_cad();
            } else if (pir_push_wake_) {
                // PIR push in progress: stay awake-but-idle (no RX, no CAD)
                // until image_tx_task takes the radio. 8 s safety timeout in
                // case the capture never fires.
                if (smtc_modem_hal_get_time_in_ms() - pir_push_wake_ms_ >= 8000) {
                    pir_push_wake_ = false;
                    enter_low_power_cad();
                }
            } else if (cad_wakeup_ms_ != 0 &&
                (smtc_modem_hal_get_time_in_ms() - cad_wakeup_ms_) <
                    APP_LP_COMM_WINDOW_MS) {
                // Fixed-length communication window (APP_LP_COMM_WINDOW_MS from
                // the CAD wakeup, not refreshed by traffic). This comparison is
                // only the idle-path view; the deadline itself is the lp_window_
                // esp_timer, which fires even while image_tx_task owns the radio.
                schedule_rx();
            } else {
                cad_wakeup_ms_ = 0;
                // Awake and idle: sample / broadcast VBAT now rather than ever
                // waking for it.
                vbat_maintenance_tick();
                enter_low_power_cad();
            }
        } else if (tx_burst_active_) {
            schedule_tx();
            if (!tx_burst_active_ && !ptt_active_ && mode_ == Mode::idle) {
                schedule_rx();
            }
        } else if (!ptt_active_) {
            if (low_power_cad_active_) {
                low_power_cad_active_ = false;
                configure_flrc();
                // Only a genuine low-power-off clears the window deadline. A
                // call taking the radio also lands here, and its deadline
                // (armed by start_intercom_local) must keep running.
                if (!g_low_power_enabled) {
                    disarm_lp_window_timer();
                    lp_standby_pending_ = false;
                    ESP_LOGI(TAG, "low power off, back to FLRC RX");
                } else {
                    ESP_LOGI(TAG, "leaving CAD for FLRC (call), window still armed");
                }
            }
            // Non-low-power node: periodic 5-min voltage broadcast. Guarded
            // internally by timestamp so this is cheap to call every idle pass.
            vbat_maintenance_tick();
            schedule_rx();
        }
    }
    taskYIELD();
}

void RadioPing::irq_callback(void *context)
{
    auto *self = static_cast<RadioPing *>(context);
    if (self != nullptr) {
        // Anchor slot timing to the IRQ rather than delayed task dispatch.
        self->last_irq_us_ = esp_timer_get_time();
        self->irq_pending_ = true;
        // If a task is block-waiting for TX_DONE inside send_single_packet, wake
        // that task directly; otherwise wake the main radio task (RX / poll loop).
        TaskHandle_t target = self->tx_done_waiter_;
        if (target == nullptr) {
            target = self->task_handle_;
        }
        if (target != nullptr) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(target, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void RadioPing::handle_irq(ral_irq_t irq)
{
    Mode completed_mode = mode_;

    if (completed_mode == Mode::rx_pending) {
        if ((irq & RAL_IRQ_RX_DONE) != 0) {
            // Keep RX continuous across back-to-back image or intercom packets;
            // re-arming between packets can move the FIFO read pointer mid-frame.
            bool keep_continuous_rx =
                image_rx_pending_ || (intercom_active_ && is_gateway_);
            mode_ = Mode::idle;
            // Preserve the RX_DONE timestamp before packet processing.
            rx_done_us_ = last_irq_us_;
            handle_rx_packet();
            if (keep_continuous_rx) {
                if (mode_ == Mode::idle) {
                    mode_ = Mode::rx_pending;
                }
            } else if (mode_ == Mode::idle && !ptt_active_ && !tx_burst_active_) {
                schedule_rx();
            }
        } else if ((irq & RAL_IRQ_RX_HDR_ERROR) != 0) {
            mode_ = Mode::idle;
            rx_hdr_errors_++;
            if ((rx_hdr_errors_ % 10) == 1) {
                ESP_LOGW(TAG, "RX header errors=%lu", static_cast<unsigned long>(rx_hdr_errors_));
            }
            schedule_rx();
        } else if ((irq & RAL_IRQ_RX_TIMEOUT) != 0) {
            mode_ = Mode::idle;
        } else {
            // Non-terminal IRQ (e.g. FIFO_LEVEL, PREAMBLE_DETECTED)
            // Do NOT reset mode or re-arm — packet still being received
        }
    } else if (completed_mode == Mode::tx_pending) {
        mode_ = Mode::idle;
        if ((irq & RAL_IRQ_TX_DONE) == 0) {
            ESP_LOGW(TAG, "TX irq=0x%08lx", static_cast<unsigned long>(irq));
        }
        if (APP_FLRC_VOICE_TX_GAP_MS > 0) {
            vTaskDelay(ms_to_ticks_min_1(APP_FLRC_VOICE_TX_GAP_MS));
        }
    } else if (completed_mode == Mode::cad_pending) {
        handle_cad_irq(irq);
    }
}

void RadioPing::schedule_rx()
{
    if (mode_ != Mode::idle) return;

    // set_rx starts a new hardware receive epoch. Any trailing software bytes
    // belonged to the FIFO epoch that standby/timeout just ended.
    rx_stream_size_ = 0;

    // The gateway uses continuous RX: an unsolicited PIR push could fall into
    // the timeout + re-arm blind spot. Every gateway TX path leaves RX via
    // ral_set_standby first. Nodes keep the short timeout for CAD sleep.
    uint32_t rx_timeout = APP_FLRC_RX_TIMEOUT_MS;
    if (image_rx_pending_ || is_gateway_ || intercom_active_) {
        rx_timeout = RAL_RX_TIMEOUT_CONTINUOUS_MODE;
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(false);
    // Drop residual bytes only at the existing receive-epoch boundary.
    // This does not fix a transmitter advertising more bytes than it wrote.
    (void)lr20xx_radio_fifo_clear_rx(radio_.ral.context);
    // RX must accept full image fragments after a short voice/control TX.
    ral_status_t status = set_flrc_payload_length(&radio_.ral, APP_FLRC_MAX_PAYLOAD_BYTES);
    if (status == RAL_STATUS_OK) status = ral_set_dio_irq_params(&radio_.ral,
                                                 RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                 RAL_IRQ_RX_HDR_ERROR);
    if (status == RAL_STATUS_OK) status = ral_set_rx(&radio_.ral, rx_timeout);
    smtc_modem_hal_unprotect_api_call();

    if (status == RAL_STATUS_OK) {
        mode_ = Mode::rx_pending;
    } else {
        ESP_LOGW(TAG, "schedule RX failed: %d", status);
    }
}

void RadioPing::schedule_tx()
{
    if (mode_ != Mode::idle) return;
    if (tx_queue_ == nullptr) return;

    UBaseType_t queued = uxQueueMessagesWaiting(tx_queue_);
    if (queued == 0) {
        if (!ptt_active_) {
            tx_flush_pending_ = false;
            tx_burst_active_ = false;
        }
        return;
    }
    if (ptt_active_ && queued < APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        return;
    }

    uint16_t tx_size = 0;
    if (!build_voice_packet(&tx_size)) {
        if (!ptt_active_ && uxQueueMessagesWaiting(tx_queue_) == 0) {
            tx_flush_pending_ = false;
            tx_burst_active_ = false;
        }
        return;
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = set_flrc_payload_length(&radio_.ral, tx_size);
    if (status == RAL_STATUS_OK) status = static_cast<ral_status_t>(lr20xx_radio_fifo_clear_tx(radio_.ral.context));
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, tx_buf_, tx_size);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status == RAL_STATUS_OK) {
        mode_ = Mode::tx_pending;
        // Our own transmit half of the link account. Counted only once the TX is
        // actually armed, so a failed set_tx does not inflate the rate.
        note_air_bytes(tx_size);
        if (!ptt_active_ && uxQueueMessagesWaiting(tx_queue_) == 0) {
            tx_flush_pending_ = false;
        }
    } else {
        ESP_LOGW(TAG, "schedule TX failed: %d", status);
    }
}

bool RadioPing::leave_rx_for_tx()
{
    if (mode_ == Mode::idle) return true;
    if (mode_ != Mode::rx_pending && mode_ != Mode::cad_pending) return false;

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    if (status == RAL_STATUS_OK) {
        status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    }
    smtc_modem_hal_unprotect_api_call();
    irq_pending_ = false;
    cad_pending_ms_ = 0;
    rx_stream_size_ = 0;
    mode_ = Mode::idle;
    return status == RAL_STATUS_OK;
}

void RadioPing::service_intercom()
{
    if (!intercom_active_) return;

    // Low power: the window is a hard deadline for a call too, and this is the
    // only place it can be enforced during one (poll_once skips its low-power
    // branch). Leave like a lost master sync; the gateway resets its own side
    // on the next key press.
    if (g_low_power_enabled && !is_gateway_ && lp_window_expired_) {
        ESP_LOGW(TAG, "comm window expired during intercom: ending the call");
        stop_intercom_local();
        schedule_rx();
        return;
    }

    // A low-power node may still own the radio in LoRa CAD when CONFIG arrives.
    // Intercom requires FLRC before the first START/master slot can be received.
    if (low_power_cad_active_) {
        if (!leave_rx_for_tx()) {
            ESP_LOGW(TAG, "intercom CAD exit deferred: mode=%u",
                     static_cast<unsigned>(mode_));
            return;
        }
        if (!configure_flrc()) {
            intercom_missed_slots_++;
            ESP_LOGW(TAG, "intercom FLRC restore failed");
            return;
        }
        low_power_cad_active_ = false;
        cad_wakeup_ms_ = 0;
        // The call's deadline was armed by start_intercom_local and must survive
        // this CAD->FLRC switch: do NOT disarm it here.
        schedule_rx();
        ESP_LOGI(TAG, "intercom restored FLRC from low-power CAD");
    }
    const int64_t now_us = esp_timer_get_time();

    if (is_gateway_) {
        if (now_us < intercom_next_slot_us_) return;
        const int64_t period_us =
            static_cast<int64_t>(APP_INTERCOM_SLOT_PERIOD_MS) * 1000LL;
        intercom_next_slot_us_ += period_us;
        if (intercom_next_slot_us_ <= now_us) {
            intercom_next_slot_us_ = now_us + period_us;
            intercom_missed_slots_++;
        }
        uint8_t flags = kVoiceFlagMaster;
        if (!intercom_start_confirmed_) flags |= kVoiceFlagStart;
        if (intercom_stop_requested_) flags |= kVoiceFlagStop;
        (void)send_intercom_slot(flags);
        // Periodic per-call transport diagnostics.
        intercom_masters_tx_++;
        if ((intercom_masters_tx_ % 100U) == 1U) {
            ESP_LOGI(TAG,
                     "intercom gw diag masters=%lu voice_rx=%lu probe_rx=%lu "
                     "rearm=%lu sw_crc=%lu hdr=%lu unknown=%lu",
                     static_cast<unsigned long>(intercom_masters_tx_),
                     static_cast<unsigned long>(intercom_voice_rx_),
                     static_cast<unsigned long>(intercom_probe_rx_),
                     static_cast<unsigned long>(intercom_rearm_after_rx_),
                     static_cast<unsigned long>(rx_crc_errors_),
                     static_cast<unsigned long>(rx_hdr_errors_),
                     static_cast<unsigned long>(rx_unknown_packets_));
        }
        if ((intercom_masters_tx_ % 250U) == 0U) {
            log_intercom_image_stats(false);
        }
        return;
    }

    if (intercom_start_confirmed_ &&
        smtc_modem_hal_get_time_in_ms() - intercom_last_sync_ms_ >
            APP_INTERCOM_LINK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "intercom master sync lost; leaving session");
        stop_intercom_local();
        schedule_rx();
        return;
    }
    if (!intercom_start_confirmed_ &&
        smtc_modem_hal_get_time_in_ms() - intercom_last_sync_ms_ >
            APP_INTERCOM_START_TIMEOUT_MS) {
        ESP_LOGW(TAG, "intercom prepared session timed out");
        stop_intercom_local();
        schedule_rx();
        return;
    }
    if (!intercom_reply_pending_ || now_us < intercom_reply_due_us_) return;
    intercom_reply_pending_ = false;
    if (smtc_modem_hal_get_time_in_ms() - intercom_last_sync_ms_ >
        APP_INTERCOM_SYNC_TIMEOUT_MS) {
        intercom_missed_slots_++;
        return;
    }
    uint8_t flags = kVoiceFlagNodeReply;
    if (intercom_stop_reply_) flags |= kVoiceFlagStopAck;
    const bool stop_after_reply = intercom_stop_reply_;
    (void)send_intercom_slot(flags);
    if (stop_after_reply) {
        stop_intercom_local();
        schedule_rx();
    }
}

bool RadioPing::send_intercom_slot(uint8_t flags)
{
    intercom_img_rearm_target_us_ = 0;
    if (!leave_rx_for_tx()) {
        intercom_missed_slots_++;
        return false;
    }
    uint16_t tx_size = 0;
    if (!build_voice_packet(&tx_size, flags)) {
        intercom_missed_slots_++;
        schedule_rx();
        return false;
    }
    const bool ok = send_single_packet(tx_buf_, tx_size,
                                       APP_INTERCOM_TX_TIMEOUT_MS);
    if (ok) intercom_tx_slots_++;
    else {
        intercom_missed_slots_++;
        if (intercom_missed_slots_ == 1U ||
            (intercom_missed_slots_ % 50U) == 0U) {
            ESP_LOGW(TAG, "intercom TX failed role=%s flags=0x%02x mode=%u missed=%lu",
                     is_gateway_ ? "gateway" : "node", flags,
                     static_cast<unsigned>(mode_),
                     static_cast<unsigned long>(intercom_missed_slots_));
        }
    }
    // In-call image fusion: JPEG fragments ride the same uplink window as the
    // voice reply. Empty slots stay empty so the radio returns to RX at once.
    if (ok && intercom_active_ && !is_gateway_ && (flags & kVoiceFlagNodeReply) &&
        (flags & kVoiceFlagStopAck) == 0 && APP_INTERCOM_IMAGE_PROBE > 0) {
        // Send real image data only when a frame is ready.
        if (APP_INTERCOM_IMAGE_ENABLE &&
            (intercom_img_total_frags_ > 0 || intercom_img_pending_ != nullptr)) {
            send_intercom_image_burst(APP_INTERCOM_IMAGE_PROBE);
            if ((intercom_probe_tx_ % 250U) == 0U) {
                log_intercom_image_stats(false);
            }
        }
    }
    schedule_rx();
    if (intercom_img_rearm_target_us_ != 0) {
        if (mode_ == Mode::rx_pending) {
            const int32_t slack_us = static_cast<int32_t>(
                intercom_img_rearm_target_us_ - esp_timer_get_time());
            if (!intercom_img_rearm_slack_valid_ ||
                slack_us < intercom_img_rearm_min_slack_us_) {
                intercom_img_rearm_min_slack_us_ = slack_us;
                intercom_img_rearm_slack_valid_ = true;
            }
            if (slack_us <= 0) {
                intercom_img_rearm_late_++;
            }
        } else {
            intercom_img_rearm_late_++;
        }
        intercom_img_rearm_target_us_ = 0;
    }
    if (((intercom_tx_slots_ + intercom_missed_slots_) % 100U) == 1U) {
        // drops = frames discarded before air; if it tracks rx_lost the loss is
        // queue overflow, not over-the-air.
        ESP_LOGI(TAG, "intercom slots tx=%lu rx=%lu missed=%lu queued=%u drops=%lu",
                 static_cast<unsigned long>(intercom_tx_slots_),
                 static_cast<unsigned long>(intercom_rx_slots_),
                 static_cast<unsigned long>(intercom_missed_slots_),
                 tx_queue_ ? static_cast<unsigned>(uxQueueMessagesWaiting(tx_queue_)) : 0U,
                 static_cast<unsigned long>(tx_queue_drops_));
    }
    return ok;
}

void RadioPing::send_intercom_probe_burst(uint16_t count)
{
    if (count == 0) return;

    const void *ctx = radio_.ral.context;

    // Per-slot deadline: stop the burst BURST_GUARD_US before the next master
    // (master_rx + SLOT_PERIOD) to restore priority and re-arm RX.
    const int64_t next_master_us =
        intercom_reply_due_us_ - static_cast<int64_t>(APP_INTERCOM_NODE_GUARD_US) +
        static_cast<int64_t>(APP_INTERCOM_SLOT_PERIOD_MS) * 1000LL;
    const int64_t deadline_us =
        next_master_us - static_cast<int64_t>(APP_INTERCOM_IMAGE_BURST_GUARD_US);

    // Drop below voice_tx (AEC) for the burst so its reference-FIFO cadence
    // stays aligned.
    const UBaseType_t saved_prio = uxTaskPriorityGet(nullptr);
    vTaskPrioritySet(nullptr, APP_INTERCOM_IMAGE_BURST_PRIORITY);

    tx_done_waiter_ = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(nullptr);

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    // The preceding short packet changed the shared FLRC length register.
    if (set_flrc_payload_length(&radio_.ral, APP_FLRC_BURST_PAYLOAD_LEN) != RAL_STATUS_OK) {
        smtc_modem_hal_unprotect_api_call();
        tx_done_waiter_ = nullptr;
        mode_ = Mode::idle;
        vTaskPrioritySet(nullptr, saved_prio);
        return;
    }
    (void)ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(ctx, LR20XX_RADIO_FALLBACK_FS);
    (void)lr20xx_radio_fifo_clear_tx(ctx);
    smtc_modem_hal_unprotect_api_call();

    // Reuse the member tx_buf_ (send_single_packet already finished for this
    // slot) to keep the radio task's stack free, matching the rest of this path.
    uint8_t *pkt = tx_buf_;
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeIntercomProbe;
    pkt[5] = 1;
    put_u16_le(&pkt[6], static_cast<uint16_t>(intercom_probe_tx_));
    put_u16_le(&pkt[8], intercom_session_);
    std::memset(pkt + 10, 0, APP_FLRC_BURST_PAYLOAD_LEN - 10);

    // One fragment at a time: each loop is a deadline check + AEC-yield point.
    // A prefill pipeline would commit packets before the deadline re-check.
    uint16_t sent = 0;
    for (uint16_t pos = 0; pos < count; pos++) {
        if (esp_timer_get_time() >= deadline_us) {
            intercom_probe_deadline_stops_++;
            break;
        }
        put_u16_le(&pkt[10], pos);
        put_u32_le(pkt + APP_FLRC_BURST_PAYLOAD_LEN - 4U,
                   crc32_ieee(pkt, APP_FLRC_BURST_PAYLOAD_LEN - 4U));
        mode_ = Mode::tx_pending;
        smtc_modem_hal_protect_api_call();
        (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_BURST_PAYLOAD_LEN);
        (void)ral_set_tx(&radio_.ral);
        smtc_modem_hal_unprotect_api_call();
        (void)wait_for_tx_done(50);
        sent++;
    }

    smtc_modem_hal_protect_api_call();
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(
        ctx, LR20XX_RADIO_FALLBACK_STDBY_XOSC);
    smtc_modem_hal_unprotect_api_call();
    mode_ = Mode::idle;
    tx_done_waiter_ = nullptr;

    // Restore full priority BEFORE returning so re-arm RX / next master service
    // runs at the normal radio priority.
    vTaskPrioritySet(nullptr, saved_prio);

    intercom_probe_tx_++;
    intercom_probe_sent_ += sent;
}

void RadioPing::prepare_intercom_image()
{
    // The disabled live-capture path can use a synthetic frame for diagnostics.
    intercom_img_cursor_ = 0;
    intercom_img_frames_tx_ = 0;
    intercom_img_session_ = 0;
    if (APP_INTERCOM_IMAGE_ENABLE == 0) {
        return;
    }
#if APP_INTERCOM_IMAGE_REAL_CAPTURE_MS > 0
    // Live capture publishes JPEGs asynchronously; discard any stale prior-call
    // frame and leave the slot empty until the first new frame arrives.
    if (intercom_img_buf_) {
        heap_caps_free(intercom_img_buf_);
        intercom_img_buf_ = nullptr;
    }
    uint8_t *stale = nullptr;
    taskENTER_CRITICAL(&intercom_img_lock_);
    stale = intercom_img_pending_;
    intercom_img_pending_ = nullptr;
    intercom_img_pending_len_ = 0;
    taskEXIT_CRITICAL(&intercom_img_lock_);
    if (stale) {
        heap_caps_free(stale);
    }
    intercom_img_len_ = 0;
    intercom_img_total_frags_ = 0;
    ESP_LOGI(TAG, "intercom image: real slow-refresh, awaiting first captured frame");
    return;
#else
    const size_t want = APP_INTERCOM_IMAGE_FRAME_BYTES;
    if (!intercom_img_buf_ || intercom_img_len_ != want) {
        if (intercom_img_buf_) {
            heap_caps_free(intercom_img_buf_);
            intercom_img_buf_ = nullptr;
        }
        intercom_img_buf_ = static_cast<uint8_t *>(
            heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!intercom_img_buf_) {
            intercom_img_buf_ = static_cast<uint8_t *>(
                heap_caps_malloc(want, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        }
        if (!intercom_img_buf_) {
            intercom_img_len_ = 0;
            intercom_img_total_frags_ = 0;
            ESP_LOGW(TAG, "intercom image: alloc %u bytes failed, image disabled",
                     static_cast<unsigned>(want));
            return;
        }
        intercom_img_len_ = want;
        // Deterministic pattern so the gateway's CRC16 per fragment is meaningful
        // (all-zero would still pass CRC but hides byte-order/offset bugs).
        for (size_t i = 0; i < want; i++) {
            intercom_img_buf_[i] = static_cast<uint8_t>((i * 31U + 7U) & 0xFFU);
        }
    }
    intercom_img_total_frags_ = static_cast<uint16_t>(
        (intercom_img_len_ + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) /
        APP_IMAGE_FRAGMENT_DATA_SIZE);
    ESP_LOGI(TAG, "intercom image ready: %u bytes, %u frags/frame, %u frags/slot",
             static_cast<unsigned>(intercom_img_len_),
             static_cast<unsigned>(intercom_img_total_frags_),
             static_cast<unsigned>(APP_INTERCOM_IMAGE_PROBE));
#endif  // APP_INTERCOM_IMAGE_REAL_CAPTURE_MS
}

void RadioPing::intercom_image_publish(uint8_t *jpeg, size_t jpeg_len)
{
    // Capture-feed task -> pending slot, newest frame wins. Ownership of an
    // accepted blob moves to the radio task; a dropped blob is freed here.
    if (!jpeg || jpeg_len == 0) {
        if (jpeg) {
            heap_caps_free(jpeg);
        }
        return;
    }
    uint8_t *old = nullptr;
    taskENTER_CRITICAL(&intercom_img_lock_);
    old = intercom_img_pending_;
    intercom_img_pending_ = jpeg;
    intercom_img_pending_len_ = jpeg_len;
    taskEXIT_CRITICAL(&intercom_img_lock_);
    if (old) {
        heap_caps_free(old);
    }
}

void RadioPing::send_intercom_image_burst(uint16_t count)
{
    // Real ImageData: predictive slot-tail deadline, one fragment at a time, the
    // cursor rolling across slots until every fragment was attempted once.
#if APP_INTERCOM_IMAGE_REAL_CAPTURE_MS > 0
    // Adopt the freshest pending JPEG only when no frame is active. A partial frame
    // is always finished before replacement, so sessions can never be spliced.
    if (intercom_img_buf_ == nullptr) {
        uint8_t *fresh = nullptr;
        size_t   fresh_len = 0;
        taskENTER_CRITICAL(&intercom_img_lock_);
        fresh = intercom_img_pending_;
        fresh_len = intercom_img_pending_len_;
        intercom_img_pending_ = nullptr;
        intercom_img_pending_len_ = 0;
        taskEXIT_CRITICAL(&intercom_img_lock_);
        if (fresh) {
            intercom_img_buf_ = fresh;
            intercom_img_len_ = fresh_len;
            intercom_img_total_frags_ = static_cast<uint16_t>(
                (fresh_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) /
                APP_IMAGE_FRAGMENT_DATA_SIZE);
            intercom_img_session_++;  // new frame boundary marker for the gateway
            intercom_img_frames_adopted_++;
        }
    }
#endif
    if (count == 0 || !intercom_img_buf_ || intercom_img_total_frags_ == 0) {
        return;
    }

    const void *ctx = radio_.ral.context;
    const int64_t next_master_us =
        intercom_reply_due_us_ - static_cast<int64_t>(APP_INTERCOM_NODE_GUARD_US) +
        static_cast<int64_t>(APP_INTERCOM_SLOT_PERIOD_MS) * 1000LL;
    const int64_t deadline_us =
        next_master_us - static_cast<int64_t>(APP_INTERCOM_IMAGE_BURST_GUARD_US);
    intercom_img_rearm_target_us_ = next_master_us;

    // Keep normal priority: voice_tx/AEC is on CPU1, and CPU0 voice_play must
    // not delay TX_DONE handling or the RX re-arm.

    tx_done_waiter_ = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(nullptr);

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    // The preceding short packet changed the shared FLRC length register.
    if (set_flrc_payload_length(&radio_.ral, APP_FLRC_BURST_PAYLOAD_LEN) != RAL_STATUS_OK) {
        smtc_modem_hal_unprotect_api_call();
        tx_done_waiter_ = nullptr;
        mode_ = Mode::idle;
        return;
    }
    (void)ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(ctx, LR20XX_RADIO_FALLBACK_FS);
    (void)lr20xx_radio_fifo_clear_tx(ctx);
    smtc_modem_hal_unprotect_api_call();

    ImageTxRequest req;
    req.jpeg = intercom_img_buf_;
    req.jpeg_len = intercom_img_len_;
    // Per-FRAME image session id (not the voice session): the gateway keys frame
    // boundaries off this. In synthetic mode it stays 0 -> gateway shows one frame.
    req.session_id = intercom_img_session_;

    uint8_t *pkt = tx_buf_;
    uint16_t sent = 0;
    for (uint16_t i = 0; i < count; i++) {
        const int64_t now_us = esp_timer_get_time();
        if (now_us + kIntercomImageFragmentStartBudgetUs >= deadline_us) {
            intercom_probe_deadline_stops_++;
            break;
        }
        uint16_t frag = intercom_img_cursor_;
        uint16_t len = build_image_fragment(pkt, req, frag, intercom_img_total_frags_);
        if (len < APP_FLRC_BURST_PAYLOAD_LEN) {
            std::memset(pkt + len, 0, APP_FLRC_BURST_PAYLOAD_LEN - len);
        }
        mode_ = Mode::tx_pending;
        smtc_modem_hal_protect_api_call();
        (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_BURST_PAYLOAD_LEN);
        (void)ral_set_tx(&radio_.ral);
        smtc_modem_hal_unprotect_api_call();
        intercom_img_frag_tx_attempts_++;
        if (wait_for_tx_done(50)) {
            intercom_img_frag_tx_done_++;
        } else {
            intercom_img_frag_tx_timeouts_++;
        }
        sent++;

        intercom_img_cursor_++;
        if (intercom_img_cursor_ >= intercom_img_total_frags_) {
            intercom_img_cursor_ = 0;
            intercom_img_frames_tx_++;
            heap_caps_free(intercom_img_buf_);
            intercom_img_buf_ = nullptr;
            intercom_img_len_ = 0;
            intercom_img_total_frags_ = 0;
            break;
        }
    }

    smtc_modem_hal_protect_api_call();
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(
        ctx, LR20XX_RADIO_FALLBACK_STDBY_XOSC);
    smtc_modem_hal_unprotect_api_call();
    mode_ = Mode::idle;
    tx_done_waiter_ = nullptr;

    intercom_probe_tx_++;       // reuse: bursts issued (denominator for logs)
    intercom_probe_sent_ += sent; // reuse: real fragments actually put on air
}

bool RadioPing::configure_flrc()
{
    ralf_params_flrc_t params = {};
    params.rf_freq_in_hz = APP_FLRC_FREQUENCY_HZ;
    params.output_pwr_in_dbm = APP_FLRC_TX_POWER_DBM;
    params.mod_params.raw_bit_rate = APP_FLRC_RAW_BIT_RATE;
    params.mod_params.cr = APP_FLRC_CODING_RATE;
    params.mod_params.pulse_shape = APP_FLRC_PULSE_SHAPE;
    params.pkt_params = flrc_packet_params(APP_FLRC_MAX_PAYLOAD_BYTES);
    params.sync_word[0] = kSyncWord;
    params.sync_word[1] = nullptr;
    params.sync_word[2] = nullptr;
    params.is_tx = true;
    params.crc_seed = 0xFFFFFFFFUL;
    params.crc_polynomial = 0x04C11DB7UL;
    if (ralf_setup_flrc(&radio_, &params) != RAL_STATUS_OK) {
        return false;
    }

    const void *ctx = radio_.ral.context;
    if (lr20xx_radio_fifo_configure_1024_byte_tx_fifo(ctx) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "configure_flrc: 1024 TX FIFO failed");
        return false;
    }
    if (lr20xx_radio_fifo_configure_1024_byte_rx_fifo(ctx) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "configure_flrc: 1024 RX FIFO failed");
        return false;
    }
    return true;
}

bool RadioPing::build_voice_packet(uint16_t *tx_size, uint8_t flags)
{
    if (tx_size == nullptr || tx_queue_ == nullptr) return false;
    TxFrame frame = {};
    bool have_frame = xQueueReceive(tx_queue_, &frame, 0) == pdTRUE;
    if (!have_frame && flags == 0) return false;

    std::memcpy(tx_buf_, kMagic, sizeof(kMagic));
    tx_buf_[4] = kPacketTypeVoice;
    tx_buf_[5] = 2;
    put_u16_le(&tx_buf_[6], have_frame ? frame.seq : tx_seq_);
    put_u32_le(&tx_buf_[8], smtc_modem_hal_get_time_in_ms());
    tx_buf_[12] = 0;
    tx_buf_[13] = flags & kVoiceFlagMask;

    uint16_t offset = kHeaderSize;
    uint8_t frame_count = 0;
    const uint8_t frame_limit = intercom_active_ ? APP_INTERCOM_FRAMES_PER_PACKET :
                                                   APP_FLRC_OPUS_FRAMES_PER_PACKET;
    while (have_frame && frame_count < frame_limit) {
        if (frame.len == 0 || frame.len > APP_OPUS_MAX_PACKET_BYTES ||
            offset + 1U + frame.len > APP_FLRC_VOICE_MAX_PAYLOAD_BYTES) {
            break;
        }

        tx_buf_[offset++] = static_cast<uint8_t>(frame.len);
        std::memcpy(&tx_buf_[offset], frame.payload, frame.len);
        offset = static_cast<uint16_t>(offset + frame.len);
        frame_count++;

        have_frame = frame_count < frame_limit &&
            xQueueReceive(tx_queue_, &frame, 0) == pdTRUE;
    }

    tx_buf_[12] = frame_count;
    if (intercom_active_ || flags != 0) {
        put_u16_le(&tx_buf_[8], intercom_session_);
    }
    put_u32_le(tx_buf_ + offset, crc32_ieee(tx_buf_, offset));
    *tx_size = static_cast<uint16_t>(offset + 4U);
    return frame_count > 0 || flags != 0;
}

void RadioPing::enqueue_voice_frame(const uint8_t *payload, uint16_t len)
{
    if (!tx_queue_ || !payload || len == 0 || len > APP_OPUS_MAX_PACKET_BYTES) return;
    if (intercom_active_) {
        TxFrame stale = {};
        while (uxQueueMessagesWaiting(tx_queue_) >= APP_INTERCOM_TX_QUEUE_FRAMES &&
               xQueueReceive(tx_queue_, &stale, 0) == pdTRUE) {
            tx_queue_drops_++;
        }
    }
    TxFrame frame = {.seq = tx_seq_++, .len = len, .payload = {}};
    std::memcpy(frame.payload, payload, len);
    if (xQueueSend(tx_queue_, &frame, 0) != pdTRUE) {
        TxFrame dropped = {};
        (void)xQueueReceive(tx_queue_, &dropped, 0);
        tx_queue_drops_++;
        (void)xQueueSend(tx_queue_, &frame, 0);
    }
}

void RadioPing::capture_voice_packet()
{
    if (tx_queue_ == nullptr) {
        return;
    }

    audio_proc_.process_tx_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);

    TxFrame frame = {
        .seq = tx_seq_++,
        .len = 0,
        .payload = {},
    };

    int encoded = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                frame.payload,
                                APP_OPUS_MAX_PACKET_BYTES);
    if (encoded <= 0) {
        ESP_LOGW(TAG, "Opus encode failed: %d", encoded);
        return;
    }
    if (encoded > 255) {
        ESP_LOGW(TAG, "Opus packet too large: %d", encoded);
        return;
    }

    frame.len = static_cast<uint16_t>(encoded);

    if (xQueueSend(tx_queue_, &frame, 0) != pdTRUE) {
        TxFrame dropped;
        (void)xQueueReceive(tx_queue_, &dropped, 0);
        tx_queue_drops_++;
        if (xQueueSend(tx_queue_, &frame, 0) == pdTRUE) {
            if ((tx_queue_drops_ % APP_TX_DROP_LOG_EVERY_N) == 1) {
                ESP_LOGW(TAG, "voice TX queue full, dropped oldest seq=%u drops=%lu",
                         dropped.seq, static_cast<unsigned long>(tx_queue_drops_));
            }
        } else {
            if ((tx_queue_drops_ % APP_TX_DROP_LOG_EVERY_N) == 1) {
                ESP_LOGW(TAG, "voice TX queue full drops=%lu",
                         static_cast<unsigned long>(tx_queue_drops_));
            }
        }
    }
}

void RadioPing::handle_rx_packet()
{
    if (rx_stream_buf_ == nullptr) {
        ESP_LOGE(TAG, "RX stream buffer unavailable");
        return;
    }

    // RX FIFO level is a byte count: back-to-back RX_DONE events can collapse
    // into one wake-up with several packets plus a partial one in the FIFO.
    // Keep the byte stream and restore packet boundaries in software.
    uint16_t level = 0;
    smtc_modem_hal_protect_api_call();
    ral_status_t status = static_cast<ral_status_t>(
        lr20xx_radio_fifo_get_rx_level(radio_.ral.context, &level));
    smtc_modem_hal_unprotect_api_call();
    if (status != RAL_STATUS_OK) {
        ESP_LOGW(TAG, "RX FIFO level failed: %d", status);
        return;
    }
    if (level == 0) {
        return;
    }

    process_rx_stream(rx_stream_rssi_);
    if (static_cast<size_t>(level) > kRxStreamCapacity - rx_stream_size_) {
        ESP_LOGW(TAG, "RX stream overflow reset: buffered=%u fifo=%u",
                 static_cast<unsigned>(rx_stream_size_), level);
        rx_stream_size_ = 0;
    }
    if (static_cast<size_t>(level) > kRxStreamCapacity) {
        ESP_LOGE(TAG, "RX FIFO level exceeds stream capacity: %u", level);
        return;
    }

    uint16_t remaining = level;
    while (remaining > 0) {
        const uint16_t take = remaining > APP_FLRC_MAX_PAYLOAD_BYTES
                                  ? APP_FLRC_MAX_PAYLOAD_BYTES : remaining;
        smtc_modem_hal_protect_api_call();
        status = static_cast<ral_status_t>(lr20xx_radio_fifo_read_rx(
            radio_.ral.context, rx_stream_buf_ + rx_stream_size_, take));
        smtc_modem_hal_unprotect_api_call();
        if (status != RAL_STATUS_OK) {
            ESP_LOGW(TAG, "RX FIFO read failed: %d requested=%u", status, take);
            break;
        }
        rx_stream_size_ += take;
        remaining = static_cast<uint16_t>(remaining - take);
    }

    ral_flrc_rx_pkt_status_t pkt_status = {};
    smtc_modem_hal_protect_api_call();
    const ral_status_t pkt_status_result =
        ral_get_flrc_rx_pkt_status(&radio_.ral, &pkt_status);
    smtc_modem_hal_unprotect_api_call();
    if (pkt_status_result == RAL_STATUS_OK) {
        rx_stream_rssi_ = pkt_status.rssi_sync_in_dbm;
    } else {
        ESP_LOGW(TAG, "RX packet status failed: %d", pkt_status_result);
    }

    process_rx_stream(rx_stream_rssi_);
}

void RadioPing::process_rx_stream(int16_t rssi)
{
    while (rx_stream_size_ > 0) {
        if (rx_stream_size_ < sizeof(kMagic)) {
            return;
        }

        if (std::memcmp(rx_stream_buf_, kMagic, sizeof(kMagic)) != 0) {
            // memchr for the magic's first byte; this is the RX hot path.
            size_t next_magic = 1;
            while (next_magic + sizeof(kMagic) <= rx_stream_size_) {
                const size_t span =
                    rx_stream_size_ - next_magic - (sizeof(kMagic) - 1U);
                const auto *hit = static_cast<const uint8_t *>(
                    std::memchr(rx_stream_buf_ + next_magic, kMagic[0], span));
                if (hit == nullptr) {
                    next_magic = rx_stream_size_;
                    break;
                }
                next_magic = static_cast<size_t>(hit - rx_stream_buf_);
                if (std::memcmp(rx_stream_buf_ + next_magic, kMagic,
                                sizeof(kMagic)) == 0) {
                    break;
                }
                ++next_magic;
            }
            // Discard the prefix; keep the last kMagic-1 bytes, which may hold
            // the head of a magic split across two reads.
            size_t skipped = next_magic;
            if (next_magic + sizeof(kMagic) > rx_stream_size_) {
                const size_t keep = rx_stream_size_ < sizeof(kMagic) - 1U
                                        ? rx_stream_size_ : sizeof(kMagic) - 1U;
                skipped = rx_stream_size_ - keep;
            }

            const bool log_this = ((rx_unknown_packets_ + 1U) % 50U) == 1U;

            std::memmove(rx_stream_buf_, rx_stream_buf_ + skipped,
                         rx_stream_size_ - skipped);
            rx_stream_size_ -= skipped;
            rx_unknown_packets_++;
            if (log_this) {
                ESP_LOGW(TAG,
                         "RX stream resync count=%lu skipped=%u buffered=%u",
                         static_cast<unsigned long>(rx_unknown_packets_),
                         static_cast<unsigned>(skipped),
                         static_cast<unsigned>(rx_stream_size_));
            }
            continue;
        }

        const size_t packet_size =
            rx_packet_size_from_header(rx_stream_buf_, rx_stream_size_);
        if (packet_size == 0) {
            return;
        }
        if (packet_size == kRxPacketMalformed ||
            packet_size > sizeof(rx_buf_)) {
            // Drop one byte, then let the magic search above find the next
            // valid packet boundary. This also recovers from a corrupt header.
            std::memmove(rx_stream_buf_, rx_stream_buf_ + 1,
                         rx_stream_size_ - 1U);
            rx_stream_size_--;
            rx_unknown_packets_++;
            continue;
        }
        if (rx_stream_size_ < packet_size) {
            return;
        }

        // Validate before dispatching: a damaged type/length may describe the
        // wrong boundary, so resync by one byte and keep the rest of the batch.
        if (!rx_packet_crc_valid(rx_stream_buf_, packet_size)) {
            rx_crc_errors_++;
            if (intercom_active_ && is_gateway_ &&
                rx_stream_buf_[4] == kPacketTypeImageData) {
                intercom_img_crc_errors_++;
            }
            if ((rx_crc_errors_ % 50U) == 1U) {
                ESP_LOGW(TAG, "RX software CRC rejects=%lu",
                         static_cast<unsigned long>(rx_crc_errors_));
            }
            std::memmove(rx_stream_buf_, rx_stream_buf_ + 1, rx_stream_size_ - 1U);
            --rx_stream_size_;
            continue;
        }

        std::memcpy(rx_buf_, rx_stream_buf_, packet_size);
        std::memmove(rx_stream_buf_, rx_stream_buf_ + packet_size,
                     rx_stream_size_ - packet_size);
        rx_stream_size_ -= packet_size;
        const size_t dispatch_size = needs_crc32_trailer(rx_buf_[4])
                                         ? packet_size - 4U : packet_size;
        dispatch_rx_packet(static_cast<uint16_t>(dispatch_size), rssi);
    }
}

void RadioPing::dispatch_rx_packet(uint16_t len, int16_t rssi)
{
    if (len < kHeaderSize || std::memcmp(rx_buf_, kMagic, sizeof(kMagic)) != 0) {
        rx_unknown_packets_++;
        if ((rx_unknown_packets_ % 50U) == 1U) {
            // On the RX hot path; kept at DEBUG so it never floods the critical
            // receive loop (raise the log level to see it when debugging EMI).
            ESP_LOGD(TAG, "RX unknown packets=%lu len=%u rssi=%d hdr=%02x%02x%02x%02x",
                     static_cast<unsigned long>(rx_unknown_packets_), len, rssi,
                     rx_buf_[0], rx_buf_[1], rx_buf_[2], rx_buf_[3]);
        }
        return;
    }

    // Air-traffic accounting, after the magic check so only this link's packets
    // count.
    note_air_bytes(len);
    note_air_rssi(rssi);

    // Low power (node): the communication window is not refreshed by traffic,
    // so awake time per wakeup stays bounded.
    if (intercom_active_ && rx_buf_[4] == kPacketTypeConfig && !is_gateway_ &&
        rx_buf_[8] == APP_CFG_KEY_INTERCOM &&
        static_cast<uint16_t>(get_u32_le(&rx_buf_[9])) == intercom_session_) {
        intercom_last_sync_ms_ = smtc_modem_hal_get_time_in_ms();
        send_config_ack(APP_CFG_KEY_INTERCOM, intercom_session_);
        return;
    }
    // Count and discard optional slot-timing probe packets.
    if (intercom_active_ && is_gateway_ && rx_buf_[4] == kPacketTypeIntercomProbe) {
        intercom_probe_rx_++;
        if ((intercom_probe_rx_ % 200U) == 1U) {
            ESP_LOGI(TAG, "intercom probe rx=%lu session=%u rssi=%d",
                     static_cast<unsigned long>(intercom_probe_rx_),
                     get_u16_le(&rx_buf_[8]), rssi);
        }
        return;
    }
    // Handle in-call image fragments before the non-voice rejection gate.
    if (intercom_active_ && is_gateway_ && rx_buf_[4] == kPacketTypeImageData) {
        handle_intercom_image_data(len);
        return;
    }
    if (intercom_active_ && rx_buf_[4] != kPacketTypeVoice) {
        ESP_LOGD(TAG, "intercom ignored packet type=%u", rx_buf_[4]);
        return;
    }

    if (rx_buf_[4] == kPacketTypeVoice) {
        const uint16_t session = get_u16_le(&rx_buf_[8]);
        const uint8_t flags = rx_buf_[13] & kVoiceFlagMask;
        if (!intercom_active_ && flags == 0) {
            (void)queue_voice_packet(len, rssi);
            return;
        }
        if (!intercom_active_ && !is_gateway_ &&
            (flags & kVoiceFlagStop) != 0 &&
            session == intercom_last_stopped_session_) {
            intercom_session_ = session;
            (void)send_intercom_slot(kVoiceFlagNodeReply | kVoiceFlagStopAck);
            return;
        }
        if (!intercom_active_ || session != intercom_session_) {
            ESP_LOGD(TAG, "voice session mismatch rx=%u local=%u", session,
                     intercom_session_);
            return;
        }
        const bool valid = queue_voice_packet(len, rssi);
        if (valid && is_gateway_ && intercom_active_) {
            // Voice replies provide the baseline for appended-fragment metrics.
            intercom_voice_rx_++;
        }
        if (valid && !is_gateway_ && (flags & kVoiceFlagMaster) != 0) {
            if ((flags & kVoiceFlagStart) != 0) {
                if (!intercom_start_confirmed_) {
                    ESP_LOGI(TAG, "intercom START received session=%u rssi=%d", session, rssi);
                }
                intercom_start_confirmed_ = true;
            }
            intercom_last_sync_ms_ = smtc_modem_hal_get_time_in_ms();
            // Base the reply deadline on RX_DONE so task jitter cannot consume the
            // guard before the next master slot.
            intercom_reply_due_us_ =
                (rx_done_us_ != 0 ? rx_done_us_ : esp_timer_get_time()) +
                static_cast<int64_t>(APP_INTERCOM_NODE_GUARD_US);
            intercom_reply_pending_ = true;
            intercom_stop_reply_ = (flags & kVoiceFlagStop) != 0;
            intercom_rx_slots_++;
        } else if (valid && is_gateway_ && (flags & kVoiceFlagNodeReply) != 0) {
            intercom_rx_slots_++;
            if (!intercom_start_confirmed_) {
                ESP_LOGI(TAG, "intercom first node reply session=%u rssi=%d", session, rssi);
            }
            intercom_start_confirmed_ = true;
            if ((flags & kVoiceFlagStopAck) != 0) {
                ESP_LOGI(TAG, "intercom STOP_ACK received session=%u", session);
                intercom_stop_confirmed_ = true;
            }
        }
    } else if (rx_buf_[4] == kPacketTypePing) {
        uint16_t seq = get_u16_le(&rx_buf_[6]);
        log_rx(seq, len, rssi);
    } else if (rx_buf_[4] == kPacketTypeImageCmd) {
        handle_image_cmd();
    } else if (rx_buf_[4] == kPacketTypeImageData) {
        image_rx_last_rssi_ = rssi;
        handle_image_data(len);
    } else if (rx_buf_[4] == kPacketTypeImageNack) {
        handle_image_nack();
    } else if (rx_buf_[4] == kPacketTypeImageDone) {
        handle_image_done();
    } else if (rx_buf_[4] == kPacketTypeImageEOT) {
        handle_image_eot();
    } else if (rx_buf_[4] == kPacketTypeImageStart) {
        handle_image_start(len);
    } else if (rx_buf_[4] == kPacketTypeImageCmdAck) {
        handle_image_cmd_ack();
    } else if (rx_buf_[4] == kPacketTypeConfig) {
        uint8_t key = rx_buf_[8];
        uint32_t value = get_u32_le(&rx_buf_[9]);
        ESP_LOGI(TAG, "RX Config: key=%u value=%lu", key, static_cast<unsigned long>(value));
#if APP_INTERCOM_AEC_ENABLE
        if (key == APP_CFG_KEY_INTERCOM && value != 0 && !is_gateway_ &&
            !echo_canceller_.ready()) {
            log_intercom_heap("node before AEC");
            if (!echo_canceller_.init()) {
                ESP_LOGE(TAG,
                         "intercom CONFIG rejected: node ESP-SR direct AEC init failed");
                return;
            }
            log_intercom_heap("node after AEC");
        }
#endif
        if (config_received_cb_) {
            config_received_cb_(key, value);
        }
        send_config_ack(key, value);
        if (key == APP_CFG_KEY_INTERCOM && value != 0 && !is_gateway_) {
            start_intercom_local(static_cast<uint16_t>(value));
        }
        // Low power: a config exchange is not a photo session; close the
        // window early and return to CAD sleep on the next idle pass.
        if (g_low_power_enabled && !is_gateway_ && cad_wakeup_ms_ != 0) {
            cad_wakeup_ms_ = 0;
            disarm_lp_window_timer();
            ESP_LOGI(TAG, "config ACK sent, closing comm window -> CAD sleep");
        }
    } else if (rx_buf_[4] == kPacketTypeConfigAck) {
        ESP_LOGI(TAG, "RX ConfigAck");
        config_ack_received_ = true;
    } else if (rx_buf_[4] == kPacketTypeVbat) {
        // Battery voltage broadcast: [14..15] vbat_mv, [16..19] CRC32 over [0..15].
        if (len >= kHeaderSize + 6) {
            uint32_t hdr_crc = crc32_ieee(rx_buf_, 16);
            uint32_t rx_crc = get_u32_le(&rx_buf_[16]);
            if (hdr_crc == rx_crc) {
                uint16_t vbat_mv = get_u16_le(&rx_buf_[14]);
                ESP_LOGI(TAG, "RX Vbat: %u mV (%u.%02u V)", vbat_mv, vbat_mv / 1000, (vbat_mv % 1000) / 10);
                if (vbat_mv > 0 && vbat_received_cb_) vbat_received_cb_(vbat_mv);
                // The Vbat broadcast is the node's heartbeat; refresh the status
                // line so RSSI does not go stale.
                link_stats_publish();
            } else {
                ESP_LOGW(TAG, "RX Vbat CRC mismatch, dropping");
            }
        }
    } else if (rx_buf_[4] == kPacketTypeDoorbell) {
        handle_doorbell_packet(len);
    } else {
        ESP_LOGW(TAG, "RX unsupported packet type=%u len=%u rssi=%d", rx_buf_[4], len, rssi);
    }
}

bool RadioPing::queue_voice_packet(uint16_t len, int16_t rssi)
{
    if (!voice_queue_ || len < kHeaderSize) return false;
    uint8_t frame_count = rx_buf_[12];
    const uint8_t flags = rx_buf_[13] & kVoiceFlagMask;
    if (frame_count == 0) {
        return flags != 0 && len == kHeaderSize;
    }
    if (frame_count > APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        ESP_LOGW(TAG, "RX bad voice packet len=%u frames=%u", len, frame_count);
        return false;
    }

    uint16_t seq = get_u16_le(&rx_buf_[6]);
    uint16_t offset = kHeaderSize;
    for (uint8_t i = 0; i < frame_count; i++) {
        if (offset >= len) {
            ESP_LOGW(TAG, "RX truncated voice packet len=%u frames=%u", len, frame_count);
            return false;
        }

        uint8_t opus_len = rx_buf_[offset++];
        if (opus_len == 0 || opus_len > APP_OPUS_MAX_PACKET_BYTES || offset + opus_len > len) {
            ESP_LOGW(TAG, "RX bad voice frame len=%u opus_len=%u", len, opus_len);
            return false;
        }

        uint16_t frame_seq = static_cast<uint16_t>(seq + i);
        log_rx(frame_seq, len, rssi);

        VoicePacket packet = {
            .seq = frame_seq,
            .len = opus_len,
            .rssi = rssi,
            .payload = {},
            .av_stream = false,
        };
        std::memcpy(packet.payload, &rx_buf_[offset], opus_len);
        offset = static_cast<uint16_t>(offset + opus_len);

        if (xQueueSend(voice_queue_, &packet, 0) != pdTRUE) {
            VoicePacket dropped;
            (void)xQueueReceive(voice_queue_, &dropped, 0);
            rx_queue_drops_++;
            if (xQueueSend(voice_queue_, &packet, 0) == pdTRUE) {
                ESP_LOGW(TAG, "voice queue full, dropped oldest seq=%u drops=%lu",
                         dropped.seq, static_cast<unsigned long>(rx_queue_drops_));
            } else {
                ESP_LOGW(TAG, "voice queue full drops=%lu seq=%u",
                         static_cast<unsigned long>(rx_queue_drops_), frame_seq);
            }
        }
    }
    return true;
}

// Whether the node's microphone rides along with requested frames. Every
// requested stream is view-and-listen, so this is compile-time; the ring
// window always carries audio because an unanswered ring is kept with sound.
bool RadioPing::av_audio_wanted() const
{
    return APP_AV_STREAM_ENABLE != 0 || doorbell_session_open();
}

void RadioPing::av_audio_reset()
{
    portENTER_CRITICAL(&av_audio_lock_);
    av_audio_head_ = 0;
    av_audio_count_ = 0;
    av_audio_next_seq_ = 0;
    av_audio_tail_seq_ = 0;
    portEXIT_CRITICAL(&av_audio_lock_);
}

void RadioPing::av_audio_note_activity()
{
    if (!av_audio_wanted() || is_gateway_) return;
    // Re-arming after the window lapsed starts a new stream: drop what the
    // ring still holds so stale audio is not attached to a fresh picture.
    if (!av_audio_capture_due()) {
        av_audio_reset();
    }
    av_audio_active_until_ms_ =
        smtc_modem_hal_get_time_in_ms() + APP_AV_AUDIO_ACTIVE_WINDOW_MS;
}

bool RadioPing::av_audio_capture_due() const
{
    if (!av_audio_wanted() || is_gateway_ ||
        intercom_active_ || (g_low_power_enabled && lp_window_expired_)) return false;
    if (av_audio_active_until_ms_ == 0) return false;
    // Unsigned wrap is intentional: the difference stays small and correct
    // across the 32-bit millisecond rollover, an absolute compare would not.
    return (int32_t)(av_audio_active_until_ms_ - smtc_modem_hal_get_time_in_ms()) > 0;
}

void RadioPing::av_audio_push(const uint8_t *opus, uint8_t len)
{
    if (opus == nullptr || len == 0 || len > APP_OPUS_MAX_PACKET_BYTES) return;

    portENTER_CRITICAL(&av_audio_lock_);
    if (av_audio_count_ == APP_AV_AUDIO_RING_FRAMES) {
        // Drop the oldest frame and advance the tail sequence: a visible gap
        // beats a stale run that drifts behind the video.
        av_audio_tail_seq_ = static_cast<uint16_t>(av_audio_tail_seq_ + 1U);
        av_audio_count_--;
        av_audio_dropped_++;
    } else if (av_audio_count_ == 0) {
        av_audio_tail_seq_ = av_audio_next_seq_;
    }
    AvAudioFrame &slot = av_audio_ring_[av_audio_head_];
    slot.seq = av_audio_next_seq_;
    slot.len = len;
    std::memcpy(slot.data, opus, len);
    av_audio_head_ =
        static_cast<uint16_t>((av_audio_head_ + 1U) % APP_AV_AUDIO_RING_FRAMES);
    av_audio_count_++;
    av_audio_next_seq_ = static_cast<uint16_t>(av_audio_next_seq_ + 1U);
    portEXIT_CRITICAL(&av_audio_lock_);
}

size_t RadioPing::av_audio_drain(uint8_t *out, size_t out_cap,
                                 uint16_t *out_first_seq)
{
    if (out == nullptr || out_cap == 0) return 0;

    size_t used = 0;
    uint16_t expected_seq = 0;
    if (out_first_seq != nullptr) *out_first_seq = 0;
    while (true) {
        AvAudioFrame frame;
        bool got = false;

        // One frame per critical section; this task shares core 0 with the
        // radio ISR.
        portENTER_CRITICAL(&av_audio_lock_);
        if (av_audio_count_ > 0) {
            const uint16_t tail = static_cast<uint16_t>(
                (av_audio_head_ + APP_AV_AUDIO_RING_FRAMES - av_audio_count_) %
                APP_AV_AUDIO_RING_FRAMES);
            // Commit the removal only once the frame is known to fit.
            if (used + 1U + av_audio_ring_[tail].len <= out_cap &&
                (used == 0 || av_audio_ring_[tail].seq == expected_seq)) {
                frame = av_audio_ring_[tail];
                av_audio_count_--;
                av_audio_tail_seq_ = static_cast<uint16_t>(av_audio_tail_seq_ + 1U);
                got = true;
            }
        }
        portEXIT_CRITICAL(&av_audio_lock_);

        if (!got) break;
        if (used == 0 && out_first_seq != nullptr) *out_first_seq = frame.seq;
        expected_seq = static_cast<uint16_t>(frame.seq + 1U);
        out[used++] = frame.len;
        std::memcpy(&out[used], frame.data, frame.len);
        used += frame.len;
    }
    return used;
}

void RadioPing::deliver_image_audio(const uint8_t *blob, size_t len)
{
    av_audio_deliver(blob, len, image_rx_audio_seq_, image_rx_last_rssi_);
}

void RadioPing::play_stored_audio(const uint8_t *blob, size_t len, uint16_t first_seq)
{
    if (!is_gateway_ || intercom_active_) return;
    av_audio_deliver(blob, len, first_seq, 0);
}

void RadioPing::flush_playback_audio()
{
    if (is_gateway_ && voice_queue_ != nullptr && !intercom_active_) {
        xQueueReset(voice_queue_);
    }
}

void RadioPing::av_audio_deliver(const uint8_t *blob, size_t len,
                                 uint16_t first_seq, int16_t rssi)
{
    if (voice_queue_ == nullptr || blob == nullptr || len == 0) return;
    // Visitor session: the ring owns the speaker; ring-time audio goes to the
    // visitor record only.
    if (doorbell_stream_armed_.load(std::memory_order_acquire)) return;

    size_t offset = 0;
    uint16_t index = 0;
    while (offset < len) {
        const uint8_t opus_len = blob[offset++];
        if (opus_len == 0 || opus_len > APP_OPUS_MAX_PACKET_BYTES ||
            offset + opus_len > len) {
            // The payload passed its CRC32, so this is a framing disagreement,
            // not a bit flip. Drop the rest.
            ESP_LOGW(TAG, "A/V audio framing bad at %u/%u frame_len=%u",
                     static_cast<unsigned>(offset), static_cast<unsigned>(len),
                     opus_len);
            return;
        }

        VoicePacket packet = {
            .seq = static_cast<uint16_t>(first_seq + index),
            .len = opus_len,
            .rssi = rssi,
            .payload = {},
            .av_stream = true,
        };
        std::memcpy(packet.payload, &blob[offset], opus_len);
        offset += opus_len;
        index++;

        if (xQueueSend(voice_queue_, &packet, 0) != pdTRUE) {
            VoicePacket dropped;
            (void)xQueueReceive(voice_queue_, &dropped, 0);
            rx_queue_drops_++;
            if (xQueueSend(voice_queue_, &packet, 0) != pdTRUE) {
                ESP_LOGW(TAG, "A/V audio queue full drops=%lu seq=%u",
                         static_cast<unsigned long>(rx_queue_drops_), packet.seq);
            }
        }
    }
}

void RadioPing::av_playback_process(int16_t *pcm, size_t samples, bool synthetic)
{
    bool mute = false;
#if APP_AV_HOWL_ENABLE
    if (synthetic) {
        mute = av_howl_.muted();
    } else {
        const bool was_muted = av_howl_.muted();
        mute = av_howl_.process(pcm, samples);
        const HowlSuppressor::Features &f = av_howl_.last();

        if (mute && !was_muted) {
            ESP_LOGW(TAG, "A/V howl: muting playback clip=%lu%% rms=%lu events=%lu",
                     static_cast<unsigned long>(f.clip_percent),
                     static_cast<unsigned long>(f.rms),
                     static_cast<unsigned long>(av_howl_.mute_events()));
        } else if (!mute && was_muted) {
            ESP_LOGI(TAG, "A/V howl: playback restored after %lu frames%s",
                     static_cast<unsigned long>(av_howl_.muted_frames()),
                     av_howl_.hit_cap() ? " (cap)" : "");
        }

        // Window maxima, not a per-second sample: a single 10 ms snapshot
        // would miss every transient the thresholds are supposed to catch.
        if (f.clip_percent > av_play_clip_max_) av_play_clip_max_ = f.clip_percent;
        if (f.rms > av_play_rms_max_) av_play_rms_max_ = f.rms;
        av_play_frames_++;
#if APP_AV_HOWL_LOG_EVERY_FRAMES > 0
        if ((av_play_frames_ % APP_AV_HOWL_LOG_EVERY_FRAMES) == 0U) {
            ESP_LOGI(TAG, "A/V play: frames=%lu clip_max=%lu%% rms_max=%lu muted=%d events=%lu",
                     static_cast<unsigned long>(av_play_frames_),
                     static_cast<unsigned long>(av_play_clip_max_),
                     static_cast<unsigned long>(av_play_rms_max_),
                     mute ? 1 : 0,
                     static_cast<unsigned long>(av_howl_.mute_events()));
            av_play_clip_max_ = 0;
            av_play_rms_max_ = 0;
        }
#endif
    }
#else
    (void)synthetic;
#endif

    apply_av_playback_gain(pcm, samples);
    if (mute) {
        std::memset(pcm, 0, samples * sizeof(int16_t));
    }
}

void RadioPing::log_rx(uint16_t seq, uint16_t len, int16_t rssi)
{
    if (!have_expected_rx_seq_) {
        expected_rx_seq_ = static_cast<uint16_t>(seq + 1);
        have_expected_rx_seq_ = true;
    } else if (seq != expected_rx_seq_) {
        uint16_t gap = static_cast<uint16_t>(seq - expected_rx_seq_);
        if (gap < 0x8000) {
            rx_lost_ += gap;
            ESP_LOGW(TAG, "FLRC loss gap=%u expected=%u got=%u total_lost=%lu rssi=%d dBm",
                     gap, expected_rx_seq_, seq, static_cast<unsigned long>(rx_lost_), rssi);
        }
        expected_rx_seq_ = static_cast<uint16_t>(seq + 1);
    } else {
        expected_rx_seq_ = static_cast<uint16_t>(expected_rx_seq_ + 1);
    }

    rx_packets_++;
}

void RadioPing::wait_for_jitter_buffer()
{
    if (playback_active_ || voice_queue_ == nullptr || APP_RX_JITTER_FRAMES <= 1U) {
        return;
    }
    // A call that has been playing kept its ring topped up through the gap,
    // so the cushion is already there; blocking here would let it run dry.
    if (intercom_active_ && playout_primed_) {
        return;
    }

    const UBaseType_t target_waiting = static_cast<UBaseType_t>(APP_RX_JITTER_FRAMES - 1U);
    const uint32_t start_ms = smtc_modem_hal_get_time_in_ms();
    while (uxQueueMessagesWaiting(voice_queue_) < target_waiting) {
        if (smtc_modem_hal_get_time_in_ms() - start_ms >= APP_RX_JITTER_BUFFER_MS) {
            break;
        }
        vTaskDelay(ms_to_ticks_min_1(1));
    }
}

void RadioPing::conceal_missing_frames(uint16_t seq, bool av_stream)
{
    if (!have_expected_play_seq_) {
        expected_play_seq_ = seq;
        have_expected_play_seq_ = true;
    }

    uint16_t gap = static_cast<uint16_t>(seq - expected_play_seq_);
    if (gap > 0 && gap <= APP_RX_MAX_PLC_FRAMES) {
        for (uint16_t i = 0; i < gap; i++) {
            codec_lock();
            int decoded = codec_.decode_lost(rx_pcm_, APP_AUDIO_FRAME_SAMPLES);
            codec_unlock();
            if (decoded <= 0) {
                ESP_LOGW(TAG, "Opus PLC failed: %d", decoded);
                break;
            }
            if (intercom_active_) {
                apply_intercom_playback_gain(rx_pcm_, static_cast<size_t>(decoded));
            } else if (av_stream) {
                // Same level as its neighbours, and silent if they are: a PLC
                // frame at 2x level, or leaking through a mute, is a click.
                av_playback_process(rx_pcm_, static_cast<size_t>(decoded), true);
            }
            const bool written = play_mono_frame(rx_pcm_, static_cast<size_t>(decoded));
#if APP_INTERCOM_AEC_ENABLE
            // PLC audio is written to I2S too, so it must enter the AEC
            // reference timeline as well, once it is actually in the ring.
            if (written && intercom_active_) {
                echo_canceller_.push_reference(rx_pcm_,
                                               static_cast<size_t>(decoded));
            }
#else
            (void)written;
#endif
            last_rx_audio_ms_ = smtc_modem_hal_get_time_in_ms();
            playback_active_ = true;
        }
    }

    expected_play_seq_ = static_cast<uint16_t>(seq + 1);
}

bool RadioPing::read_mono_frame(int16_t *mono, size_t samples)
{
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    size_t got_total = 0;
    const size_t target = samples * 2 * sizeof(int16_t);

    while (got_total < target) {
        size_t got = 0;
        esp_err_t err = bsp_audio_read(reinterpret_cast<uint8_t *>(stereo) + got_total,
                                       target - got_total, &got);
        if (err != ESP_OK || got == 0) {
            // Silent on purpose: a depth change or camera release fails a read
            // or two every time. note_mic_read_failure() reports a run.
            return false;
        }
        got_total += got;
    }

    for (size_t i = 0; i < samples; i++) {
        mono[i] = stereo[2 * i];
    }
    return true;
}

void RadioPing::note_mic_read_failure()
{
    // 1 s of unbroken failures. A depth change or a camera release costs a
    // few reads; the I2S staying down (a rebuild that failed, see
    // bsp_audio_set_dma_desc_num) or an RX DMA that stopped delivering costs
    // all of them, and on the door station nothing else would ever bring it
    // back: the stream carries video and no sound until the next reboot.
    const int64_t now_us = esp_timer_get_time();
    if (mic_read_fail_since_us_ == 0) {
        mic_read_fail_since_us_ = now_us;
        return;
    }
    if (now_us - mic_read_fail_since_us_ < 1000000LL) return;
    mic_read_fail_since_us_ = 0;

    // Suspended means the camera holds the I2S's DMA memory on purpose; its
    // owner brings the I2S back.
    if (suspended_) return;

    mic_rebuild_attempts_++;
    const esp_err_t err = bsp_audio_rebuild();
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "microphone reads failed for 1 s, I2S rebuilt (attempt %lu)",
                 static_cast<unsigned long>(mic_rebuild_attempts_));
        mic_rebuild_attempts_ = 0;
    } else if (mic_rebuild_attempts_ == 1U || (mic_rebuild_attempts_ % 30U) == 0U) {
        ESP_LOGE(TAG, "microphone reads failing, I2S rebuild failed: %s (attempt %lu)",
                 esp_err_to_name(err),
                 static_cast<unsigned long>(mic_rebuild_attempts_));
    }
}

// True only when the whole frame reached the DMA ring; a short write means the
// I2S channel was disabled underneath us, not a full ring.
bool RadioPing::play_mono_frame(const int16_t *mono, size_t samples, bool pa_on)
{
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    if (samples > APP_AUDIO_FRAME_SAMPLES) {
        samples = APP_AUDIO_FRAME_SAMPLES;
    }

    for (size_t i = 0; i < samples; i++) {
        stereo[2 * i] = mono[i];
        stereo[2 * i + 1] = mono[i];
    }

    if (pa_on) {
        set_playback_pa(true);
    }
    const size_t bytes = samples * 2 * sizeof(int16_t);
    size_t written = 0;
    esp_err_t err = bsp_audio_write(stereo, bytes, &written);
    if (err != ESP_OK || written != bytes) {
        ESP_LOGW(TAG, "audio write failed: %s written=%u/%u",
                 esp_err_to_name(err), static_cast<unsigned>(written),
                 static_cast<unsigned>(bytes));
        // Whatever the ring holds now is unknown; stop keeping it.
        playout_primed_ = false;
        return false;
    }
    note_playout(samples);
    return true;
}

void RadioPing::note_playout(size_t samples)
{
    const int64_t now = esp_timer_get_time();
    const int64_t start = playout_end_us_ > now ? playout_end_us_ : now;
    playout_end_us_ = start + static_cast<int64_t>(samples) * 1000000 /
                                  static_cast<int64_t>(APP_AUDIO_SAMPLE_RATE_HZ);
    // The ring cannot hold more than its depth, whatever the sum says.
    const int64_t full_us = now + static_cast<int64_t>(bsp_audio_tx_ring_ms()) * 1000;
    if (playout_end_us_ > full_us) {
        playout_end_us_ = full_us;
    }
    if (intercom_active_ && !suspended_) {
        playout_primed_ = true;
    }
}

TickType_t RadioPing::intercom_fill_wait()
{
    if (!intercom_active_ || suspended_) {
        playout_primed_ = false;
    }
    if (!playout_primed_) return portMAX_DELAY;
    const int64_t slack_us = playout_end_us_ - esp_timer_get_time() -
        static_cast<int64_t>(APP_INTERCOM_PLAYOUT_GUARD_MS) * 1000;
    if (slack_us <= 0) return 0;
    // Round up: waking early would only come back round to wait again.
    return ms_to_ticks_min_1(static_cast<uint32_t>((slack_us + 999) / 1000));
}

bool RadioPing::intercom_fill_due() const
{
    return playout_primed_ && intercom_active_ && !suspended_ &&
           playout_end_us_ - esp_timer_get_time() <
               static_cast<int64_t>(APP_INTERCOM_PLAYOUT_GUARD_MS) * 1000;
}

// The frame that was due has not arrived and the ring is down to the guard.
// Write concealment for the first few (what conceal_missing_frames() would
// have written anyway, only in time), then silence, and give each filler to
// the AEC reference like any other frame: the ring and the reference then
// never lose step, whatever the radio does.
void RadioPing::intercom_fill_frame()
{
    size_t samples = APP_AUDIO_FRAME_SAMPLES;
    bool concealed = false;
    if (intercom_fill_run_ < APP_RX_MAX_PLC_FRAMES) {
        codec_lock();
        const int decoded = codec_.decode_lost(rx_pcm_, APP_AUDIO_FRAME_SAMPLES);
        codec_unlock();
        if (decoded > 0) {
            samples = static_cast<size_t>(decoded);
            apply_intercom_playback_gain(rx_pcm_, samples);
            concealed = true;
        }
    }
    if (!concealed) {
        std::memset(rx_pcm_, 0, sizeof(rx_pcm_));
    }
    intercom_fill_run_++;

    // The PA is already on (a call frame primed the keeper); a filler racing
    // hang-up must not switch it back on.
    const bool written = play_mono_frame(rx_pcm_, samples, false);
    if (!written) return;
#if APP_INTERCOM_AEC_ENABLE
    echo_canceller_.push_reference(rx_pcm_, samples);
#endif
    // The filler took the next frame's place. If that frame turns up late it
    // reads as behind the timeline, and conceal_missing_frames() plays it and
    // resyncs, one frame of extra cushion; if it was lost the gap is smaller.
    if (have_expected_play_seq_) {
        expected_play_seq_ = static_cast<uint16_t>(expected_play_seq_ + 1);
    }
}

void RadioPing::set_playback_pa(bool on)
{
    if (playback_pa_on_ == on) {
        return;
    }
    esp_err_t err = bsp_audio_pa_enable(on);
    if (err == ESP_OK) {
        playback_pa_on_ = on;
    } else {
        ESP_LOGW(TAG, "PA %s failed: %s", on ? "enable" : "disable", esp_err_to_name(err));
    }
}

void RadioPing::update_playback_timeout()
{
    if (!playback_active_) return;
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (now - last_rx_audio_ms_ > APP_RX_AUDIO_TIMEOUT_MS) {
        // A call rides through a dropout with the PA on: the play task keeps
        // the ring fed (intercom_fill_frame), while the PA's mute/unmute is
        // a thump on the speaker that the AEC never sees as reference. In the
        // soak logs both howls started within seconds of that toggle after a
        // 200 ms radio gap. The PA goes off at hang-up in stop_intercom_local().
        if (!intercom_active_) {
            set_playback_pa(false);
        }
        playback_active_ = false;
        have_expected_play_seq_ = false;
    }
}

// --- Image transfer implementation ---

void RadioPing::image_tx_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->image_tx_task();
}

void RadioPing::trigger_image_capture()
{
    if (intercom_active_) {
        ESP_LOGW(TAG, "image capture blocked: intercom active=%d started=%d session=%u",
                 intercom_active_ ? 1 : 0, intercom_start_confirmed_ ? 1 : 0,
                 intercom_session_);
        return;
    }
    // UI and esp_timer callbacks never touch the radio directly: collapse into
    // one pending request and wake the radio task.
    s_image_stream_active.store(true, std::memory_order_release);
    image_capture_req_.store(true, std::memory_order_release);
    ESP_LOGI(TAG,
             "image capture queued: suspended=%d mode=%u req_active=%d rx_pending=%d "
             "abort_pending=%d",
             suspended_ ? 1 : 0, static_cast<unsigned>(mode_),
             image_req_active_ ? 1 : 0, image_rx_pending_ ? 1 : 0,
             image_rx_abort_req_.load(std::memory_order_acquire) ? 1 : 0);
    TaskHandle_t task = task_handle_;
    if (task != nullptr) {
        xTaskNotifyGive(task);
    }
}

void RadioPing::abort_image_rx()
{
    // Leaving the page cancels a queued next-frame request as well as the
    // current RX. The radio task performs the actual teardown.
    s_image_stream_active.store(false, std::memory_order_release);
    image_capture_req_.store(false, std::memory_order_release);
    image_rx_abort_req_.store(true, std::memory_order_release);
    // Drop audio still queued from the stream so the speaker falls silent
    // with the picture.
    if (is_gateway_ && voice_queue_ != nullptr) {
        xQueueReset(voice_queue_);
    }
    // A visitor session ended by navigation or an answer: tell the app so it
    // silences the ring.
    if (doorbell_stream_armed_.exchange(false, std::memory_order_acq_rel) &&
        doorbell_stream_end_cb_ != nullptr) {
        doorbell_stream_end_cb_(false);
    }
    TaskHandle_t task = task_handle_;
    if (task != nullptr) {
        xTaskNotifyGive(task);
    }
}

void RadioPing::quiesce_image_stream(uint32_t timeout_ms)
{
    if (!is_gateway_) return;
    // No next frame: the final ACK carries no next session, so the node stops
    // on its own.
    s_image_stream_active.store(false, std::memory_order_release);
    image_capture_req_.store(false, std::memory_order_release);
    // The visitor session is over the moment the user answers: silence the
    // bell now rather than after the last frame has trickled in.
    if (doorbell_stream_armed_.exchange(false, std::memory_order_acq_rel) &&
        doorbell_stream_end_cb_ != nullptr) {
        doorbell_stream_end_cb_(false);
    }
    // The radio task stops the ImageCmd round and decides whether anything is
    // still worth waiting for (check_image_rx_quiesce).
    image_rx_quiesce_req_.store(true, std::memory_order_release);
    TaskHandle_t task = task_handle_;
    if (task != nullptr) {
        xTaskNotifyGive(task);
    }

    const uint32_t start = smtc_modem_hal_get_time_in_ms();
    while (smtc_modem_hal_get_time_in_ms() - start < timeout_ms) {
        if (!image_rx_quiesce_req_.load(std::memory_order_acquire) &&
            !image_rx_pending_) {
            ESP_LOGI(TAG, "image stream quiesced in %lums",
                     static_cast<unsigned long>(smtc_modem_hal_get_time_in_ms() - start));
            return;
        }
        vTaskDelay(ms_to_ticks_min_1(5));
    }
    ESP_LOGW(TAG, "image stream quiesce timed out (%lums): aborting the transfer",
             static_cast<unsigned long>(timeout_ms));
    abort_image_rx();
}

void RadioPing::doorbell_stream_request(uint32_t duration_ms)
{
    doorbell_stream_duration_ms_ = duration_ms;
    doorbell_stream_start_req_.store(true, std::memory_order_release);
    TaskHandle_t task = task_handle_;
    if (task != nullptr) {
        xTaskNotifyGive(task);
    }
}

void RadioPing::service_doorbell_stream()
{
    if (doorbell_stream_start_req_.exchange(false, std::memory_order_acq_rel)) {
        // Arm before starting: the deadline is the session's clock even when
        // the picture could not be requested.
        doorbell_stream_deadline_ms_ =
            smtc_modem_hal_get_time_in_ms() + doorbell_stream_duration_ms_;
        doorbell_stream_armed_.store(true, std::memory_order_release);
        const bool started =
            doorbell_stream_start_cb_ != nullptr && doorbell_stream_start_cb_();
        if (started) {
            ESP_LOGI(TAG, "doorbell session: stream started, %lums",
                     static_cast<unsigned long>(doorbell_stream_duration_ms_));
        } else {
            ESP_LOGW(TAG, "doorbell session: stream could not be started, ring only");
        }
        return;
    }

    if (!doorbell_stream_armed_.load(std::memory_order_acquire)) return;
    if ((int32_t)(smtc_modem_hal_get_time_in_ms() - doorbell_stream_deadline_ms_) < 0) {
        return;
    }
    // Clear first: the callback stops the stream through the UI, which comes
    // back into abort_image_rx(), and that must not report a second ending.
    if (!doorbell_stream_armed_.exchange(false, std::memory_order_acq_rel)) return;
    ESP_LOGI(TAG, "doorbell session: time is up, not answered");
    if (doorbell_stream_end_cb_ != nullptr) doorbell_stream_end_cb_(true);
}

void RadioPing::check_image_capture_request()
{
    if (!image_capture_req_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    start_image_capture_request();
}

void RadioPing::start_image_capture_request()
{
    if (image_tx_active_) {
        ESP_LOGW(TAG, "image TX already active, ignoring trigger");
        return;
    }

    // Do not replace an active session; its retry and RX timeout paths own
    // recovery if the peer stops responding.
    if (image_rx_pending_ || image_req_active_) {
        uint16_t active_session = image_xfer_.rx_active()
                                      ? image_xfer_.rx_session_id()
                                      : image_req_session_;
        ESP_LOGI(TAG, "capture trigger ignored: request/RX active (session=%u received=%u/%u)",
                 active_session, image_xfer_.rx_received_count(),
                 image_xfer_.rx_total_count());
        return;
    }
    // A late EOT from the previous session must not complete against the new
    // request's pending state.
    image_xfer_.rx_reset();
    image_rx_expected_crc32_ = 0;
    image_rx_request_ms_ = 0;
    image_start_wait_ms_ = 0;

    // Pick the session ONCE for this whole request. Retries reuse it (they do
    // NOT ++), so a resend can never spawn a second capture / a different JPEG.
    image_req_session_ = image_session_id_++;
    if (image_session_id_ == 0) {
        image_session_id_ = 1;
    }
    // Low-power rounds begin with LoRa wake-up; normal rounds start in FLRC.
    image_req_active_ = true;
    image_req_debug_last_ms_ = smtc_modem_hal_get_time_in_ms();
    ESP_LOGI(TAG, "image request start: session=%u low_power=%d mode=%u",
             image_req_session_, g_low_power_enabled ? 1 : 0,
             static_cast<unsigned>(mode_));
    start_image_req_round();

    image_rx_pending_ = true;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    schedule_rx();
}

// Start a request round, including LoRa wake-up when the node uses CAD standby.
void RadioPing::start_image_req_round()
{
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (g_low_power_enabled) {
        if (!send_lora_wakeup()) {
            ESP_LOGE(TAG, "LoRa wakeup failed (round retry)");
            // Back off; push round_end out too so the round-timeout check does
            // not re-fire immediately.
            uint32_t backoff = now + APP_IMAGE_REQ_RETRY_INTERVAL_LP_MS;
            image_req_round_end_ms_ = backoff;
            image_req_next_ms_ = backoff;
            return;
        }
        configure_flrc();
        image_req_round_end_ms_ = now + APP_IMAGE_REQ_ROUND_MS;
    }
    send_image_cmd_once();
    if (mode_ != Mode::rx_pending) {
        schedule_rx();
    }
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
}

// Send one ImageCmd for image_req_session_ (build + TX only). The LoRa wakeup /
// FLRC reconfig is handled once per round by start_image_req_round.
void RadioPing::send_image_cmd_once()
{
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageCmd;
    pkt[5] = 1;
    put_u16_le(&pkt[6], image_req_session_);
    put_u32_le(&pkt[8], smtc_modem_hal_get_time_in_ms());
    pkt[12] = 0;
    pkt[13] = 0;

    image_cmd_sent_ms_ = smtc_modem_hal_get_time_in_ms();

    // Stop RX before TX
    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    send_single_packet(pkt, kHeaderSize);
    // Keep the RX watchdog fresh so a long capture doesn't trip the 10s giveup.
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
}

// Radio task: resend ImageCmd once per interval until the node acks or an
// ImageStart clears image_req_active_.
void RadioPing::check_image_req_retry()
{
    // Accepted-but-never-started watchdog: the node acked but its capture died
    // before ImageStart. Re-open a full request round.
    if (image_start_wait_ms_ != 0 && !image_req_active_ && image_rx_pending_ &&
        !image_tx_active_ &&
        (smtc_modem_hal_get_time_in_ms() - image_start_wait_ms_) >=
            APP_IMAGE_START_WAIT_MS) {
        ESP_LOGW(TAG,
                 "no ImageStart %ums after the node's ack (session=%u): "
                 "re-requesting",
                 static_cast<unsigned>(APP_IMAGE_START_WAIT_MS), image_req_session_);
        image_start_wait_ms_ = 0;
        image_req_active_ = true;
        image_req_debug_last_ms_ = smtc_modem_hal_get_time_in_ms();
        start_image_req_round();
        return;
    }

    if (!image_req_active_) return;
    // A transfer already started (ImageStart / data) — stop requesting.
    if (image_tx_active_) {
        image_req_active_ = false;
        return;
    }
    uint32_t now = smtc_modem_hal_get_time_in_ms();

    if (now - image_req_debug_last_ms_ >= 1000U) {
        image_req_debug_last_ms_ = now;
        ESP_LOGI(TAG,
                 "image request waiting: session=%u mode=%u suspended=%d rx_pending=%d "
                 "received=%u/%u",
                 image_req_session_, static_cast<unsigned>(mode_), suspended_ ? 1 : 0,
                 image_rx_pending_ ? 1 : 0, image_xfer_.rx_received_count(),
                 image_xfer_.rx_total_count());
    }

    // Low power: a round that elapsed without ImageStart starts a fresh one
    // (new LoRa wakeup), uncapped. Non-low-power has no rounds.
    if (g_low_power_enabled && (int32_t)(now - image_req_round_end_ms_) >= 0) {
        ESP_LOGI(TAG, "ImageCmd round timed out, new wakeup round (session=%u)",
                 image_req_session_);
        start_image_req_round();
        return;
    }

    if ((int32_t)(now - image_req_next_ms_) < 0) return;

    // 30 ms cadence, non-harmonic with the node's 50 ms ImageStart retry.
    send_image_cmd_once();
    if (mode_ != Mode::rx_pending) {
        schedule_rx();
    }
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
}

void RadioPing::stop_image_req_retry()
{
    image_req_active_ = false;
}

// Takes ownership of `jpeg` (a heap_caps allocation): image_tx_task frees it
// when the transfer finishes or aborts. If the queue is full the request never
// reaches the task, so it is freed here.
void RadioPing::send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id)
{
    if (intercom_active_) {
        ESP_LOGW(TAG, "drop image while intercom is active");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
        return;
    }
    if (!image_tx_queue_) {
        ESP_LOGE(TAG, "image_tx_queue not initialized");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
        return;
    }
    ImageTxRequest req = { .jpeg = jpeg, .jpeg_len = jpeg_len, .session_id = session_id };
    if (xQueueSend(image_tx_queue_, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "image_tx_queue full");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
    }
}

/*
 * Image transfer protocol (T=transmitter/camera, R=receiver/controller):
 *
 * 1. T sends all ImageData fragments continuously, R does not reply
 * 2. T sends ImageEOT, R must reply; if T gets no reply, T resends EOT
 * 3. R replies with ImageACK containing list of missing fragment indices
 * 4. If missing list is empty → transfer complete
 * 5. If missing list has entries → T resends those fragments
 * 6. T sends missing fragments continuously, R does not reply
 * 7. T sends ImageEOT again, R must reply
 * 8. Repeat until complete or retry limit reached
 */
void RadioPing::image_tx_task()
{
    ImageTxRequest req;
    while (true) {
        if (xQueueReceive(image_tx_queue_, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Low power: the window closed during capture/encode; the gateway has
        // stopped listening, so drop the frame.
        if (g_low_power_enabled && !is_gateway_ && lp_window_expired_) {
            ESP_LOGW(TAG, "comm window expired before TX: dropping session=%u",
                     req.session_id);
            heap_caps_free(const_cast<uint8_t *>(req.jpeg));
            req.jpeg = nullptr;
            continue;
        }
        // A call started while this frame sat in the queue; a plain transfer
        // would fight its slots.
        if (intercom_active_) {
            ESP_LOGW(TAG, "call in progress: dropping queued image session=%u",
                     req.session_id);
            heap_caps_free(const_cast<uint8_t *>(req.jpeg));
            req.jpeg = nullptr;
            continue;
        }

        image_tx_active_ = true;
        s_image_tx_session.store(req.session_id, std::memory_order_release);
        image_done_received_ = false;
        image_nack_received_ = false;
        // Clear any preemption left over from the previous transfer: it was
        // raised for a session we are no longer sending.
        image_tx_preempt_req_ = false;
        suspended_ = true;

        // Leaving LoRa CAD for FLRC: clear the flag now or enter_low_power_cad()
        // skips the camera power-down after the push.
        low_power_cad_active_ = false;

        vTaskDelay(pdMS_TO_TICKS(APP_RADIO_TASK_POLL_MS * 2));

        smtc_modem_hal_protect_api_call();
        if (!configure_flrc()) {
            ESP_LOGE(TAG, "image TX: configure_flrc failed");
        }
        smtc_modem_hal_unprotect_api_call();

        // A/V stream: append the Opus frames captured since the previous image
        // so one payload carries [JPEG][audio] and the EOT/NACK retransmission
        // covers the audio for free. An unsolicited PIR push never armed the
        // microphone window and carries no audio.
        uint16_t audio_len = 0;
        uint16_t audio_seq = 0;
        if (av_audio_capture_due()) {
            const size_t got = av_audio_drain(av_audio_blob_, sizeof(av_audio_blob_),
                                              &audio_seq);
            if (got > 0) {
                // A fresh buffer, not a realloc: only the JPEG's heap_caps_free()
                // contract is guaranteed, not its capabilities.
                uint8_t *combined = static_cast<uint8_t *>(
                    heap_caps_malloc(req.jpeg_len + got,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
                if (combined != nullptr) {
                    std::memcpy(combined, req.jpeg, req.jpeg_len);
                    std::memcpy(combined + req.jpeg_len, av_audio_blob_, got);
                    heap_caps_free(const_cast<uint8_t *>(req.jpeg));
                    req.jpeg = combined;
                    req.jpeg_len += got;
                    audio_len = static_cast<uint16_t>(got);
                } else {
                    // Send the picture without sound rather than drop the frame.
                    ESP_LOGW(TAG, "A/V audio append failed: no PSRAM for %u+%u",
                             static_cast<unsigned>(req.jpeg_len),
                             static_cast<unsigned>(got));
                }
            }
        }

        uint16_t total_fragments = static_cast<uint16_t>(
            (req.jpeg_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
        uint32_t jpeg_crc32 = crc32_ieee(req.jpeg, req.jpeg_len);

        bool was_ptt = ptt_active_;
        ptt_active_ = false;
        tx_burst_active_ = false;
        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }

        // Step 0: send ImageStart until R confirms ready (100 x 50 ms). The
        // 50 ms interval is non-harmonic with the gateway's 30 ms ImageCmd
        // flood, so an ImageStart eventually lands in a gap and stops it.
        bool r_ready = false;
        for (uint16_t start_try = 0;
             start_try < APP_IMAGE_START_RETRY_COUNT && !r_ready &&
             !image_tx_preempt_req_ && !lp_window_expired_;
             start_try++) {
            // ImageStart = 14-byte header + vbat u16 + audio_len u16 +
            // audio_seq u16 + software CRC32 over [0..19] (FLRC hardware CRC is
            // off). audio_len = trailing Opus bytes of the payload, audio_seq =
            // sequence of the first Opus frame; both zero in plain video mode.
            uint8_t start_pkt[kImageStartAvSize];
            std::memcpy(start_pkt, kMagic, sizeof(kMagic));
            start_pkt[4] = kPacketTypeImageStart;
            start_pkt[5] = kImageStartAvVersion;
            put_u16_le(&start_pkt[6], req.session_id);
            put_u16_le(&start_pkt[8], total_fragments);
            put_u32_le(&start_pkt[10], jpeg_crc32);
            put_u16_le(&start_pkt[14], bsp_vbat_get_cached());  // battery voltage mV
            put_u16_le(&start_pkt[16], audio_len);
            put_u16_le(&start_pkt[18], audio_seq);
            put_u32_le(&start_pkt[20], crc32_ieee(start_pkt, 20));
            send_single_packet(start_pkt, sizeof(start_pkt));

            image_nack_received_ = false;
            image_done_received_ = false;
            schedule_rx();

            uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
            while (!image_nack_received_ && !image_done_received_) {
                if (smtc_modem_hal_get_time_in_ms() - wait_start > APP_IMAGE_START_RETRY_INTERVAL_MS) {
                    break;
                }
                if (irq_pending_) {
                    irq_pending_ = false;
                    ral_irq_t irq = RAL_IRQ_NONE;
                    smtc_modem_hal_protect_api_call();
                    ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                    smtc_modem_hal_unprotect_api_call();
                    if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                        handle_irq(irq);
                    }
                }
                taskYIELD();
            }

            if (image_nack_received_ || image_done_received_) {
                r_ready = true;
                // ESP_LOGI(TAG, "image TX: R ready, starting data burst");
            } else {
            // ESP_LOGW(TAG, "image TX: ImageStart no response, retry %u/%u",
            //          start_try + 1, APP_IMAGE_START_RETRY_COUNT);
            }
        }

        if (!r_ready) {
            const bool preempted = image_tx_preempt_req_;
            if (preempted) {
                ESP_LOGW(TAG,
                         "image TX preempted during handshake: dropping session=%u, "
                         "gateway wants a newer one",
                         req.session_id);
            } else if (lp_window_expired_) {
                ESP_LOGW(TAG,
                         "comm window expired during handshake: dropping session=%u",
                         req.session_id);
            }
            image_tx_active_ = false;
            s_image_tx_session.store(0, std::memory_order_release);
            image_tx_preempt_req_ = false;
            suspended_ = false;
            ptt_active_ = was_ptt;
            finish_image_tx_wake_state(preempted);
            if (!ptt_active_) schedule_rx();
            // We own req.jpeg (see send_image) — free before looping for the
            // next request, even on the abort path.
            heap_caps_free(const_cast<uint8_t *>(req.jpeg));
            req.jpeg = nullptr;
            continue;
        }

        // Step 1: Blast all fragments
        burst_send_fragments(req, total_fragments, nullptr, total_fragments);

        // ESP_LOGI(TAG, "image TX: initial burst done (%u frags)", total_fragments);

        // Step 2-8: EOT + wait ACK + retransmit loop
        bool transfer_done = false;
        // No-interaction abort: APP_LP_WAKE_WINDOW_MS without any ACK/NACK
        // means the link is dead. Runs alongside the round cap.
        uint32_t last_interaction_ms = smtc_modem_hal_get_time_in_ms();
        for (uint16_t round = 0; round < APP_IMAGE_NACK_MAX_RETRIES && !transfer_done; round++) {
            // The gateway moved on to another session: drop out now.
            if (image_tx_preempt_req_) {
                ESP_LOGW(TAG,
                         "image TX preempted: dropping session=%u after %u round(s)",
                         req.session_id, round);
                break;
            }
            // Low power hard deadline. The initial burst is short and left
            // uninterrupted; this retransmit/EOT loop is where the window bites.
            if (lp_window_expired_) {
                ESP_LOGW(TAG,
                         "comm window expired: dropping session=%u after %u round(s)",
                         req.session_id, round);
                break;
            }
            if (smtc_modem_hal_get_time_in_ms() - last_interaction_ms > APP_LP_WAKE_WINDOW_MS) {
                ESP_LOGW(TAG, "image TX: no ACK/NACK for %ums, aborting transfer",
                         static_cast<unsigned>(APP_LP_WAKE_WINDOW_MS));
                break;
            }
            // Give R time to process last packets before sending EOT
            vTaskDelay(pdMS_TO_TICKS(10));

            // Send EOT, retry if no response
            bool got_response = false;
            for (uint16_t eot_try = 0;
                 eot_try < APP_IMAGE_EOT_RETRY_COUNT && !image_tx_preempt_req_ &&
                 !lp_window_expired_;
                 eot_try++) {
                uint8_t eot[kHeaderSize];
                std::memcpy(eot, kMagic, sizeof(kMagic));
                eot[4] = kPacketTypeImageEOT;
                eot[5] = 1;
                put_u16_le(&eot[6], req.session_id);
                put_u16_le(&eot[8], total_fragments);
                eot[10] = 0; eot[11] = 0; eot[12] = 0; eot[13] = 0;
                send_single_packet(eot, kHeaderSize);

                // Wait for ACK
                image_nack_received_ = false;
                image_done_received_ = false;
                schedule_rx();

                uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
                while (!image_nack_received_ && !image_done_received_) {
                    if (smtc_modem_hal_get_time_in_ms() - wait_start > APP_IMAGE_EOT_RETRY_INTERVAL_MS) {
                        break;
                    }
                    if (irq_pending_) {
                        irq_pending_ = false;
                        ral_irq_t irq = RAL_IRQ_NONE;
                        smtc_modem_hal_protect_api_call();
                        ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                        smtc_modem_hal_unprotect_api_call();
                        if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                            handle_irq(irq);
                        }
                    }
                    taskYIELD();
                }

                if (image_nack_received_ || image_done_received_) {
                    got_response = true;
                    break;
                }
                // ESP_LOGW(TAG, "image TX: EOT no response, retry %u/%u",
                //          eot_try + 1, APP_IMAGE_EOT_RETRY_COUNT);
            }

            if (!got_response) {
                // No ACK this round: re-send EOT and wait again. Fragments are
                // only retransmitted on an explicit NACK missing-list.
                continue;
            }

            // Got a response — the link is alive, reset the no-interaction timer.
            last_interaction_ms = smtc_modem_hal_get_time_in_ms();

            if (image_done_received_ || nack_count_ == 0) {
                // ESP_LOGI(TAG, "image TX complete: all received");
                transfer_done = true;
                break;
            }

            // Retransmit missing fragments
            burst_send_fragments(req, total_fragments, nack_indices_, nack_count_);
        }

        const bool preempted = image_tx_preempt_req_;
        image_tx_active_ = false;
        s_image_tx_session.store(0, std::memory_order_release);
        image_tx_preempt_req_ = false;
        suspended_ = false;
        ptt_active_ = was_ptt;
        // ESP_LOGI(TAG, "image TX finished: session=%u done=%d",
        //          req.session_id, transfer_done ? 1 : 0);

        finish_image_tx_wake_state(preempted);

        if (mode_ != Mode::rx_pending && !ptt_active_) {
            schedule_rx();
        }

        // We own req.jpeg (see send_image).
        heap_caps_free(const_cast<uint8_t *>(req.jpeg));
        req.jpeg = nullptr;
    }
}

uint16_t RadioPing::build_image_fragment(uint8_t *pkt, const ImageTxRequest &req,
                                         uint16_t frag_index, uint16_t total_fragments)
{
    size_t offset = static_cast<size_t>(frag_index) * APP_IMAGE_FRAGMENT_DATA_SIZE;
    uint16_t frag_len = static_cast<uint16_t>(
        ((offset + APP_IMAGE_FRAGMENT_DATA_SIZE) <= req.jpeg_len)
            ? APP_IMAGE_FRAGMENT_DATA_SIZE
            : (req.jpeg_len - offset));

    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageData;
    pkt[5] = 1;
    put_u16_le(&pkt[6], req.session_id);
    put_u16_le(&pkt[8], frag_index);
    put_u16_le(&pkt[10], total_fragments);
    put_u16_le(&pkt[12], frag_len);
    std::memcpy(&pkt[kHeaderSize], req.jpeg + offset, frag_len);
    uint16_t crc = crc16_ccitt(&pkt[4], kHeaderSize - 4 + frag_len);
    put_u16_le(&pkt[kHeaderSize + frag_len], crc);

    return static_cast<uint16_t>(kHeaderSize + frag_len + 2);
}

void RadioPing::burst_send_fragments(const ImageTxRequest &req, uint16_t total_fragments,
                                     const uint16_t *indices, uint16_t count)
{
    if (count == 0) {
        return;
    }

    // Malformed NACK indices must not address beyond the JPEG buffer; compact
    // the valid ones into a dense sequence.
    uint16_t valid_indices[APP_IMAGE_NACK_MAX_INDICES];
    if (indices != nullptr) {
        uint16_t valid_count = 0;
        for (uint16_t i = 0; i < count; i++) {
            if (indices[i] < total_fragments) {
                valid_indices[valid_count++] = indices[i];
            }
        }
        if (valid_count == 0) {
            return;
        }
        indices = valid_indices;
        count = valid_count;
    }

    const void *ctx = radio_.ral.context;
    auto frag_at = [&](uint16_t pos) -> uint16_t {
        return indices ? indices[pos] : pos;
    };

    // Keep the target project's task-notification TX_DONE path: the image task
    // owns every IRQ for the complete burst while the main radio task is paused.
    tx_done_waiter_ = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(nullptr);

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    // The preceding short packet changed the shared FLRC length register.
    if (set_flrc_payload_length(&radio_.ral, APP_FLRC_BURST_PAYLOAD_LEN) != RAL_STATUS_OK) {
        smtc_modem_hal_unprotect_api_call();
        tx_done_waiter_ = nullptr;
        mode_ = Mode::idle;
        return;
    }
    (void)ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(ctx, LR20XX_RADIO_FALLBACK_FS);
    (void)lr20xx_radio_fifo_clear_tx(ctx);
    smtc_modem_hal_unprotect_api_call();

    uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
    uint16_t next_write = 0;
    auto build_padded = [&](uint16_t pos) {
        uint16_t len = build_image_fragment(pkt, req, frag_at(pos), total_fragments);
        if (len < APP_FLRC_BURST_PAYLOAD_LEN) {
            std::memset(pkt + len, 0, APP_FLRC_BURST_PAYLOAD_LEN - len);
        }
    };

    // A 1024-byte FIFO holds one packet in flight and one queued packet.
    for (int prefill = 0; prefill < 2 && next_write < count; prefill++) {
        build_padded(next_write);
        smtc_modem_hal_protect_api_call();
        (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_BURST_PAYLOAD_LEN);
        smtc_modem_hal_unprotect_api_call();
        next_write++;
    }

    mode_ = Mode::tx_pending;
    smtc_modem_hal_protect_api_call();
    (void)ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    for (uint16_t pos = 1; pos < count; pos++) {
        (void)wait_for_tx_done(50);

        mode_ = Mode::tx_pending;
        smtc_modem_hal_protect_api_call();
        (void)ral_set_tx(&radio_.ral);
        if (next_write < count) {
            build_padded(next_write);
            (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_BURST_PAYLOAD_LEN);
            next_write++;
        }
        smtc_modem_hal_unprotect_api_call();
    }

    (void)wait_for_tx_done(50);

    smtc_modem_hal_protect_api_call();
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(
        ctx, LR20XX_RADIO_FALLBACK_STDBY_XOSC);
    smtc_modem_hal_unprotect_api_call();
    mode_ = Mode::idle;
    tx_done_waiter_ = nullptr;
}

bool RadioPing::send_single_packet(const uint8_t *data, uint16_t len,
                                   uint32_t timeout_ms)
{
    if (data == nullptr || len < kHeaderSize) return false;
    // Control builders keep their existing body layout. Append integrity bytes
    // in the shared TX buffer; voice and image/battery/doorbell already have CRC.
    if (needs_crc32_trailer(data[4]) && data[4] != kPacketTypeVoice) {
        if (static_cast<size_t>(len) + 4U > sizeof(tx_buf_)) return false;
        std::memmove(tx_buf_, data, len);
        put_u32_le(tx_buf_ + len, crc32_ieee(tx_buf_, len));
        data = tx_buf_;
        len = static_cast<uint16_t>(len + 4U);
    }
    // Register as TX_DONE target and clear stale notifications before arming
    // TX, so a fast TX_DONE cannot be lost.
    tx_done_waiter_ = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(nullptr);

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    // FIFO writes do not update the on-air length. A short packet must not
    // inherit the 510-byte image/RX setting and transmit past its FIFO data.
    if (status == RAL_STATUS_OK) status = set_flrc_payload_length(&radio_.ral, len);
    if (status == RAL_STATUS_OK) status = static_cast<ral_status_t>(lr20xx_radio_fifo_clear_tx(radio_.ral.context));
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, data, len);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        tx_done_waiter_ = nullptr;
        return false;
    }

    // Second of the two FLRC TX sites (the other is schedule_tx).
    note_air_bytes(len);

    mode_ = Mode::tx_pending;
    bool ok = wait_for_tx_done(timeout_ms);
    tx_done_waiter_ = nullptr;
    return ok;
}

bool RadioPing::wait_for_tx_done(uint32_t timeout_ms)
{
    uint32_t start = smtc_modem_hal_get_time_in_ms();
    while (true) {
        if (irq_pending_) {
            irq_pending_ = false;
            ral_irq_t irq = RAL_IRQ_NONE;
            smtc_modem_hal_protect_api_call();
            ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
            smtc_modem_hal_unprotect_api_call();
            if (s == RAL_STATUS_OK && (irq & RAL_IRQ_TX_DONE)) {
                mode_ = Mode::idle;
                return true;
            }
        }
        if (smtc_modem_hal_get_time_in_ms() - start > timeout_ms) {
            mode_ = Mode::idle;
            return false;
        }
        // Block on the TX_DONE notification; the short poll timeout is a safety
        // net so a missed notification cannot dead-wait.
        ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
    }
}

void RadioPing::handle_image_cmd()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    // ESP_LOGI(TAG, "RX ImageCmd: session=%u", session_id);

    // A frame request proves a stream is running: arm the microphone before
    // any early-out below.
    av_audio_note_activity();

    // Low power: the window has expired; stay silent and let the gateway's
    // next round re-wake us with a fresh full-length window.
    if (g_low_power_enabled && !is_gateway_ && lp_window_expired_) {
        ESP_LOGW(TAG,
                 "ImageCmd session=%u past the comm window: ignoring until the "
                 "next wakeup",
                 session_id);
        return;
    }

    // A transfer for a different session is still running: it is dead weight
    // (the gateway only asks anew after giving up). Flag the preemption and stay
    // silent; the gateway's flood re-lands once the tx task releases the radio.
    bool tx_in_progress_for_this_session = false;
    if (image_tx_active_) {
        uint16_t tx_session = s_image_tx_session.load(std::memory_order_acquire);
        if (tx_session != 0 && session_id != tx_session) {
            if (!image_tx_preempt_req_) {
                ESP_LOGW(TAG,
                         "ImageCmd session=%u while sending session=%u: preempting "
                         "the active transfer",
                         session_id, tx_session);
            }
            image_tx_preempt_req_ = true;
            schedule_rx();
            return;
        }
        // Same session: our ack and ImageStart were lost. Re-ack, but do not
        // re-enter the capture path; its frame is on air right now.
        tx_in_progress_for_this_session = (tx_session != 0);
    }

    // Ask the app before acking (the callback only spawns the capture task and
    // does not touch the radio). Ack only on acceptance: a busy node stays
    // silent and the gateway's flood keeps going, so there is no deadlock. A
    // same-session retransmit is accepted again, so lost acks self-heal
    // without a second capture.
    bool accepted = true;
    if (image_capture_cb_ && !tx_in_progress_for_this_session) {
        accepted = image_capture_cb_(session_id);
    }

    if (session_id != image_cmd_debug_session_) {
        image_cmd_debug_session_ = session_id;
        ESP_LOGI(TAG,
                 "RX ImageCmd: session=%u accepted=%d intercom=%d image_tx=%d mode=%u",
                 session_id, accepted ? 1 : 0, intercom_active_ ? 1 : 0,
                 image_tx_active_ ? 1 : 0, static_cast<unsigned>(mode_));
    }

    if (accepted) {
        uint8_t ack[kHeaderSize];
        std::memcpy(ack, kMagic, sizeof(kMagic));
        ack[4] = kPacketTypeImageCmdAck;
        ack[5] = 1;
        put_u16_le(&ack[6], session_id);
        ack[8] = 0; ack[9] = 0; ack[10] = 0; ack[11] = 0; ack[12] = 0; ack[13] = 0;

        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }
        send_single_packet(ack, kHeaderSize);
    }
    schedule_rx();
}

// Gateway side: node acknowledged the ImageCmd. Stop the request-retry timer.
void RadioPing::handle_image_cmd_ack()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    if (session_id != image_req_session_) {
        return;
    }
    ESP_LOGI(TAG, "RX ImageCmdAck: session=%u request_age=%lums", session_id,
             static_cast<unsigned long>(smtc_modem_hal_get_time_in_ms() -
                                        image_cmd_sent_ms_));
    stop_image_req_retry();
    // Arm the ImageStart watchdog: if the accepted capture dies on the node,
    // check_image_req_retry re-opens a request round.
    image_start_wait_ms_ = smtc_modem_hal_get_time_in_ms();
    if (image_start_wait_ms_ == 0) image_start_wait_ms_ = 1;
    schedule_rx();
}

void RadioPing::handle_image_start(uint16_t len)
{
    if (len != 18U && len != 20U && len != kImageStartAvSize) {
        ESP_LOGW(TAG, "ImageStart invalid length=%u", len);
        return;
    }
    // Software CRC32 protects the header and metadata (hardware CRC is off);
    // on mismatch drop the packet and wait for a clean resend. Three lengths:
    // 18 bytes (CRC over [0..13]), 20 (vbat at [14..15], CRC over [0..15]),
    // 24 (audio_len [16..17], audio_seq [18..19], CRC over [0..19]).
    bool has_vbat = (len >= kHeaderSize + 6);
    bool has_audio = (len >= kHeaderSize + 10);
    uint16_t crc_len = has_audio ? 20 : (has_vbat ? 16 : kHeaderSize);
    uint16_t crc_offset = crc_len;

    if (len < crc_offset + 4) {
        ESP_LOGW(TAG, "ImageStart too short (len=%u), dropping", len);
        return;
    }
    uint32_t hdr_crc = crc32_ieee(rx_buf_, crc_len);
    uint32_t rx_hdr_crc = get_u32_le(&rx_buf_[crc_offset]);
    if (hdr_crc != rx_hdr_crc) {
        ESP_LOGW(TAG, "ImageStart header CRC32 mismatch (len=%u ver=%u calc=0x%08lx rx=0x%08lx), dropping",
                 len, rx_buf_[5], static_cast<unsigned long>(hdr_crc),
                 static_cast<unsigned long>(rx_hdr_crc));
        return;
    }

    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);
    uint32_t expected_crc32 = get_u32_le(&rx_buf_[10]);
    uint16_t vbat_mv = has_vbat ? get_u16_le(&rx_buf_[14]) : 0;
    uint16_t audio_len = has_audio ? get_u16_le(&rx_buf_[16]) : 0;
    uint16_t audio_seq = has_audio ? get_u16_le(&rx_buf_[18]) : 0;

    // Dialling: no ready-ACK, or the node would burst a frame on top of the
    // CONFIG exchange. The CONFIG reaches it between its ImageStart retries.
    if (intercom_dialing_) {
        ESP_LOGW(TAG, "ImageStart session=%u ignored: dialling the call", session_id);
        return;
    }

    const bool first_start_for_session =
        !image_xfer_.rx_active() || image_xfer_.rx_session_id() != session_id;
    if (first_start_for_session) {
        ESP_LOGI(TAG,
                 "RX ImageStart: session=%u total=%u req_session=%u req_active=%d "
                 "rx_pending=%d mode=%u",
                 session_id, total_frags, image_req_session_, image_req_active_ ? 1 : 0,
                 image_rx_pending_ ? 1 : 0, static_cast<unsigned>(mode_));
    }

    // Only the requested session (or a repeat of the active RX session) may
    // start a transfer, so a delayed ImageStart cannot hijack the first frame.
    if (image_rx_pending_) {
        uint16_t expected_session = image_req_active_
                                        ? image_req_session_
                                        : (image_xfer_.rx_active()
                                               ? image_xfer_.rx_session_id()
                                               : image_req_session_);
        if (expected_session != 0 && session_id != expected_session) {
            ESP_LOGW(TAG, "ImageStart ignored: session=%u expected=%u",
                     session_id, expected_session);
            return;
        }
    }
    bool gateway_requested = image_rx_pending_ && session_id == image_req_session_;
    // An unrequested push is the node's PIR snapshot: received like any frame
    // but kept off the screen and filed as a snapshot.
    image_rx_unsolicited_ = !gateway_requested;
    if (image_rx_unsolicited_ && first_start_for_session) {
        ESP_LOGI(TAG, "RX ImageStart: unsolicited push (PIR snapshot) session=%u",
                 session_id);
    }

    if (has_vbat) {
        // Start of a frame's RX — DEBUG so the per-frame transfer stays quiet;
        // the completion summary ("RX done") carries the useful outcome.
        ESP_LOGD(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx vbat=%u mV (%u.%02u V)",
                 session_id, total_frags, static_cast<unsigned long>(expected_crc32),
                 vbat_mv, vbat_mv / 1000, (vbat_mv % 1000) / 10);
        // vbat_received_cb_ is deliberately not called here: it takes the LVGL
        // lock blocking, and this is the RX hot path. The UI gets the voltage
        // from the periodic kPacketTypeVbat broadcast instead.
    } else {
        ESP_LOGD(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx",
                 session_id, total_frags, static_cast<unsigned long>(expected_crc32));
    }

    // An ImageStart proves the node got our request even if its ack was lost.
    stop_image_req_retry();
    image_start_wait_ms_ = 0;

    // A duplicate ImageStart after fragments have arrived must not re-run
    // rx_begin and wipe them. During the handshake (received == 0) re-begin is
    // harmless and the ready-ACK is re-sent.
    if (image_rx_pending_ &&
        session_id == image_xfer_.rx_session_id() &&
        image_xfer_.rx_received_count() > 0) {
        return;
    }

    image_rx_start_ms_ = smtc_modem_hal_get_time_in_ms();
    // Bind the timing origin to this RX session. Unsolicited node pushes have
    // no ImageCmd preparation phase, so their request origin is ImageStart.
    image_rx_request_ms_ = gateway_requested ? image_cmd_sent_ms_ : image_rx_start_ms_;

    // Prepare RX buffer
    image_xfer_.rx_begin(session_id, total_frags);
    image_rx_pending_ = true;
    image_rx_nack_sent_ = 0;
    image_rx_eot_count_ = 0;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_last_progress_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_expected_crc32_ = expected_crc32;
    // Applied at completion; both run on the radio task.
    image_rx_audio_len_ = audio_len;
    image_rx_audio_seq_ = audio_seq;

    // Send the ready ACK before any potentially blocking UI work, then re-arm
    // RX for the node's immediate data burst.
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageNack;
    pkt[5] = 3;
    put_u16_le(&pkt[6], session_id);
    put_u16_le(&pkt[8], 0);  // missing_count = 0 (ready signal)
    put_u16_le(&pkt[10], 0); // total_received = 0
    pkt[12] = 0; pkt[13] = 0;
    send_single_packet(pkt, kHeaderSize);

    // Enter RX for incoming data immediately after the ACK, before any UI work,
    // so we are listening before the node's first fragment can arrive.
    schedule_rx();

    // UI last, off the critical path; the callback takes the LVGL lock
    // non-blocking. A snapshot push gets no progress page.
    if (image_rx_progress_cb_ && !image_rx_unsolicited_) {
        image_rx_progress_cb_(0, total_frags, 0);
    }
}

void RadioPing::finalize_intercom_image_rx_session()
{
    if (!is_gateway_ || !intercom_img_rx_active_) return;
    if (!intercom_img_current_complete_) {
        const uint16_t received = image_xfer_.rx_active()
                                      ? image_xfer_.rx_received_count()
                                      : 0U;
        const uint16_t total = intercom_img_total_frags_;
        intercom_img_sessions_incomplete_++;
        if (received < total) {
            intercom_img_missing_unique_ += static_cast<uint32_t>(total - received);
        }
    }
}

void RadioPing::log_intercom_image_stats(bool final)
{
    if (!intercom_active_ && !final) return;
    if (!is_gateway_) {
        ESP_LOGI(TAG,
                 "[IMG TX] final=%u call=%u frame_session=%u adopted=%lu "
                 "sent_once=%lu frag_try=%lu tx_done=%lu tx_timeout=%lu "
                 "deadline_stop=%lu rx_rearm_min=%ldus rx_rearm_late=%lu "
                 "missed_slots=%lu",
                 final ? 1U : 0U, intercom_session_, intercom_img_session_,
                 static_cast<unsigned long>(intercom_img_frames_adopted_),
                 static_cast<unsigned long>(intercom_img_frames_tx_),
                 static_cast<unsigned long>(intercom_img_frag_tx_attempts_),
                 static_cast<unsigned long>(intercom_img_frag_tx_done_),
                 static_cast<unsigned long>(intercom_img_frag_tx_timeouts_),
                 static_cast<unsigned long>(intercom_probe_deadline_stops_),
                 static_cast<long>(intercom_img_rearm_slack_valid_
                                       ? intercom_img_rearm_min_slack_us_
                                       : 0),
                 static_cast<unsigned long>(intercom_img_rearm_late_),
                 static_cast<unsigned long>(intercom_missed_slots_));
        return;
    }

    uint16_t partial_received = 0;
    uint16_t partial_total = 0;
    if (intercom_img_rx_active_ && !intercom_img_current_complete_) {
        partial_total = intercom_img_total_frags_;
        if (image_xfer_.rx_active()) {
            partial_received = image_xfer_.rx_received_count();
        }
    }
    const uint32_t settled = intercom_img_frames_rx_ +
                             intercom_img_sessions_incomplete_ +
                             intercom_img_sessions_unseen_;
    const uint32_t frame_ok_x10 = settled > 0
        ? (intercom_img_frames_rx_ * 1000U) / settled
        : 0U;
    const uint32_t frag_total_est = intercom_img_frags_rx_ +
                                    intercom_img_frag_seq_lost_;
    const uint32_t frag_loss_x10 = frag_total_est > 0
        ? (intercom_img_frag_seq_lost_ * 1000U) / frag_total_est
        : 0U;
    const uint32_t elapsed_ms = smtc_modem_hal_get_time_in_ms() -
                                intercom_img_rate_ms_;
    const uint32_t fps_x10 = elapsed_ms > 0
        ? (intercom_img_frames_rx_ * 10000U) / elapsed_ms
        : 0U;

    ESP_LOGI(TAG,
             "[IMG RX] final=%u call=%u current=%u seen=%lu unseen=%lu complete=%lu "
             "incomplete=%lu partial=%u/%u frame_ok=%lu.%01lu%% fps=%lu.%01lu | "
             "frag_valid=%lu unique=%lu dup=%lu seq_lost_min=%lu "
             "loss_min=%lu.%01lu%% missing_unique=%lu crc=%lu malformed=%lu "
             "alloc_fail=%lu reasm_fail=%lu",
             final ? 1U : 0U, intercom_session_, intercom_img_rx_session_,
             static_cast<unsigned long>(intercom_img_sessions_seen_),
             static_cast<unsigned long>(intercom_img_sessions_unseen_),
             static_cast<unsigned long>(intercom_img_frames_rx_),
             static_cast<unsigned long>(intercom_img_sessions_incomplete_),
             static_cast<unsigned>(partial_received),
             static_cast<unsigned>(partial_total),
             static_cast<unsigned long>(frame_ok_x10 / 10U),
             static_cast<unsigned long>(frame_ok_x10 % 10U),
             static_cast<unsigned long>(fps_x10 / 10U),
             static_cast<unsigned long>(fps_x10 % 10U),
             static_cast<unsigned long>(intercom_img_frags_rx_),
             static_cast<unsigned long>(intercom_img_frags_unique_),
             static_cast<unsigned long>(intercom_img_frags_duplicate_),
             static_cast<unsigned long>(intercom_img_frag_seq_lost_),
             static_cast<unsigned long>(frag_loss_x10 / 10U),
             static_cast<unsigned long>(frag_loss_x10 % 10U),
             static_cast<unsigned long>(intercom_img_missing_unique_),
             static_cast<unsigned long>(intercom_img_crc_errors_),
             static_cast<unsigned long>(intercom_img_malformed_),
             static_cast<unsigned long>(intercom_img_alloc_failures_),
             static_cast<unsigned long>(intercom_img_reassemble_failures_));
}

void RadioPing::handle_intercom_image_data(uint16_t len)
{
    const uint16_t session_id = get_u16_le(&rx_buf_[6]);
    const uint16_t frag_index = get_u16_le(&rx_buf_[8]);
    const uint16_t total_frags = get_u16_le(&rx_buf_[10]);
    const uint16_t frag_len = get_u16_le(&rx_buf_[12]);

    if (total_frags == 0 || frag_index >= total_frags ||
        frag_len > APP_IMAGE_FRAGMENT_DATA_SIZE ||
        len < static_cast<uint16_t>(kHeaderSize + frag_len + 2)) {
        intercom_img_malformed_++;
        return;
    }
    // The stream parser verified this fragment's CRC16 before dispatch.

    const bool new_session = !intercom_img_rx_active_ ||
                             intercom_img_rx_session_ != session_id ||
                             intercom_img_total_frags_ != total_frags;
    if (new_session) {
        if (intercom_img_rx_active_) {
            const uint16_t session_gap = static_cast<uint16_t>(
                session_id - intercom_img_rx_session_);
            if (session_gap > 1U && session_gap < 0x8000U) {
                intercom_img_sessions_unseen_ +=
                    static_cast<uint32_t>(session_gap - 1U);
            }
            if (intercom_img_have_expected_frag_ && intercom_img_total_frags_ > 0) {
                intercom_img_frag_seq_lost_ +=
                    static_cast<uint32_t>((intercom_img_total_frags_ -
                                           intercom_img_expected_frag_) %
                                          intercom_img_total_frags_);
            }
            finalize_intercom_image_rx_session();
        } else if (session_id > 1U) {
            // Image sessions start at 1 for every call. A first observed value
            // above 1 means one or more whole JPEGs were lost before any fragment.
            intercom_img_sessions_unseen_ += static_cast<uint32_t>(session_id - 1U);
        }
        intercom_img_frag_seq_lost_ += frag_index;
        intercom_img_sessions_seen_++;
        intercom_img_rx_session_ = session_id;
        intercom_img_total_frags_ = total_frags;
        intercom_img_rx_active_ = true;
        intercom_img_current_complete_ = false;
        intercom_img_rx_frame_start_ms_ = smtc_modem_hal_get_time_in_ms();
        image_xfer_.rx_begin(session_id, total_frags);
    } else if (intercom_img_have_expected_frag_) {
        intercom_img_frag_seq_lost_ += static_cast<uint32_t>(
            (frag_index + total_frags - intercom_img_expected_frag_) % total_frags);
    }
    intercom_img_expected_frag_ = static_cast<uint16_t>((frag_index + 1U) % total_frags);
    intercom_img_have_expected_frag_ = true;
    intercom_img_frags_rx_++;

    if (!image_xfer_.rx_active()) {
        image_xfer_.rx_begin(session_id, total_frags);
        if (!image_xfer_.rx_active()) {
            intercom_img_alloc_failures_++;
            return;
        }
    }

    const uint16_t before = image_xfer_.rx_received_count();
    const bool complete = image_xfer_.rx_fragment(
        session_id, frag_index, total_frags, &rx_buf_[kHeaderSize], frag_len);
    const uint16_t after = image_xfer_.rx_received_count();
    if (after > before) {
        intercom_img_frags_unique_++;
    } else {
        intercom_img_frags_duplicate_++;
    }

    if (!complete) return;

#if APP_INTERCOM_IMAGE_REAL_CAPTURE_MS > 0
    if (intercom_img_current_complete_) return;
    intercom_img_current_complete_ = true;
    intercom_img_shown_session_ = session_id;
    intercom_img_shown_valid_ = true;
    intercom_img_frames_rx_++;
    const uint32_t complete_ms = smtc_modem_hal_get_time_in_ms();
    const uint32_t transfer_ms = complete_ms - intercom_img_rx_frame_start_ms_;
    // An in-call frame counts for the link account too, or the status strip
    // would freeze for the whole call. Published before the picture.
    link_stats_frame_done(transfer_ms);
    if (intercom_img_frame_cb_) {
        uint8_t *jpeg = nullptr;
        size_t jpeg_len = 0;
        const uint32_t reassemble_start_ms = smtc_modem_hal_get_time_in_ms();
        if (image_xfer_.rx_reassemble(&jpeg, &jpeg_len) == ESP_OK && jpeg) {
            const uint32_t reassemble_ms = smtc_modem_hal_get_time_in_ms() -
                                           reassemble_start_ms;
            intercom_img_frame_cb_(jpeg, jpeg_len, session_id, total_frags,
                                    transfer_ms, reassemble_ms,
                                    intercom_img_frame_cb_ctx_);
        } else {
            intercom_img_reassemble_failures_++;
        }
    }
#else
    intercom_img_frames_rx_++;
    image_xfer_.rx_restart();
#endif
}

void RadioPing::handle_image_data(uint16_t len)
{
    // In-call piggyback path has no handshake; route it to the frame-rate counter.
    if (intercom_active_ && is_gateway_) {
        handle_intercom_image_data(len);
        return;
    }

    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t frag_index = get_u16_le(&rx_buf_[8]);
    uint16_t total_frags = get_u16_le(&rx_buf_[10]);
    uint16_t frag_len = get_u16_le(&rx_buf_[12]);

    if (!image_rx_pending_ || session_id != image_xfer_.rx_session_id() ||
        total_frags != image_xfer_.rx_total_count()) {
        return;
    }

    if (frag_len > APP_IMAGE_FRAGMENT_DATA_SIZE) {
        ESP_LOGW(TAG, "RX ImageData: bad frag_len=%u", frag_len);
        return;
    }

    if (len < static_cast<uint16_t>(kHeaderSize + frag_len + 2)) {
        return;
    }

    // The stream parser verified this fragment's CRC16 before dispatch.

    image_xfer_.rx_fragment(session_id, frag_index, total_frags,
                            &rx_buf_[kHeaderSize], frag_len);
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_pending_ = true;
    if (frag_index == 0) {
        image_rx_nack_sent_ = 0;
    }
}

void RadioPing::handle_image_nack()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t active_session = s_image_tx_session.load(std::memory_order_acquire);
    if (active_session == 0 || session_id != active_session) {
        return;
    }

    uint16_t missing_count = get_u16_le(&rx_buf_[8]);

    if (missing_count > APP_IMAGE_NACK_MAX_INDICES) {
        missing_count = APP_IMAGE_NACK_MAX_INDICES;
    }

    nack_count_ = missing_count;
    for (uint16_t i = 0; i < missing_count; i++) {
        nack_indices_[i] = get_u16_le(&rx_buf_[kHeaderSize + i * 2]);
    }

    uint16_t total_received = get_u16_le(&rx_buf_[10]);
    // ESP_LOGI(TAG, "RX ImageACK: session=%u missing=%u received=%u",
    //          session_id, missing_count, total_received);
    image_nack_received_ = true;
    if (missing_count == 0) {
        uint16_t next_session = get_u16_le(&rx_buf_[12]);
        if (next_session != 0 && next_session != session_id) {
            av_audio_note_activity();
            s_chained_capture_session.store(next_session, std::memory_order_release);
            ESP_LOGD(TAG, "chained capture queued: current=%u next=%u",
                     session_id, next_session);
        }
        image_done_received_ = true;
    }
}

void RadioPing::handle_image_done()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    // ESP_LOGI(TAG, "RX ImageDone: session=%u", session_id);
    image_done_received_ = true;
}

void RadioPing::handle_image_eot()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);

    // ESP_LOGI(TAG, "RX ImageEOT: session=%u received=%u/%u",
    //          session_id, image_xfer_.rx_received_count(), total_frags);

    // The final ACK may be lost: re-ACK the completed session with the same
    // next-session id so the node leaves its EOT retry loop.
    if (session_id == image_rx_done_session_ &&
        (!image_rx_pending_ || !image_xfer_.rx_active() ||
         session_id != image_xfer_.rx_session_id())) {
        uint8_t pkt[kHeaderSize];
        std::memcpy(pkt, kMagic, sizeof(kMagic));
        pkt[4] = kPacketTypeImageNack;
        pkt[5] = 3;
        put_u16_le(&pkt[6], session_id);
        put_u16_le(&pkt[8], 0);
        put_u16_le(&pkt[10], 0);
        uint16_t repeated_next =
            s_image_stream_active.load(std::memory_order_acquire)
                ? s_image_rx_done_next_session
                : 0;
        put_u16_le(&pkt[12], repeated_next);
        send_single_packet(pkt, kHeaderSize);
        schedule_rx();
        return;
    }

    if (!image_rx_pending_) {
        // Already completed — still send ACK so T stops retrying
        if (session_id == image_rx_done_session_) {
            uint8_t pkt[kHeaderSize];
            std::memcpy(pkt, kMagic, sizeof(kMagic));
            pkt[4] = kPacketTypeImageNack;
            pkt[5] = 3;
            put_u16_le(&pkt[6], session_id);
            put_u16_le(&pkt[8], 0);
            put_u16_le(&pkt[10], 0);
            pkt[12] = 0; pkt[13] = 0;
            send_single_packet(pkt, kHeaderSize);
            schedule_rx();
        }
        return;
    }

    // A late EOT must never touch another session's reassembly state.
    if (!image_xfer_.rx_active() ||
        session_id != image_xfer_.rx_session_id() ||
        total_frags != image_xfer_.rx_total_count()) {
        ESP_LOGW(TAG, "ImageEOT ignored: session=%u/%u total=%u/%u",
                 session_id, image_xfer_.rx_session_id(),
                 total_frags, image_xfer_.rx_total_count());
        return;
    }

    // Exit continuous RX to send response
    if (mode_ == Mode::rx_pending) {
        smtc_modem_hal_protect_api_call();
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        smtc_modem_hal_unprotect_api_call();
        mode_ = Mode::idle;
    }

    // UI before the NACK: the node waits for our reply, so LVGL work here
    // costs no packets. Not for a snapshot push.
    if (image_rx_progress_cb_ && !image_rx_unsolicited_) {
        uint16_t total = image_xfer_.rx_total_count();
        image_rx_progress_cb_(image_xfer_.rx_received_count(), total, image_rx_last_rssi_);
    }

    // Build ACK with missing indices
    uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageNack;
    pkt[5] = 3;
    put_u16_le(&pkt[6], session_id);

    uint16_t missing_indices[APP_IMAGE_NACK_MAX_INDICES];
    uint16_t missing_count = image_xfer_.rx_get_missing(missing_indices, APP_IMAGE_NACK_MAX_INDICES);
    if (missing_count == 0 && image_rx_expected_crc32_ != 0) {
        uint32_t actual_crc32 = image_xfer_.rx_crc32();
        if (actual_crc32 != image_rx_expected_crc32_) {
            ESP_LOGW(TAG, "image RX crc32 mismatch: expected=0x%08lx actual=0x%08lx, requesting full resend",
                     static_cast<unsigned long>(image_rx_expected_crc32_),
                     static_cast<unsigned long>(actual_crc32));
            image_xfer_.rx_begin(session_id, total_frags);
            image_rx_pending_ = true;
            image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
            image_rx_last_progress_ms_ = 0;
            missing_count = image_xfer_.rx_get_missing(missing_indices, APP_IMAGE_NACK_MAX_INDICES);
        } else {
            // ESP_LOGI(TAG, "image RX crc32 ok: 0x%08lx",
            //          static_cast<unsigned long>(actual_crc32));
        }
    }

    uint16_t next_session = 0;
    if (missing_count == 0 && !image_rx_unsolicited_ &&
        s_image_stream_active.load(std::memory_order_acquire)) {
        next_session = image_session_id_++;
        if (image_session_id_ == 0) {
            image_session_id_ = 1;
        }
    }

    put_u16_le(&pkt[8], missing_count);
    put_u16_le(&pkt[10], image_xfer_.rx_received_count());
    put_u16_le(&pkt[12], next_session);

    for (uint16_t i = 0; i < missing_count; i++) {
        put_u16_le(&pkt[kHeaderSize + i * 2], missing_indices[i]);
    }

    uint16_t pkt_len = static_cast<uint16_t>(kHeaderSize + missing_count * 2);
    send_single_packet(pkt, pkt_len);
    schedule_rx();

    if (image_rx_eot_cb_ && !image_rx_unsolicited_) {
        bool is_first = (image_rx_eot_count_ == 0);
        image_rx_eot_count_++;
        image_rx_eot_cb_(missing_count, is_first);
    }

    if (missing_count == 0) {
        image_rx_pending_ = false;
        image_rx_done_session_ = session_id;
        s_image_rx_done_next_session = next_session;
        uint32_t now_ms = smtc_modem_hal_get_time_in_ms();
        uint32_t transfer_ms = now_ms - image_rx_start_ms_;
        int32_t prepare_delta =
            static_cast<int32_t>(image_rx_start_ms_ - image_rx_request_ms_);
        uint32_t prepare_ms =
            prepare_delta >= 0 ? static_cast<uint32_t>(prepare_delta) : 0U;
        uint32_t total_ms = prepare_ms + transfer_ms;
        image_rx_transfer_ms_ = transfer_ms;
        image_rx_done_ms_ = now_ms;
        ESP_LOGI(TAG, "RX done | prepare=%lums transfer=%lums total=%lums",
                 static_cast<unsigned long>(prepare_ms),
                 static_cast<unsigned long>(transfer_ms),
                 static_cast<unsigned long>(total_ms));
        // Only a completed frame counts; the timeout path does not call this.
        link_stats_frame_done(transfer_ms);
        if (image_rx_complete_cb_) {
            image_rx_complete_cb_(&image_xfer_);
        }
        // One-shot: a following plain-video frame must not have its JPEG tail
        // mistaken for Opus.
        image_rx_audio_len_ = 0;

        // Arm for the piggybacked session without an ImageCmd; the request
        // retry sends it if no ImageStart arrives.
        if (next_session != 0 &&
            s_image_stream_active.load(std::memory_order_acquire)) {
            image_req_session_ = next_session;
            image_req_active_ = true;
            image_req_next_ms_ = now_ms + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
            image_req_round_end_ms_ = now_ms + APP_IMAGE_REQ_ROUND_MS;
            image_cmd_sent_ms_ = now_ms;
            image_rx_request_ms_ = now_ms;
            image_rx_pending_ = true;
            image_rx_last_frag_ms_ = now_ms;
            ESP_LOGD(TAG, "stream chained: completed=%u next=%u",
                     session_id, next_session);
        }
    } else {
        // Once per retransmit round; the per-frame outcome is in "RX done".
        ESP_LOGI(TAG, "image RX: sent ACK with %u missing (first=%u), waiting for retransmit",
                 missing_count, missing_indices[0]);
        if (image_rx_eot_count_ == 1) {
            for (uint16_t i = 0; i < missing_count; i += 16) {
                char line[128];
                int pos = 0;
                for (uint16_t j = i; j < missing_count && j < i + 16; j++) {
                    pos += snprintf(line + pos, sizeof(line) - pos, "%u ", missing_indices[j]);
                }
                ESP_LOGD(TAG, "  missing: %s", line);
            }
        }
        image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    }
}

void RadioPing::check_image_rx_timeout()
{
    if (!image_rx_pending_) return;
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (image_xfer_.rx_complete() &&
        now - image_rx_last_frag_ms_ < APP_IMAGE_RX_TIMEOUT_MS) {
        return;
    }
    if (image_xfer_.rx_complete()) {
        if (image_rx_expected_crc32_ != 0) {
            uint32_t actual_crc32 = image_xfer_.rx_crc32();
            if (actual_crc32 != image_rx_expected_crc32_) {
                ESP_LOGW(TAG, "image RX complete timeout crc32 mismatch: expected=0x%08lx actual=0x%08lx",
                         static_cast<unsigned long>(image_rx_expected_crc32_),
                         static_cast<unsigned long>(actual_crc32));
                image_xfer_.rx_begin(image_xfer_.rx_session_id(), image_xfer_.rx_total_count());
                image_rx_last_frag_ms_ = now;
                image_rx_last_progress_ms_ = 0;
                return;
            }
        }
        image_rx_pending_ = false;
        image_rx_done_session_ = image_xfer_.rx_session_id();
        // ESP_LOGI(TAG, "image RX complete (no EOT seen before timeout)");
        if (image_rx_complete_cb_) {
            image_rx_complete_cb_(&image_xfer_);
        }
        image_rx_audio_len_ = 0;   // one-shot, same as the EOT path
        return;
    }

    if (now - image_rx_last_frag_ms_ < 10000U) {
        return;
    }

    ESP_LOGW(TAG,
             "image RX timeout: session=%u received=%u/%u req_active=%d mode=%u "
             "intercom=%d suspended=%d",
             image_xfer_.rx_session_id(), image_xfer_.rx_received_count(),
             image_xfer_.rx_total_count(), image_req_active_ ? 1 : 0,
             static_cast<unsigned>(mode_), intercom_active_ ? 1 : 0,
             suspended_ ? 1 : 0);
    image_rx_pending_ = false;
    image_start_wait_ms_ = 0;
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
}

// Radio task: tear down an image RX the UI asked to abort. Clears the RX state
// and the ImageCmd retry; the node's TX side self-aborts once ACKs stop.
void RadioPing::check_image_rx_abort()
{
    if (!image_rx_abort_req_.exchange(false, std::memory_order_acq_rel)) return;

    image_capture_req_.store(false, std::memory_order_release);
    ESP_LOGI(TAG,
             "image RX abort consumed: req_active=%d rx_pending=%d session=%u "
             "received=%u/%u mode=%u intercom=%d",
             image_req_active_ ? 1 : 0, image_rx_pending_ ? 1 : 0,
             image_xfer_.rx_session_id(), image_xfer_.rx_received_count(),
             image_xfer_.rx_total_count(), static_cast<unsigned>(mode_),
             intercom_active_ ? 1 : 0);
    image_req_active_ = false;
    image_rx_pending_ = false;
    image_start_wait_ms_ = 0;
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
    schedule_rx();
}

// Radio task, for quiesce_image_stream(): stop the ImageCmd round and keep only
// what the node has committed to (a transfer in flight or an acknowledged
// request). An unacknowledged request is dropped.
void RadioPing::check_image_rx_quiesce()
{
    if (!image_rx_quiesce_req_.exchange(false, std::memory_order_acq_rel)) return;

    const bool node_committed = image_xfer_.rx_active() || image_start_wait_ms_ != 0;
    ESP_LOGI(TAG,
             "image stream quiesce: req_active=%d rx_pending=%d session=%u "
             "received=%u/%u committed=%d",
             image_req_active_ ? 1 : 0, image_rx_pending_ ? 1 : 0,
             image_xfer_.rx_session_id(), image_xfer_.rx_received_count(),
             image_xfer_.rx_total_count(), node_committed ? 1 : 0);
    image_req_active_ = false;
    image_start_wait_ms_ = 0;
    if (!node_committed) {
        image_rx_pending_ = false;
        image_xfer_.rx_reset();
        image_rx_nack_sent_ = 0;
        schedule_rx();
    }
}

bool RadioPing::send_config(uint8_t key, uint32_t value)
{
    if (intercom_active_ && key != APP_CFG_KEY_INTERCOM) {
        ESP_LOGW(TAG, "config key=%u rejected while intercom is active", key);
        return false;
    }
    ESP_LOGI(TAG, "send_config: key=%u value=%lu", key, static_cast<unsigned long>(value));

    suspended_ = true;
    const uint32_t ack_timeout_ms =
        (key == APP_CFG_KEY_INTERCOM && value != 0)
            ? APP_INTERCOM_START_TIMEOUT_MS
            : kConfigAckTimeoutMs;

    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeConfig;
    pkt[5] = 1;
    put_u16_le(&pkt[6], 0);
    pkt[8] = key;
    put_u32_le(&pkt[9], value);
    pkt[13] = 0;

    // Wake the node with a LoRa preamble first. Always for the LOW_POWER key
    // itself, so the two sides cannot get stuck desynced.
    if (g_low_power_enabled || key == APP_CFG_KEY_LOW_POWER) {
        uint32_t t0 = smtc_modem_hal_get_time_in_ms();
        if (!send_lora_wakeup()) {
            ESP_LOGE(TAG, "LoRa wakeup failed for config");
            suspended_ = false;
            if (!ptt_active_) schedule_rx();
            return false;
        }
        uint32_t elapsed = smtc_modem_hal_get_time_in_ms() - t0;
        ESP_LOGI(TAG, "LoRa wakeup preamble TX took %lu ms (config)", (unsigned long)elapsed);
        configure_flrc();
    }

    for (int attempt = 0; attempt < 3; attempt++) {
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();

        send_single_packet(pkt, kHeaderSize);

        config_ack_received_ = false;
        schedule_rx();

        uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
        while (!config_ack_received_) {
            if (smtc_modem_hal_get_time_in_ms() - wait_start > ack_timeout_ms) {
                break;
            }
            if (irq_pending_) {
                irq_pending_ = false;
                ral_irq_t irq = RAL_IRQ_NONE;
                smtc_modem_hal_protect_api_call();
                ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                smtc_modem_hal_unprotect_api_call();
                if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                    handle_irq(irq);
                }
            }
            taskYIELD();
        }

        if (config_ack_received_) {
            ESP_LOGI(TAG, "send_config: ACK received on attempt %d", attempt + 1);
            suspended_ = false;
            if (!ptt_active_) schedule_rx();
            return true;
        }
        ESP_LOGW(TAG, "send_config: no ACK, attempt %d/3", attempt + 1);
    }

    ESP_LOGW(TAG, "send_config: failed after 3 attempts");
    suspended_ = false;
    if (!ptt_active_) schedule_rx();
    return false;
}

void RadioPing::send_config_ack(uint8_t key, uint32_t value)
{
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeConfigAck;
    pkt[5] = 1;
    put_u16_le(&pkt[6], 0);
    pkt[8] = key;
    put_u32_le(&pkt[9], value);
    pkt[13] = 0;

    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    send_single_packet(pkt, kHeaderSize);

    if (!ptt_active_) {
        schedule_rx();
    }
}

bool RadioPing::configure_lora_cad()
{
    const void *ctx = radio_.ral.context;

    lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
    lr20xx_radio_common_set_rf_freq(ctx, APP_FLRC_FREQUENCY_HZ);

    lr20xx_radio_lora_mod_params_t mod = {};
    mod.sf = LR20XX_RADIO_LORA_SF7;
    mod.bw = LR20XX_RADIO_LORA_BW_125;
    mod.cr = LR20XX_RADIO_LORA_CR_4_5;
    mod.ppm = LR20XX_RADIO_LORA_NO_PPM;
    if (lr20xx_radio_lora_set_modulation_params(ctx, &mod) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "lora mod params failed");
        return false;
    }

    lr20xx_radio_lora_cad_params_t cad = {};
    cad.cad_symb_nb = 2;
    cad.pnr_delta = 0;
    cad.cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_STANDBYRC;
    cad.cad_timeout_in_pll_step = 0;
    cad.cad_detect_peak = 56;
    if (lr20xx_radio_lora_configure_cad_params(ctx, &cad) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "lora cad params failed");
        return false;
    }

    return true;
}

bool RadioPing::low_power_sleep(uint32_t ms)
{
    // The USB Serial/JTAG console does not survive light sleep; flush first.
    fflush(stdout);

    esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);

    // ESP32-S3 light-sleep GPIO wake supports level mode only, so arm
    // GPIO_INTR_HIGH_LEVEL like the PIR ISR. Only while pir_armed_: during the
    // cooldown the still-high pin must not wake us. The ISR handles the trigger
    // on resume; this only supplies the wake source.
    bool pir_wake = pir_enabled_ && pir_armed_;
    if (pir_wake) {
        gpio_wakeup_enable(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
        esp_sleep_enable_gpio_wakeup();
    }

    // Doorbell key: the button task is frozen during sleep, so arm the KEY_ADC
    // line as a LOW_LEVEL wake. See bsp_button_arm_sleep_wakeup() for why this
    // must be confirmed by an ADC read after the wake.
    bool key_wake = false;
#if APP_DOORBELL_ENABLE
    key_wake = bsp_button_arm_sleep_wakeup() == ESP_OK;
#endif

    esp_light_sleep_start();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool woken_by_pir = false;

    if (key_wake) {
        // Hand the pad back before anything can call adc_oneshot_read().
        bsp_button_disarm_sleep_wakeup();
    }
    if (pir_wake) {
        gpio_wakeup_disable(APP_PIR_GPIO);
    }

    if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        // PIR and the key ladder share one wake cause; read the ADC to tell
        // which (a real press is still held).
        if (key_wake && bsp_button_sample_adc() == APP_DOORBELL_KEY) {
            ESP_LOGI(TAG, "light sleep: woken by doorbell key");
            doorbell_trigger();
        } else if (pir_wake) {
            // The GPIO ISR handles the trigger on resume; just report the wake.
            woken_by_pir = true;
            ESP_LOGI(TAG, "light sleep: woken by PIR");
        }
    }
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGD(TAG, "light sleep: timer wake");
    }

    return woken_by_pir;
}

void RadioPing::finish_image_tx_wake_state(bool preempted)
{
    if (!g_low_power_enabled || is_gateway_) return;

    // The PIR self-push guard covers exactly "capture + push"; that work is over
    // whichever way this transfer ended, so it always goes.
    pir_push_wake_ = false;

    if (lp_window_expired_) {
        // The hard deadline outranks a pending preemption: this wakeup is
        // spent, a fresh LoRa wakeup is needed.
        ESP_LOGI(TAG, "image TX ended past the comm window -> CAD sleep");
        return;
    }

    if (preempted) {
        // Preempted by a call: start_intercom_local() owns the window now.
        if (intercom_active_) {
            ESP_LOGI(TAG, "image TX preempted by a call, window is the call's");
            return;
        }
        // The gateway is flooding ImageCmd for a newer session: restart the
        // window so the node can accept it. It gets its own deadline, so a
        // preemption chain cannot extend the wakeup forever.
        cad_wakeup_ms_ = smtc_modem_hal_get_time_in_ms();
        arm_lp_window_timer();
        ESP_LOGI(TAG, "image TX preempted, holding FLRC for the new request");
        return;
    }

    // Otherwise leave the CAD-wakeup window alone; follow-up requests may
    // still arrive inside it. A PIR self-push never opened one.
    if (cad_wakeup_ms_ == 0) {
        ESP_LOGI(TAG, "image TX done, no wake window open -> CAD sleep");
    } else {
        uint32_t elapsed = smtc_modem_hal_get_time_in_ms() - cad_wakeup_ms_;
        uint32_t left = elapsed < APP_LP_COMM_WINDOW_MS
                            ? (APP_LP_COMM_WINDOW_MS - elapsed)
                            : 0U;
        ESP_LOGI(TAG, "image TX done, %lums left of the comm window",
                 static_cast<unsigned long>(left));
    }
}

void RadioPing::notify_capture_starting()
{
    // Doorbell guard between "capture dispatched" and "image_tx_task owns the
    // radio", where no other flag is set. A timestamp, not a flag: a capture
    // that dies before TX must not disable the doorbell for good.
    last_capture_dispatch_ms_ = smtc_modem_hal_get_time_in_ms();

    // Low-power node only: keep-awake guard for the capture + push.
    if (!g_low_power_enabled || is_gateway_) return;
    pir_push_wake_ = true;
    pir_push_wake_ms_ = smtc_modem_hal_get_time_in_ms();
}

void RadioPing::lp_window_timer_cb(void *arg)
{
    // esp_timer context: raise the flag only; owners unwind at their own
    // checkpoints.
    static_cast<RadioPing *>(arg)->lp_window_expired_ = true;
}

void RadioPing::arm_lp_window_timer()
{
    // Node-only while low power is on; otherwise the flag would never be
    // cleared.
    if (is_gateway_ || !g_low_power_enabled) return;

    if (lp_window_timer_ == nullptr) {
        esp_timer_create_args_t args = {};
        args.callback = &RadioPing::lp_window_timer_cb;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "lp_window";
        if (esp_timer_create(&args, &lp_window_timer_) != ESP_OK) {
            ESP_LOGE(TAG, "low power window timer create failed");
            lp_window_timer_ = nullptr;
            return;
        }
    }

    (void)esp_timer_stop(lp_window_timer_);  // no-op when not running
    lp_window_expired_ = false;
    if (esp_timer_start_once(lp_window_timer_,
                             static_cast<uint64_t>(APP_LP_COMM_WINDOW_MS) * 1000ULL) != ESP_OK) {
        ESP_LOGE(TAG, "low power window timer start failed");
    }
}

void RadioPing::disarm_lp_window_timer()
{
    if (lp_window_timer_ != nullptr) {
        (void)esp_timer_stop(lp_window_timer_);
    }
    lp_window_expired_ = false;
}

void RadioPing::enter_low_power_cad()
{
    if (mode_ != Mode::idle) return;

    const void *ctx = radio_.ral.context;

    // Back in CAD: cancel the window deadline so a stale expiry cannot cut the
    // next wakeup short.
    disarm_lp_window_timer();

    if (!low_power_cad_active_) {
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();
        low_power_cad_active_ = true;
        cad_wakeup_ms_ = 0;
        lp_standby_pending_ = true;
        ESP_LOGI(TAG, "entering low power CAD mode");
    }

    // Release the camera DVP (rebuilt on demand). The app refuses while a
    // capture owns it, so keep asking each CAD cycle.
    if (lp_standby_pending_ && low_power_standby_cb_) {
        if (low_power_standby_cb_(true)) {
            lp_standby_pending_ = false;
        }
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();

    if (!configure_lora_cad()) {
        smtc_modem_hal_unprotect_api_call();
        ESP_LOGE(TAG, "CAD config failed, fallback to FLRC RX");
        low_power_cad_active_ = false;
        configure_flrc();
        schedule_rx();
        return;
    }

    ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK);
    lr20xx_radio_lora_set_cad(ctx);
    smtc_modem_hal_unprotect_api_call();

    cad_pending_ms_ = smtc_modem_hal_get_time_in_ms();
    mode_ = Mode::cad_pending;
}

void RadioPing::handle_cad_irq(ral_irq_t irq)
{
    mode_ = Mode::idle;
    cad_pending_ms_ = 0;

    if ((irq & RAL_IRQ_CAD_DONE) == 0) {
        ESP_LOGW(TAG, "CAD unexpected irq=0x%08lx", static_cast<unsigned long>(irq));
        return;
    }

    ESP_LOGI(TAG, "CAD done: %s", (irq & RAL_IRQ_CAD_OK) ? "activity detected" : "channel clear");

    if ((irq & RAL_IRQ_CAD_OK) != 0) {
        ESP_LOGI(TAG, "CAD detected activity, switching to FLRC RX (%ums comm window)",
                 static_cast<unsigned>(APP_LP_COMM_WINDOW_MS));
        low_power_cad_active_ = false;
        cad_wakeup_ms_ = smtc_modem_hal_get_time_in_ms();
        // Start the hard deadline for this wakeup. Whatever the node is doing
        // when it fires, it stops and goes back to CAD sleep.
        arm_lp_window_timer();
        smtc_modem_hal_protect_api_call();
        configure_flrc();
        smtc_modem_hal_unprotect_api_call();
        schedule_rx();
    } else {
        const void *ctx = radio_.ral.context;
        lr20xx_system_sleep_cfg_t sleep_cfg = {};
        sleep_cfg.is_clk_32k_enabled = 1;
        sleep_cfg.is_ram_retention_enabled = 1;
        smtc_modem_hal_protect_api_call();
        lr20xx_system_set_sleep_mode(ctx, &sleep_cfg, 0);
        smtc_modem_hal_unprotect_api_call();

        // LR2021 is now asleep and SPI is idle, so light-sleep the ESP32 too
        // for the 500ms CAD off-period. Wakes on timer (next CAD) or PIR.
        bool woken_by_pir = low_power_sleep(500);

        if (woken_by_pir) {
            // PIR is a self-initiated push: do not touch the radio here;
            // image_tx_task takes it over entirely. Only keep the loop from
            // dropping back into CAD sleep before tx_task fires the capture.
            pir_push_wake_ = true;
            pir_push_wake_ms_ = smtc_modem_hal_get_time_in_ms();
            ESP_LOGI(TAG, "PIR wake: staying awake, capture will push image");
        }
    }
}

bool RadioPing::send_lora_wakeup()
{
    const void *ctx = radio_.ral.context;

    ESP_LOGI(TAG, "sending LoRa wakeup (508 symbol preamble)");

    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending || mode_ == Mode::cad_pending) {
        ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        cad_pending_ms_ = 0;
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);

    lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
    lr20xx_radio_common_set_rf_freq(ctx, APP_FLRC_FREQUENCY_HZ);

    lr20xx_radio_lora_mod_params_t mod = {};
    mod.sf = LR20XX_RADIO_LORA_SF7;
    mod.bw = LR20XX_RADIO_LORA_BW_125;
    mod.cr = LR20XX_RADIO_LORA_CR_4_5;
    mod.ppm = LR20XX_RADIO_LORA_NO_PPM;
    lr20xx_radio_lora_set_modulation_params(ctx, &mod);

    lr20xx_radio_lora_pkt_params_t pkt = {};
    pkt.preamble_len_in_symb = 508;
    pkt.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT;
    pkt.pld_len_in_bytes = 4;
    pkt.crc = LR20XX_RADIO_LORA_CRC_ENABLED;
    pkt.iq = LR20XX_RADIO_LORA_IQ_STANDARD;
    lr20xx_radio_lora_set_packet_params(ctx, &pkt);

    uint8_t dummy[4] = {0xCA, 0xFE, 0x00, 0x01};
    lr20xx_radio_fifo_write_tx(ctx, dummy, 4);

    ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    lr20xx_radio_common_set_tx(ctx, 2000);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::tx_pending;
    bool ok = wait_for_tx_done(1500);
    if (!ok) {
        ESP_LOGE(TAG, "LoRa wakeup TX timeout");
    } else {
        ESP_LOGI(TAG, "LoRa wakeup sent");
    }
    return ok;
}

void RadioPing::send_vbat_broadcast()
{
    // Battery voltage broadcast: minimal FLRC packet with current cached voltage.
    // Layout: [0..13] 14-byte header (magic + type=12, rest zeroed)
    //         [14..15] vbat_mv (u16 LE)
    //         [16..19] CRC32 covering [0..15]
    uint8_t pkt[kHeaderSize + 6];
    std::memset(pkt, 0, sizeof(pkt));
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeVbat;
    put_u16_le(&pkt[14], bsp_vbat_get_cached());
    put_u32_le(&pkt[16], crc32_ieee(pkt, 16));

    send_single_packet(pkt, kHeaderSize + 6);
    ESP_LOGI(TAG, "vbat broadcast sent: %u mV", bsp_vbat_get_cached());
}

bool RadioPing::doorbell_link_busy() const
{
    // A call or an image transfer owns the packet plane outright.
    if (intercom_active_ || image_tx_active_ || suspended_ || ptt_active_) {
        return true;
    }
    // Low power: the whole communication window counts as busy; its hard timer
    // clears cad_wakeup_ms_, so this cannot get stuck.
    if (cad_wakeup_ms_ != 0 || pir_push_wake_) {
        return true;
    }
    // Non-low-power has no window to key off, so cover the capture handshake
    // with the dispatch timestamp instead. See notify_capture_starting().
    if (last_capture_dispatch_ms_ != 0 &&
        (smtc_modem_hal_get_time_in_ms() - last_capture_dispatch_ms_) <
            APP_DOORBELL_CAPTURE_GUARD_MS) {
        return true;
    }
    return false;
}

void RadioPing::doorbell_trigger()
{
#if APP_DOORBELL_ENABLE
    // Only the door station rings.
    if (is_gateway_) return;

    const uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (doorbell_last_press_ms_ != 0 &&
        (now - doorbell_last_press_ms_) < APP_DOORBELL_PRESS_COOLDOWN_MS) {
        return;
    }
    doorbell_last_press_ms_ = now;

    // One ring per visitor session.
    if (doorbell_session_open()) {
        ESP_LOGI(TAG, "doorbell press dropped: visitor session still open");
        return;
    }

    // Decide at press time: a press latched during a transfer would fire the
    // instant it cleared, on top of the gateway's next request.
    if (doorbell_link_busy()) {
        ESP_LOGI(TAG, "doorbell press dropped: link busy");
        return;
    }

    doorbell_pending_ = true;
    // Wake the radio task now instead of after the next poll period.
    if (task_handle_ != nullptr) xTaskNotifyGive(task_handle_);
#endif
}

void RadioPing::service_doorbell()
{
#if APP_DOORBELL_ENABLE
    if (!doorbell_pending_ || is_gateway_) return;

    // A TX is already on air; the burst can wait one poll pass rather than
    // trampling it. Everything else is either idle or something we drop below.
    if (mode_ == Mode::tx_pending) return;

    // Second gate: the link may have gone busy since the press. Never queue
    // for later.
    if (doorbell_link_busy()) {
        doorbell_pending_ = false;
        ESP_LOGI(TAG, "doorbell press dropped: link went busy");
        return;
    }

    doorbell_pending_ = false;
    send_doorbell_burst();
#endif
}

void RadioPing::send_doorbell_burst()
{
    // Take the radio back to FLRC standby first (the HAL wakes the chip on the
    // first SPI access). The gateway never sleeps, so no LoRa preamble.
    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending || mode_ == Mode::cad_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        cad_pending_ms_ = 0;
        mode_ = Mode::idle;
    }
    smtc_modem_hal_start_radio_tcxo();
    const bool flrc_ok = configure_flrc();
    smtc_modem_hal_unprotect_api_call();

    if (!flrc_ok) {
        ESP_LOGE(TAG, "doorbell: FLRC config failed, press dropped");
        return;
    }
    // The radio is out of LoRa CAD now. Clear the flag so the idle path re-enters
    // CAD from scratch instead of assuming it is still armed.
    low_power_cad_active_ = false;

    // Layout mirrors kPacketTypeVbat: [0..13] header, [14..15] event id,
    // [16..19] CRC32 over [0..15].
    uint8_t pkt[kHeaderSize + 6];
    std::memset(pkt, 0, sizeof(pkt));
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeDoorbell;
    put_u16_le(&pkt[14], ++doorbell_event_id_);
    put_u32_le(&pkt[16], crc32_ieee(pkt, 16));

    unsigned sent = 0;
    for (unsigned i = 0; i < APP_DOORBELL_PACKET_REPEAT; i++) {
        if (send_single_packet(pkt, kHeaderSize + 6)) sent++;
        if (i + 1U < APP_DOORBELL_PACKET_REPEAT) {
            vTaskDelay(ms_to_ticks_min_1(APP_DOORBELL_PACKET_GAP_MS));
        }
    }
    ESP_LOGI(TAG, "doorbell burst sent: id=%u %u/%u packets",
             static_cast<unsigned>(doorbell_event_id_), sent,
             static_cast<unsigned>(APP_DOORBELL_PACKET_REPEAT));

    // The ring opens a visitor session here too: requested frames carry the
    // microphone and further presses are refused. Time-based, because the
    // node never learns when the gateway's ring is over.
    uint32_t until = smtc_modem_hal_get_time_in_ms() + APP_DOORBELL_SESSION_MS;
    if (until == 0) until = 1;   // 0 is the "no session" value
    doorbell_session_until_ms_.store(until, std::memory_order_release);
}

bool RadioPing::doorbell_session_open() const
{
    const uint32_t until = doorbell_session_until_ms_.load(std::memory_order_acquire);
    if (until == 0) return false;
    return (int32_t)(until - smtc_modem_hal_get_time_in_ms()) > 0;
}

void RadioPing::handle_doorbell_packet(uint16_t len)
{
    if (len < kHeaderSize + 6) return;
    if (get_u32_le(&rx_buf_[16]) != crc32_ieee(rx_buf_, 16)) {
        ESP_LOGW(TAG, "RX Doorbell CRC mismatch, dropping");
        return;
    }
    const uint16_t event_id = get_u16_le(&rx_buf_[14]);

    // Drop only retransmissions of this press, never a new one. The window
    // outlasts one burst; expiring it keeps a rebooted node's first press
    // from being mistaken for a repeat.
    const uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (doorbell_have_last_event_ && event_id == doorbell_last_event_id_ &&
        (now - doorbell_last_event_ms_) < APP_DOORBELL_EVENT_DEDUP_MS) {
        ESP_LOGD(TAG, "RX Doorbell id=%u repeat, already rung", event_id);
        return;
    }
    doorbell_last_event_id_ = event_id;
    doorbell_have_last_event_ = true;
    doorbell_last_event_ms_ = now;

    ESP_LOGI(TAG, "RX Doorbell id=%u -> chime", event_id);
    if (doorbell_cb_ != nullptr) doorbell_cb_();
}

// ---- Link air-traffic accounting ------------------------------------------
// Rate = bytes actually put on air / time spent putting them there. Numerator:
// every FLRC packet in both directions at full on-air length, retransmissions
// included. Denominator: a rolling window of the last completed frames trimmed
// to kLinkStatsWindowMs; with a single frame, that frame's own transfer time.
// FPS is only reported with two or more frames in the window.
void RadioPing::note_air_bytes(uint16_t len)
{
    link_bytes_total_ += len;
}

void RadioPing::note_air_rssi(int16_t rssi)
{
    // Bound the accumulator (a node never publishes these); halving both
    // keeps the average.
    if (link_rssi_count_ >= 4096U) {
        link_rssi_sum_ /= 2;
        link_rssi_count_ /= 2U;
    }
    link_rssi_sum_ += rssi;
    link_rssi_count_++;
}

void RadioPing::link_stats_frame_done(uint32_t transfer_ms)
{
    const uint32_t now = smtc_modem_hal_get_time_in_ms();

    // Bytes attributable to this frame alone: everything counted since the
    // previous frame ended. Only used for the single-frame case below.
    const uint32_t prev_bytes =
        (link_mark_count_ > 0) ? link_marks_[link_mark_count_ - 1U].bytes_total : 0U;
    link_last_frame_bytes_ = link_bytes_total_ - prev_bytes;
    link_last_transfer_ms_ = transfer_ms;

    if (link_mark_count_ == kLinkStatsMarks) {
        for (uint8_t i = 1; i < kLinkStatsMarks; i++) {
            link_marks_[i - 1U] = link_marks_[i];
        }
        link_mark_count_--;
    }
    link_marks_[link_mark_count_].end_ms = now;
    link_marks_[link_mark_count_].bytes_total = link_bytes_total_;
    link_mark_count_++;

    link_stats_publish();
}

void RadioPing::link_stats_publish()
{
    const uint32_t now = smtc_modem_hal_get_time_in_ms();

    // Drop marks that fell out of the window, but always keep the newest so a
    // single recent frame can still report its own air rate.
    while (link_mark_count_ > 1U &&
           (now - link_marks_[0].end_ms) > kLinkStatsWindowMs) {
        for (uint8_t i = 1; i < link_mark_count_; i++) {
            link_marks_[i - 1U] = link_marks_[i];
        }
        link_mark_count_--;
    }

    uint32_t bytes_per_s = 0;

    if (link_mark_count_ >= 2U) {
        const LinkMark &oldest = link_marks_[0];
        const LinkMark &newest = link_marks_[link_mark_count_ - 1U];
        // Unsigned differences, so a wrap of either counter is handled.
        const uint32_t dt = newest.end_ms - oldest.end_ms;
        const uint32_t dbytes = newest.bytes_total - oldest.bytes_total;
        if (dt > 0U) {
            bytes_per_s =
                static_cast<uint32_t>(static_cast<uint64_t>(dbytes) * 1000ULL / dt);
        }
    } else if (link_mark_count_ == 1U && link_last_transfer_ms_ > 0U &&
               (now - link_marks_[0].end_ms) <= kLinkStatsWindowMs) {
        bytes_per_s = static_cast<uint32_t>(
            static_cast<uint64_t>(link_last_frame_bytes_) * 1000ULL /
            link_last_transfer_ms_);
    }

    int16_t rssi_avg = 0;
    const bool rssi_valid = link_rssi_count_ > 0U;
    if (rssi_valid) {
        rssi_avg = static_cast<int16_t>(link_rssi_sum_ /
                                        static_cast<int32_t>(link_rssi_count_));
    }
    // Reset per publish so the average tracks the current window instead of all
    // history. Done even with no callback registered, to bound the accumulator.
    link_rssi_sum_ = 0;
    link_rssi_count_ = 0;

    if (link_stats_cb_ != nullptr) {
        link_stats_cb_(bytes_per_s, rssi_avg, rssi_valid);
    }
}

void RadioPing::vbat_maintenance_tick()
{
    // Gateway never broadcasts its voltage, only receives from nodes.
    if (is_gateway_) return;

    uint32_t now = smtc_modem_hal_get_time_in_ms();

    if (g_low_power_enabled) {
        // Low-power node: sample while already awake, never wake for it. No
        // broadcast; the voltage rides in ImageStart instead.
        if ((int32_t)(now - vbat_last_sample_ms_) >= (int32_t)kVbatLowPowerSampleIntervalMs) {
            int mv = bsp_vbat_read_mv();
            if (mv >= 0) {
                ESP_LOGD(TAG, "vbat sample: %d mV", mv);
            }
            vbat_last_sample_ms_ = now;
        }
        return;
    }

    // Non-low-power node: bsp_vbat samples; broadcast the cached voltage
    // periodically (radio is already in FLRC RX).
    if ((int32_t)(now - vbat_last_broadcast_ms_) >= (int32_t)kVbatBroadcastIntervalMs) {
        send_vbat_broadcast();
        vbat_last_broadcast_ms_ = now;
    }
}

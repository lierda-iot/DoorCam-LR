#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "app_config.h"
#include "bsp.h"
#include "LiotLr2021.h"
#include "opus_codec.hpp"
#include "audio_processor.hpp"
#include "echo_canceller.hpp"
#include "howl_suppressor.hpp"
#include "image_transfer.hpp"

// Return true only after accepting the capture or a same-session retransmit;
// rejected requests remain eligible for gateway retry.
typedef bool (*image_capture_cb_t)(uint16_t session_id);
typedef void (*image_rx_complete_cb_t)(ImageTransfer *xfer);
typedef void (*image_rx_progress_cb_t)(uint16_t received, uint16_t total, int16_t rssi);
typedef void (*image_rx_eot_cb_t)(uint16_t missing_count, bool is_first_eot);
typedef void (*config_received_cb_t)(uint8_t key, uint32_t value);
// Gateway: node battery voltage arrived (ImageStart or periodic Vbat broadcast).
typedef void (*vbat_received_cb_t)(uint16_t vbat_mv);
// Release or restore peripherals around CAD standby. A failed release is
// retried on the next CAD cycle.
typedef bool (*low_power_standby_cb_t)(bool entering);
typedef void (*intercom_state_cb_t)(bool active);
typedef void (*doorbell_cb_t)(void);
// Gateway visitor stream callbacks. expired distinguishes ring timeout from a
// user answer or navigation event.
typedef bool (*doorbell_stream_start_cb_t)(void);
typedef void (*doorbell_stream_end_cb_t)(bool expired);
// Aggregate air traffic in both directions for the gateway status bar.
typedef void (*link_stats_cb_t)(uint32_t bytes_per_s,
                                int16_t rssi_avg,
                                bool rssi_valid);

class RadioPing {
public:
    esp_err_t init();
    esp_err_t init_gateway();
    esp_err_t start();
    esp_err_t start_gateway();
    void handle_button(bsp_btn_id_t id, bool pressed);
    void suspend();
    void resume();

    // Image transfer: B triggers A to capture
    void trigger_image_capture();
    // Gateway: request abort of the in-progress image RX. Only sets a flag; the
    // radio task tears the state down so radio access stays serialized.
    void abort_image_rx();
    // Stop requesting frames and wait for the accepted transfer to finish so
    // the node is listening before call setup. Abort after timeout_ms.
    void quiesce_image_stream(uint32_t timeout_ms);
    // Image transfer: A sends JPEG fragments to B. Takes ownership of `jpeg`
    // (heap_caps allocation); the tx task frees it. Callers must not free it.
    void send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id);
    // Register callbacks
    void set_image_capture_cb(image_capture_cb_t cb) { image_capture_cb_ = cb; }
    void set_image_rx_complete_cb(image_rx_complete_cb_t cb) { image_rx_complete_cb_ = cb; }
    void set_image_rx_progress_cb(image_rx_progress_cb_t cb) { image_rx_progress_cb_ = cb; }
    void set_vbat_received_cb(vbat_received_cb_t cb) { vbat_received_cb_ = cb; }
    void set_image_rx_eot_cb(image_rx_eot_cb_t cb) { image_rx_eot_cb_ = cb; }
    void set_config_received_cb(config_received_cb_t cb) { config_received_cb_ = cb; }
    void set_low_power_standby_cb(low_power_standby_cb_t cb) { low_power_standby_cb_ = cb; }
    void set_intercom_state_cb(intercom_state_cb_t cb) { intercom_state_cb_ = cb; }
    void set_link_stats_cb(link_stats_cb_t cb) { link_stats_cb_ = cb; }
    void set_inter_packet_us(uint32_t us) { image_tx_inter_packet_us_ = us; }

    bool send_config(uint8_t key, uint32_t value);
    bool set_intercom(bool enable);
    bool intercom_active() const { return intercom_active_ && intercom_start_confirmed_; }

    // Gateway: trailing Opus bytes of the last completed payload ([JPEG][audio])
    // and the sink that takes them. Valid only between a completed transfer and
    // the next ImageStart; radio task only.
    uint16_t image_rx_audio_len() const { return image_rx_audio_len_; }
    uint16_t image_rx_audio_seq() const { return image_rx_audio_seq_; }
    void deliver_image_audio(const uint8_t *blob, size_t len);
    // Gateway: same sink fed from a stored visitor record. `first_seq` is the
    // original arrival sequence so jitter/concealment sees the real timeline.
    void play_stored_audio(const uint8_t *blob, size_t len, uint16_t first_seq);
    void flush_playback_audio();

    // Publishes an encoded JPEG to the radio task. Takes ownership of `jpeg`.
    void intercom_image_publish(uint8_t *jpeg, size_t jpeg_len);
    // Gateway: invoked once per new image frame with a reassembled JPEG copy the
    // callback takes ownership of (must free it).
    using IntercomImageFrameCb = void (*)(uint8_t *jpeg, size_t len,
                                          uint16_t session, uint16_t fragments,
                                          uint32_t transfer_ms,
                                          uint32_t reassemble_ms, void *ctx);
    void set_intercom_image_frame_cb(IntercomImageFrameCb cb, void *ctx) {
        intercom_img_frame_cb_ = cb;
        intercom_img_frame_cb_ctx_ = ctx;
    }

    ImageTransfer &image_xfer() { return image_xfer_; }
    uint32_t last_transfer_ms() const { return image_rx_transfer_ms_; }
    bool image_tx_busy() const { return image_tx_active_; }
    // Gateway: the transfer in flight was pushed by the node unasked (PIR
    // snapshot). Progress/EOT UI callbacks are skipped for such a transfer.
    bool image_rx_unsolicited() const { return image_rx_unsolicited_; }
    void set_pir_enabled(bool en) { pir_enabled_ = en; }
    void IRAM_ATTR pir_trigger() { pir_triggered_ = true; }
    // PIR level trigger armed state. Cleared by the ISR on detection, set again
    // by the re-arm timer; low_power_sleep only arms the GPIO wake while true.
    void IRAM_ATTR set_pir_armed(bool armed) { pir_armed_ = armed; }
    bool pir_armed() const { return pir_armed_; }

    // A PIR-triggered capture was dropped before it could push: end the
    // keep-awake guard now instead of waiting for the safety timeout.
    void notify_capture_dropped() { pir_push_wake_ = false; }

    // A capture was dispatched (gateway request or PIR). In low power the node
    // must stay awake through capture + push, otherwise CAD light sleep starves
    // the camera/JPEG task. Reuses the PIR keep-awake guard; image_tx_task
    // clears it.
    void notify_capture_starting();

    // Node: doorbell key pressed. Only latches a request; the radio task sends
    // the burst from its idle path, so this is safe from the button callback or
    // a light-sleep wake. Dropped while a call/transfer is up or in the cooldown.
    void doorbell_trigger();

    // Gateway: fired when a doorbell packet arrives and passes the chime
    // cooldown. Runs in the radio task, so the handler must not block.
    void set_doorbell_cb(doorbell_cb_t cb) { doorbell_cb_ = cb; }

    // Gateway: run the visitor session's video stream alongside the ring. Starts
    // the ordinary requested stream and arms the session deadline from the radio
    // task's idle path. The deadline is the ring's clock too, so it is armed even
    // if the stream fails to start; abort_image_rx() disarms it and reports
    // `expired = false`. While armed, A/V audio in the frames is dropped.
    void doorbell_stream_request(uint32_t duration_ms);
    void set_doorbell_stream_cbs(doorbell_stream_start_cb_t start,
                                 doorbell_stream_end_cb_t end)
    {
        doorbell_stream_start_cb_ = start;
        doorbell_stream_end_cb_ = end;
    }

private:
    enum class Mode {
        idle,
        rx_pending,
        tx_pending,
        cad_pending,
    };

    static void task_trampoline(void *arg);
    static void tx_task_trampoline(void *arg);
    static void play_task_trampoline(void *arg);
    static void image_tx_task_trampoline(void *arg);
    static void irq_callback(void *context);

    void task();
    void tx_task();
    void play_task();
    void poll_once();
    void handle_irq(ral_irq_t irq);
    void schedule_rx();
    void schedule_tx();
    bool configure_flrc();
    bool build_voice_packet(uint16_t *tx_size, uint8_t flags = 0);
    void capture_voice_packet();
    void enqueue_voice_frame(const uint8_t *payload, uint16_t len);
    void start_intercom_local(uint16_t session);
    void stop_intercom_local();
    void service_intercom();
    bool send_intercom_slot(uint8_t flags);
    /* Append optional padded timing probes after a node voice reply. */
    void send_intercom_probe_burst(uint16_t count);
    // Prepare and send in-call image fragments through the slot tail.
    void prepare_intercom_image();
    void send_intercom_image_burst(uint16_t count);
    void handle_intercom_image_data(uint16_t len);
    void finalize_intercom_image_rx_session();
    void log_intercom_image_stats(bool final);
    esp_err_t init_rx_stream();
    bool leave_rx_for_tx();
    void handle_rx_packet();
    void process_rx_stream(int16_t rssi);
    void dispatch_rx_packet(uint16_t len, int16_t rssi);
    bool queue_voice_packet(uint16_t len, int16_t rssi);
    // Protect stream audio with av_audio_lock_; sequence gaps drive the normal
    // jitter-buffer loss concealment on the gateway.
    void av_audio_push(const uint8_t *opus, uint8_t len);
    size_t av_audio_drain(uint8_t *out, size_t out_cap, uint16_t *out_first_seq);
    void av_audio_reset();
    void av_audio_deliver(const uint8_t *blob, size_t len, uint16_t first_seq,
                          int16_t rssi);
    // Enabled mode != stream in progress: in low power the microphone is armed
    // by image activity and lapses APP_AV_AUDIO_ACTIVE_WINDOW_MS later.
    void av_audio_note_activity();
    bool av_audio_capture_due() const;
    // Node: whether requested frames carry the microphone. Compile-time for the
    // ordinary stream, always for the ring window.
    bool av_audio_wanted() const;
    // Serialize Opus encode, decode, and reset across tasks. Keep the lock out
    // of AEC and I2S operations so playback cannot be starved.
    void codec_lock() { xSemaphoreTake(codec_lock_, portMAX_DELAY); }
    void codec_unlock() { xSemaphoreGive(codec_lock_); }
    // Gateway A/V playback conditioning: howl detection, fixed attenuation, then
    // zeroing while muted. A synthetic (PLC) frame only inherits the mute state.
    void av_playback_process(int16_t *pcm, size_t samples, bool synthetic);
    void log_rx(uint16_t seq, uint16_t len, int16_t rssi);
    void wait_for_jitter_buffer();
    void conceal_missing_frames(uint16_t seq, bool av_stream);
    bool read_mono_frame(int16_t *mono, size_t samples);
    // Called by tx_task on every failed read: after a run of failures with
    // nothing releasing the I2S, rebuilds it (the only path that would).
    void note_mic_read_failure();
    // pa_on false leaves the PA as it is: call fillers must not switch it back
    // on after stop_intercom_local() has turned it off.
    bool play_mono_frame(const int16_t *mono, size_t samples, bool pa_on = true);
    void set_playback_pa(bool on);
    void update_playback_timeout();
    // Call playout keeper, play task only; see APP_INTERCOM_PLAYOUT_GUARD_MS.
    // fill_wait() is how long the task may block for the next packet before
    // the ring reaches the guard (portMAX_DELAY when no call is playing).
    TickType_t intercom_fill_wait();
    bool intercom_fill_due() const;
    void intercom_fill_frame();
    void note_playout(size_t samples);

    // Image transfer methods
    void handle_image_cmd();
    void handle_image_cmd_ack();
    // Consume an external capture request in the radio task, then start it.
    // UI/esp_timer callers only set image_capture_req_ and wake this task.
    void check_image_capture_request();
    void start_image_capture_request();
    // Send one ImageCmd for image_req_session_ (build + TX only; wakeup + FLRC
    // reconfig happen once per round in start_image_req_round).
    void send_image_cmd_once();
    // Low power: begin a request round. Sends the LoRa wakeup preamble (trips the
    // node's CAD scan) + FLRC reconfig, then floods ImageCmd for the node's wake
    // window. The wakeup is a no-op outside low power.
    void start_image_req_round();
    // Radio task loop: resend ImageCmd when the retry interval elapses without
    // an ack. Same task as poll_once so radio access stays serialized.
    void check_image_req_retry();
    void stop_image_req_retry();
    void handle_image_start(uint16_t len);
    void handle_image_data(uint16_t len);
    void handle_image_eot();
    void handle_image_nack();
    void handle_image_done();
    void image_tx_task();
    // Low power (node): release image_tx_task's keep-awake state. `preempted`
    // means the gateway is flooding ImageCmd for a newer session: stay in FLRC.
    void finish_image_tx_wake_state(bool preempted);
    bool send_single_packet(const uint8_t *data, uint16_t len,
                            uint32_t timeout_ms = 50U);
    struct ImageTxRequest;
    uint16_t build_image_fragment(uint8_t *pkt, const ImageTxRequest &req,
                                  uint16_t frag_index, uint16_t total_fragments);
    void burst_send_fragments(const ImageTxRequest &req, uint16_t total_fragments,
                              const uint16_t *indices, uint16_t count);
    bool wait_for_tx_done(uint32_t timeout_ms);
    void check_image_rx_timeout();
    void check_image_rx_abort();
    void check_image_rx_quiesce();
    void send_config_ack(uint8_t key, uint32_t value);
    bool configure_lora_cad();
    void enter_low_power_cad();
    // Low power (node): the communication window is a hard deadline. The
    // esp_timer callback must not touch the radio state machine, so it only
    // raises lp_window_expired_; the radio task and image_tx_task unwind their
    // own work.
    void arm_lp_window_timer();
    void disarm_lp_window_timer();
    static void lp_window_timer_cb(void *arg);
    // Light-sleep for up to `ms`, waking on the timer or (if enabled) the PIR
    // GPIO. Returns true if woken by PIR.
    bool low_power_sleep(uint32_t ms);
    void handle_cad_irq(ral_irq_t irq);

    // True while an unsolicited doorbell burst would disturb the link. Checked
    // at the key press and again before the burst goes out.
    bool doorbell_link_busy() const;
    // Node: send the pending doorbell burst if one is latched and the link is
    // idle. Called from the radio task's idle path only.
    void service_doorbell();
    void send_doorbell_burst();
    // Gateway: a doorbell packet arrived. Applies the chime cooldown.
    void handle_doorbell_packet(uint16_t len);
    // Node: true from a ring until APP_DOORBELL_SESSION_MS later. Read by the
    // microphone task (A/V arming) and the button path (re-ring refusal).
    bool doorbell_session_open() const;
    // Gateway: radio-task checkpoint for the visitor stream (start + deadline).
    void service_doorbell_stream();

    // Battery voltage broadcast: send a small FLRC packet with current cached voltage.
    void send_vbat_broadcast();
    // Maintenance tick for low-power nodes: sample voltage every 60s, broadcast every 5min.
    void vbat_maintenance_tick();
    bool send_lora_wakeup();

    // Link air-traffic accounting for the gateway status bar. Called from the RX
    // dispatch point and both FLRC TX sites (all packet types, both directions).
    // Not called for the LoRa wakeup burst: 4 payload bytes in ~530 ms of
    // preamble would drag the rate to nonsense.
    void note_air_bytes(uint16_t len);
    // Record the RSSI of one received packet into the current window.
    void note_air_rssi(int16_t rssi);
    // One image frame finished; `transfer_ms` is the denominator when it is the
    // only frame in the window. Recomputes and fires link_stats_cb_.
    void link_stats_frame_done(uint32_t transfer_ms);
    // Recompute and publish without a frame mark (Vbat heartbeat refresh).
    void link_stats_publish();

    static constexpr uint32_t kLinkStatsWindowMs = 10000U;
    static constexpr uint8_t kLinkStatsMarks = 9U; /* 9 marks = 8 intervals */

    struct LinkMark {
        uint32_t end_ms;
        uint32_t bytes_total; /* cumulative counter value at this frame's end */
    };

    link_stats_cb_t link_stats_cb_ = nullptr;
    LinkMark link_marks_[kLinkStatsMarks] = {};
    uint8_t link_mark_count_ = 0;
    uint32_t link_bytes_total_ = 0;
    uint32_t link_last_transfer_ms_ = 0;
    uint32_t link_last_frame_bytes_ = 0;
    int32_t link_rssi_sum_ = 0;
    uint32_t link_rssi_count_ = 0;

    struct VoicePacket {
        uint16_t seq;
        uint16_t len;
        int16_t rssi;
        uint8_t payload[APP_OPUS_MAX_PACKET_BYTES];
        // Frames split out of an image payload (one-way A/V) were encoded raw
        // on the node, so the player must skip the voice path's de-emphasis.
        // Carried per packet: the wire, not the mode flag, says what a frame is.
        bool av_stream;
    };

    struct TxFrame {
        uint16_t seq;
        uint16_t len;
        uint8_t payload[APP_OPUS_MAX_PACKET_BYTES];
    };

    struct ImageTxRequest {
        const uint8_t *jpeg;
        size_t jpeg_len;
        uint16_t session_id;
    };

    // One length-prefixed Opus frame for the next image payload.
    struct AvAudioFrame {
        uint16_t seq;
        uint8_t len;
        uint8_t data[APP_OPUS_MAX_PACKET_BYTES];
    };

    // Absorb the rate difference between microphone frames and image payloads.
    AvAudioFrame av_audio_ring_[APP_AV_AUDIO_RING_FRAMES] = {};
    uint16_t av_audio_head_ = 0;      // next write slot
    uint16_t av_audio_count_ = 0;     // frames currently queued
    uint16_t av_audio_next_seq_ = 0;  // sequence to stamp on the next capture
    uint16_t av_audio_tail_seq_ = 0;  // sequence of the OLDEST queued frame
    uint32_t av_audio_dropped_ = 0;   // ring overflows, for the log only
    // Written by the radio task, polled by the microphone task's tight loop and
    // read by the image TX task. Atomic access is required across cores.
    std::atomic<uint32_t> av_audio_active_until_ms_{0};
    portMUX_TYPE av_audio_lock_ = portMUX_INITIALIZER_UNLOCKED;
    // Staging buffer for the audio appended to one image payload (kept off the
    // image TX task stack).
    uint8_t av_audio_blob_[APP_AV_AUDIO_MAX_BLOB_BYTES] = {};
    // Gateway: audio metadata for the transfer currently being reassembled,
    // taken from ImageStart and applied once the payload is complete.
    uint16_t image_rx_audio_len_ = 0;
    uint16_t image_rx_audio_seq_ = 0;
    // Gateway: A/V playback feedback killer. Playback task only.
    HowlSuppressor av_howl_;
    uint32_t av_play_frames_ = 0;
    uint32_t av_play_clip_max_ = 0;
    uint32_t av_play_rms_max_ = 0;
    // Both roles: call feedback killer on the AEC output. Microphone task
    // only; the per-window maxima feed the periodic "intercom mic" line.
    HowlSuppressor intercom_howl_{HowlSuppressor::kCallCapture};
    uint32_t intercom_mic_clip_max_ = 0;
    uint32_t intercom_mic_rms_max_ = 0;

    static RadioPing *instance_;

    TaskHandle_t task_handle_ = nullptr;
    ralf_t radio_ = RALF_LR20XX_INSTANTIATE(nullptr);
    OpusCodec codec_;
    AudioProcessor audio_proc_;
    EchoCanceller echo_canceller_;
    ImageTransfer image_xfer_;
    QueueHandle_t voice_queue_ = nullptr;
    QueueHandle_t tx_queue_ = nullptr;
    SemaphoreHandle_t codec_lock_ = nullptr;   // see codec_lock()
    QueueHandle_t image_tx_queue_ = nullptr;
    Mode mode_ = Mode::idle;
    volatile bool ptt_active_ = false;
    volatile bool suspended_ = false;
    // tx_task only: when the current run of failed microphone reads began
    // (0 = none), and rebuilds tried since one last succeeded.
    int64_t mic_read_fail_since_us_ = 0;
    uint32_t mic_rebuild_attempts_ = 0;
    bool tx_burst_active_ = false;
    bool tx_flush_pending_ = false;
    volatile bool irq_pending_ = false;
    // Task blocked in send_single_packet waiting for TX_DONE; the ISR notifies
    // it directly. nullptr => the ISR notifies task_handle_.
    volatile TaskHandle_t tx_done_waiter_ = nullptr;
    // ISR timestamp anchoring intercom slots to packet arrival, not task dispatch.
    volatile int64_t last_irq_us_ = 0;
    int64_t rx_done_us_ = 0;             // task: snapshot at the RX_DONE that woke us

    uint8_t tx_buf_[APP_FLRC_MAX_PAYLOAD_BYTES] = {};
    uint8_t rx_buf_[APP_FLRC_MAX_PAYLOAD_BYTES] = {};
    uint8_t *rx_stream_buf_ = nullptr;
    size_t rx_stream_size_ = 0;
    int16_t rx_stream_rssi_ = 0;
    int16_t tx_pcm_[APP_AUDIO_FRAME_SAMPLES] = {};
    int16_t rx_pcm_[APP_AUDIO_FRAME_SAMPLES] = {};

    uint16_t tx_seq_ = 0;
    uint16_t expected_rx_seq_ = 0;
    bool have_expected_rx_seq_ = false;
    uint32_t rx_packets_ = 0;
    uint32_t rx_lost_ = 0;
    uint32_t rx_crc_errors_ = 0;
    uint32_t rx_hdr_errors_ = 0;
    uint32_t rx_unknown_packets_ = 0;
    uint32_t rx_queue_drops_ = 0;
    uint32_t tx_queue_drops_ = 0;
    uint32_t last_rx_audio_ms_ = 0;
    uint16_t expected_play_seq_ = 0;
    bool have_expected_play_seq_ = false;
    bool playback_pa_on_ = false;
    bool playback_active_ = false;
    // Play task only. playout_end_us_ estimates when the speaker ring runs out
    // of written audio: each frame is appended at the rate the I2S plays it,
    // from "now" when the ring was already empty. The estimate is never later
    // than the truth (a first write lands after the descriptor in flight), so
    // the keeper errs towards filling early. playout_primed_: a call frame has
    // reached the ring and the keeper is in charge until hang-up.
    int64_t playout_end_us_ = 0;
    bool playout_primed_ = false;
    uint32_t intercom_fill_run_ = 0;    // fillers since the last real frame
    volatile bool intercom_active_ = false;
    volatile bool intercom_start_confirmed_ = false;
    volatile bool intercom_stop_confirmed_ = false;
    bool intercom_reply_pending_ = false;
    bool intercom_stop_reply_ = false;
    bool intercom_stop_requested_ = false;
    uint16_t intercom_session_ = 0;
    uint16_t intercom_prepared_session_ = 0;
    uint16_t intercom_last_stopped_session_ = 0;
    int64_t intercom_next_slot_us_ = 0;
    int64_t intercom_reply_due_us_ = 0;
    uint32_t intercom_last_sync_ms_ = 0;
    uint32_t intercom_tx_slots_ = 0;
    uint32_t intercom_rx_slots_ = 0;
    uint32_t intercom_missed_slots_ = 0;
    /* Optional slot-timing probe counters. */
    uint32_t intercom_probe_tx_ = 0;
    uint32_t intercom_probe_rx_ = 0;
    uint32_t intercom_probe_sent_ = 0;
    uint32_t intercom_probe_deadline_stops_ = 0;
    // RX re-arm margin before the next master slot.
    int64_t intercom_img_rearm_target_us_ = 0;
    int32_t intercom_img_rearm_min_slack_us_ = 0;
    bool intercom_img_rearm_slack_valid_ = false;
    uint32_t intercom_img_rearm_late_ = 0;
    // Gateway counters for appended image and probe delivery.
    uint32_t intercom_masters_tx_ = 0;   // masters sent (per-slot denominator)
    uint32_t intercom_voice_rx_ = 0;     // node voice replies received (~100% baseline)
    uint32_t intercom_rearm_after_rx_ = 0; // schedule_rx() re-arms during a live session
    // In-call image state shared by the node sender and gateway reassembler.
    uint8_t *intercom_img_buf_ = nullptr;   // node: the fixed frame (PSRAM)
    size_t   intercom_img_len_ = 0;         // node: frame byte length
    uint16_t intercom_img_total_frags_ = 0; // node+gw: fragments per frame
    uint16_t intercom_img_cursor_ = 0;      // node: next fragment index to send
    uint32_t intercom_img_frames_tx_ = 0;   // node: frames whose fragments were sent once
    uint32_t intercom_img_frames_rx_ = 0;   // gw: complete frames reassembled
    uint32_t intercom_img_frags_rx_ = 0;    // gw: valid fragments accepted
    uint32_t intercom_img_rate_ms_ = 0;     // gw: window origin for frames/s log
    uint32_t intercom_img_frames_adopted_ = 0;
    uint32_t intercom_img_frag_tx_attempts_ = 0;
    uint32_t intercom_img_frag_tx_done_ = 0;
    uint32_t intercom_img_frag_tx_timeouts_ = 0;
    uint32_t intercom_img_sessions_seen_ = 0;
    uint32_t intercom_img_sessions_unseen_ = 0;
    uint32_t intercom_img_sessions_incomplete_ = 0;
    uint32_t intercom_img_missing_unique_ = 0;
    uint32_t intercom_img_frags_unique_ = 0;
    uint32_t intercom_img_frags_duplicate_ = 0;
    uint32_t intercom_img_frag_seq_lost_ = 0;
    uint32_t intercom_img_crc_errors_ = 0;
    uint32_t intercom_img_malformed_ = 0;
    uint32_t intercom_img_alloc_failures_ = 0;
    uint32_t intercom_img_reassemble_failures_ = 0;
    uint32_t intercom_img_rx_frame_start_ms_ = 0;
    uint16_t intercom_img_expected_frag_ = 0;
    bool     intercom_img_have_expected_frag_ = false;
    bool     intercom_img_current_complete_ = false;
    // Per-frame image session ID, independent of the voice session ID.
    uint16_t intercom_img_session_ = 0;     // node: current frame's session id
    uint16_t intercom_img_rx_session_ = 0;  // gw: session currently being reassembled
    bool     intercom_img_rx_active_ = false; // gw: reassembly open for a session
    uint16_t intercom_img_shown_session_ = 0; // gw: last session already pushed to UI
    bool     intercom_img_shown_valid_ = false;
    // Node cross-core handoff: the capture-feed task (core1) publishes the newest
    // encoded JPEG here under intercom_img_lock_; the radio task (core0) adopts it at
    // a frame boundary. Ownership of the blob transfers to the radio task on adopt.
    uint8_t *intercom_img_pending_ = nullptr;
    size_t   intercom_img_pending_len_ = 0;
    portMUX_TYPE intercom_img_lock_ = portMUX_INITIALIZER_UNLOCKED;
    IntercomImageFrameCb intercom_img_frame_cb_ = nullptr;
    void    *intercom_img_frame_cb_ctx_ = nullptr;
    uint32_t intercom_mic_frames_ = 0;
    uint32_t intercom_play_frames_ = 0;
    uint64_t intercom_aec_us_total_ = 0;
    uint32_t intercom_aec_us_max_ = 0;
    uint32_t intercom_input_clip_samples_ = 0;

    // Image transfer state
    image_capture_cb_t image_capture_cb_ = nullptr;
    image_rx_complete_cb_t image_rx_complete_cb_ = nullptr;
    image_rx_progress_cb_t image_rx_progress_cb_ = nullptr;
    vbat_received_cb_t vbat_received_cb_ = nullptr;
    image_rx_eot_cb_t image_rx_eot_cb_ = nullptr;
    config_received_cb_t config_received_cb_ = nullptr;
    low_power_standby_cb_t low_power_standby_cb_ = nullptr;
    intercom_state_cb_t intercom_state_cb_ = nullptr;
    uint16_t image_session_id_ = 1;
    volatile bool image_tx_active_ = false;
    uint32_t image_tx_inter_packet_us_ = APP_IMAGE_TX_INTER_PACKET_US;
    uint32_t image_rx_last_frag_ms_ = 0;
    uint32_t image_rx_last_progress_ms_ = 0;
    uint32_t image_rx_expected_crc32_ = 0;
    // Written by the radio task; quiesce_image_stream() polls it from the UI
    // task through the volatile read.
    volatile bool image_rx_pending_ = false;
    std::atomic<bool> image_capture_req_{false};
    std::atomic<bool> image_rx_abort_req_{false};
    std::atomic<bool> image_rx_quiesce_req_{false};
    // Suppress ImageStart ACKs during call setup so image data cannot collide
    // with the configuration exchange.
    volatile bool intercom_dialing_ = false;
    uint16_t image_rx_nack_sent_ = 0;
    uint16_t image_rx_eot_count_ = 0;
    int16_t image_rx_last_rssi_ = 0;
    uint16_t image_rx_done_session_ = 0;
    uint32_t image_req_debug_last_ms_ = 0;
    uint16_t image_cmd_debug_session_ = 0;
    // Retries keep one session ID and run on the radio task.
    bool image_req_active_ = false;
    uint16_t image_req_session_ = 0;
    uint32_t image_req_next_ms_ = 0;
    // Low-power request rounds restart with a new LoRa wake-up.
    uint32_t image_req_round_end_ms_ = 0;
    // After ImageCmdAck, leave the channel clear for ImageStart; retry on timeout.
    uint32_t image_start_wait_ms_ = 0;
    uint32_t image_cmd_sent_ms_ = 0;
    uint32_t image_rx_request_ms_ = 0;
    uint32_t image_rx_start_ms_ = 0;
    uint32_t image_rx_transfer_ms_ = 0;
    uint32_t image_rx_done_ms_ = 0;

    // Pre-empt an obsolete transfer at its next handshake checkpoint.
    volatile bool image_tx_preempt_req_ = false;

    // NACK receive state for TX side (A)
    volatile bool image_nack_received_ = false;
    volatile bool image_done_received_ = false;
    uint16_t nack_indices_[APP_IMAGE_NACK_MAX_INDICES] = {};
    uint16_t nack_count_ = 0;

    // Config ACK state
    volatile bool config_ack_received_ = false;

    // Low power CAD state
    bool low_power_cad_active_ = false;
    bool is_gateway_ = false;
    uint32_t cad_wakeup_ms_ = 0;
    // Hard deadline for all FLRC work after a CAD wake-up.
    esp_timer_handle_t lp_window_timer_ = nullptr;
    volatile bool lp_window_expired_ = false;
    // Retry peripheral release when a capture still owns the hardware.
    bool lp_standby_pending_ = false;
    // Watchdog timestamp for a missing CAD_DONE interrupt; zero means idle.
    uint32_t cad_pending_ms_ = 0;
    // PIR self-push keeps the node awake until image_tx_task owns the radio.
    volatile bool pir_push_wake_ = false;
    uint32_t pir_push_wake_ms_ = 0;

    // PIR trigger state. Node-initiated captures use session ids from 0xC000
    // up, well clear of the gateway's request counter, so the two never
    // collide in the node's single-flight capture guard.
    bool pir_enabled_ = false;
    int64_t last_trigger_us_ = 0;
    uint16_t trigger_session_id_ = 0xC000;
    volatile bool pir_triggered_ = false;
    volatile bool pir_armed_ = false;
    // Gateway: see image_rx_unsolicited().
    bool image_rx_unsolicited_ = false;
    // When a capture was last dispatched. Covers the gap between dispatch and
    // image_tx_task taking the radio, during which no other flag is set yet.
    uint32_t last_capture_dispatch_ms_ = 0;

    // Doorbell. doorbell_pending_ is set from the button/light-sleep wake path
    // and consumed by the radio task's idle path, so only that task touches the
    // radio.
    doorbell_cb_t doorbell_cb_ = nullptr;
    volatile bool doorbell_pending_ = false;
    uint32_t doorbell_last_press_ms_ = 0;
    uint16_t doorbell_event_id_ = 0;
    // Gateway: last event id rung, and when. Collapses one press's burst into a
    // single ring without rate-limiting presses; the id expires after
    // APP_DOORBELL_EVENT_DEDUP_MS so a node reboot cannot be swallowed.
    uint16_t doorbell_last_event_id_ = 0;
    bool doorbell_have_last_event_ = false;
    uint32_t doorbell_last_event_ms_ = 0;
    // Node: end of the visitor session opened by its own ring (0 = none).
    // Written by the radio task in send_doorbell_burst, read from the
    // microphone task on the other core and from the button task.
    std::atomic<uint32_t> doorbell_session_until_ms_{0};
    // Gateway visitor session: requested/started/ended from the radio task,
    // disarmed from whichever task aborts the stream.
    doorbell_stream_start_cb_t doorbell_stream_start_cb_ = nullptr;
    doorbell_stream_end_cb_t doorbell_stream_end_cb_ = nullptr;
    std::atomic<bool> doorbell_stream_start_req_{false};
    std::atomic<bool> doorbell_stream_armed_{false};
    uint32_t doorbell_stream_duration_ms_ = 0;
    uint32_t doorbell_stream_deadline_ms_ = 0;

    // Battery maintenance timestamps (low-power node only; others use bsp_vbat).
    uint32_t vbat_last_sample_ms_ = 0;
    uint32_t vbat_last_broadcast_ms_ = 0;
};

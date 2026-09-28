#pragma once

/* Application-level media, radio, and UI configuration. */

/* ----- Audio capture/playback ------------------------------------------------ */

#define APP_AUDIO_FEATURES_ENABLE       1
#define APP_RADIO_FEATURES_ENABLE       1
#define APP_RADIO_HW_INIT_ENABLE        1
#define APP_RADIO_AUTO_RX_ENABLE        1
#define APP_RADIO_TASKS_ENABLE          1

/* Wi-Fi is disabled to reserve internal RAM for the media pipelines. */

/* ES8311/I2S sample rate used by both the microphone and speaker paths. */
#define APP_AUDIO_SAMPLE_RATE_HZ        16000U

/* The voice path is mono; stereo I2S samples are mixed down before encoding. */
#define APP_AUDIO_CHANNELS              1U

/* PCM sample format passed to the Opus encoder and produced by the decoder. */
#define APP_AUDIO_BITS_PER_SAMPLE       16U

/* 10 ms reduces per-frame encode blocking and makes packet loss less audible. */
#define APP_AUDIO_FRAME_MS              10U

/* Number of mono PCM samples in one Opus frame at APP_AUDIO_SAMPLE_RATE_HZ. */
#define APP_AUDIO_FRAME_SAMPLES         ((APP_AUDIO_SAMPLE_RATE_HZ * APP_AUDIO_FRAME_MS) / 1000U)

/* Number of bytes in one mono 16-bit PCM frame before Opus compression. */
#define APP_AUDIO_FRAME_BYTES           (APP_AUDIO_FRAME_SAMPLES * (APP_AUDIO_BITS_PER_SAMPLE / 8U))

/* Size used for blocking I2S reads/writes in the diagnostic local audio path. */
#define APP_AUDIO_IO_CHUNK_BYTES        2048U

/* ----- Opus voice codec ------------------------------------------------------ */

/* Low-delay mode avoids the heavier SILK VOIP path that was hitting WDT. */
#define APP_OPUS_APPLICATION            OPUS_APPLICATION_RESTRICTED_LOWDELAY

/* The stable FLRC link has enough budget for clearer low-delay speech. */
#define APP_OPUS_BITRATE_BPS            32000

/* Fixed bitrate makes packet sizing and radio scheduling easier to debug. */
#define APP_OPUS_USE_VBR                0

/* Complexity 1 leaves CPU headroom for JPEG encoding and AEC. */
#define APP_OPUS_COMPLEXITY             1

/* Fixed-rate audio keeps radio scheduling predictable. */
#define APP_OPUS_USE_DTX                0

/* Reserve enough room for a 20 ms voice packet at the target bitrate plus slack. */
#define APP_OPUS_MAX_PACKET_BYTES       96U

/* ----- FLRC radio link ------------------------------------------------------- */

/* Center frequency. Confirm the exact channel is legal for the deployment area. */
#define APP_FLRC_FREQUENCY_HZ           915120000UL

/* LR2021 FLRC high-rate mode requested for this project.
 * Informational only: the upgraded driver selects rate and bandwidth together
 * through APP_FLRC_RAW_BIT_RATE. */
#define APP_FLRC_BITRATE_BPS            2600000UL

/* Double-sided bandwidth paired with 2.6 Mbps; retained for diagnostics. */
#define APP_FLRC_BANDWIDTH_HZ           2666000UL

/* The LR2021 driver API uses enum values for bitrate and preamble length. */
#define APP_FLRC_RAW_BIT_RATE           RAL_FLRC_RAW_BIT_RATE_2_600_MBPS
#define APP_FLRC_PREAMBLE_LEN           RAL_FLRC_PREAMBLE_LENGTH_32_BITS

/* Verify that the configured power is permitted in the deployment region. */
#define APP_FLRC_TX_POWER_DBM           22

/* Rate 1/1 maximizes throughput; validate range for the installation. */
#define APP_FLRC_CODING_RATE            RAL_FLRC_CR_1_1

/* BT=0.5 keeps spectrum cleaner than no shaping at high FLRC data rates. */
#define APP_FLRC_PULSE_SHAPE            RAL_FLRC_PULSE_SHAPE_BT_05

/* Keep the existing voice packet aggregation boundary unchanged. */
#define APP_FLRC_VOICE_MAX_PAYLOAD_BYTES 255U

/* Keep the 2-byte FIFO command plus payload at 512 bytes. The radio HAL uses
 * stack buffers, and a 512-byte aligned polling transfer can use the project's
 * reserved DMA bounce buffers without allocating temporary internal memory. */
#define APP_FLRC_MAX_PAYLOAD_BYTES      510U
#define APP_FLRC_BURST_PAYLOAD_LEN      510U

/* Pack several 10 ms Opus frames per FLRC packet to reduce TX overhead while
 * keeping each radio packet to about 50 ms of audio. */
#define APP_FLRC_OPUS_FRAMES_PER_PACKET 5U

/* Gateway-master TDD intercom. Each master slot is followed by one node reply. */
#define APP_INTERCOM_SLOT_PERIOD_MS     20U
#define APP_INTERCOM_NODE_GUARD_US      2000U
#define APP_INTERCOM_TX_TIMEOUT_MS      10U
#define APP_INTERCOM_START_TIMEOUT_MS   1200U
#define APP_INTERCOM_STOP_TIMEOUT_MS    800U
#define APP_INTERCOM_SYNC_TIMEOUT_MS    250U
#define APP_INTERCOM_LINK_TIMEOUT_MS    1200U
#define APP_INTERCOM_TX_QUEUE_FRAMES    4U
#define APP_INTERCOM_FRAMES_PER_PACKET  2U

/* Wait for an active stream frame to finish before starting a call. */
#define APP_STREAM_QUIESCE_MS           400U

/* In-call image fragments use the idle tail of each node voice slot. The burst
 * runs below AEC priority and stops early enough to re-arm radio RX. */
#define APP_INTERCOM_IMAGE_GRANT_MAX    4U
#define APP_INTERCOM_IMAGE_PROBE        4U
#define APP_INTERCOM_IMAGE_BURST_PRIORITY   2
#define APP_INTERCOM_IMAGE_BURST_GUARD_US   6000
/* Enable JPEG fragments in the in-call slot-tail transport. */
#define APP_INTERCOM_IMAGE_ENABLE       1
#define APP_INTERCOM_IMAGE_FRAME_BYTES  5000U
/* Optional capture-only diagnostic during a call; zero disables it. */
#define APP_INTERCOM_IMAGE_CAPTURE_PROBE_MS  0U
/* Delay after each in-call capture and encode. A positive value selects live
 * JPEG frames; zero selects the diagnostic synthetic-frame path. */
#define APP_INTERCOM_IMAGE_REAL_CAPTURE_MS   100U
/* Playback attenuation is applied before both the AEC reference and I2S. */
#define APP_INTERCOM_INPUT_GAIN         1
#define APP_INTERCOM_PLAYBACK_PERCENT   50U
#define APP_INTERCOM_AEC_ENABLE         1

/* ESP-SR recommends filter length 4 for ESP32-S3 full-duplex AEC. */
#define APP_AFE_AEC_FILTER_LENGTH       4U

/* Residual-echo suppression (NLP) level while the adaptive filter converges
 * and afterwards. Levels: 0 normal, 1 aggressive (the ESP-SR default), 2 very
 * aggressive. A call is a closed loop through two speaker-to-microphone
 * couplings, so whatever residual both AECs leave circulates; relaxing to 0
 * after the warm-up gave clearer double-talk but, in a quiet room with a far
 * link, the residual grew into a periodic howl. 1 keeps the loop margin;
 * APP_INTERCOM_HOWL_* is the backstop if it still closes. */
#define APP_AFE_AEC_NLP_LEVEL_STARTUP   1
#define APP_AFE_AEC_NLP_LEVEL_STEADY    1
#define APP_AFE_AEC_NLP_WARMUP_MS       3000U

/* Call feedback killer, on the microphone path after the AEC of both boxes.
 * Nothing in a quiet room masks the residual echo, and once the loop gain
 * passes unity it grows into a howl within a few round trips. While the AEC
 * output looks like a howl the frames go out as silence, which opens the loop
 * at this box, and the NLP goes back to the startup level. Same features and
 * scale as the A/V killer: the AEC output at APP_INTERCOM_INPUT_GAIN 1 is the
 * raw microphone scale, like the decoded stream audio the A/V thresholds were
 * calibrated on. */
#define APP_INTERCOM_HOWL_ENABLE            1
#define APP_INTERCOM_HOWL_CLIP_LEVEL        30000   /* |sample| at/over this is "clipped" */
#define APP_INTERCOM_HOWL_ON_CLIP_PERCENT   5U
#define APP_INTERCOM_HOWL_ON_RMS            6000U
#define APP_INTERCOM_HOWL_ON_FRAMES         3U      /* 30 ms */
#define APP_INTERCOM_HOWL_OFF_CLIP_PERCENT  2U
#define APP_INTERCOM_HOWL_OFF_RMS           4500U
#define APP_INTERCOM_HOWL_OFF_FRAMES        50U     /* 500 ms, > one loop round trip */
#define APP_INTERCOM_HOWL_MAX_MUTE_FRAMES   150U    /* 1.5 s hard cap on one mute */

/* Use a shallow I2S ring during calls to keep the AEC reference aligned, and a
 * deep ring during camera operation to tolerate capture and encode latency. */
#define APP_INTERCOM_NODE_DMA_DESC_NUM      6U
#define APP_AUDIO_NODE_DEFAULT_DMA_DESC_NUM 24U

/* RX timeout used by the packet receiver before it re-arms listening. */
#define APP_FLRC_RX_TIMEOUT_MS          100U

/* Extra gap after each voice TX packet. Keep at 0 for continuous 20 ms audio. */
#define APP_FLRC_VOICE_TX_GAP_MS        0U

/* Log one voice frame every N packets to avoid flooding the serial console. */
#define APP_VOICE_LOG_EVERY_N           25U

/* Poll period for the RAC engine task. Keep well below one audio frame so TX
 * done/RX done is handled without stretching the 20 ms voice cadence. */
#define APP_RADIO_TASK_POLL_MS          2U

/* FreeRTOS priority for the radio engine task. */
#define APP_RADIO_TASK_PRIORITY         4

/* RAC callbacks and the radio loop use about 2.2 KB of this stack. */
#define APP_RADIO_TASK_STACK_BYTES      8192U

/* Keep direct RAL radio control on CPU0. */
#define APP_RADIO_TASK_CORE             0

/* Dedicated sync word for this project's FLRC test/audio packets. */
#define APP_FLRC_SYNC_WORD_0            0x4CU
#define APP_FLRC_SYNC_WORD_1            0x52U
#define APP_FLRC_SYNC_WORD_2            0x32U
#define APP_FLRC_SYNC_WORD_3            0x31U

/* ----- Internal button-voice path (not bound by the doorbell UI) ------------- */

/* Button used as push-to-talk. K1 is the ADC-ladder key near 0.82 V. */
#define APP_PTT_BUTTON                  BSP_BTN_PTT

/* Receiver-side jitter buffer target before starting speaker playback. */
#define APP_RX_JITTER_BUFFER_MS         60U

/* Number of encoded voice frames to queue before starting speaker playback. */
#define APP_RX_JITTER_FRAMES            ((APP_RX_JITTER_BUFFER_MS + APP_AUDIO_FRAME_MS - 1U) / APP_AUDIO_FRAME_MS)

/* Conceal one missing aggregated FLRC packet before resyncing. */
#define APP_RX_MAX_PLC_FRAMES           APP_FLRC_OPUS_FRAMES_PER_PACKET

/* Stop playback if no voice packet arrives within this interval. During a
 * call only the playback state resets; the PA stays on until hang-up (see
 * RadioPing::update_playback_timeout). */
#define APP_RX_AUDIO_TIMEOUT_MS         200U

/* During a call the play task never lets the speaker ring run dry: whenever
 * less than this much written audio is left, it writes a concealment (later a
 * silent) frame and feeds the same frame to the AEC reference. A dry ring
 * plays cleared buffers the reference never saw and restarts at a new phase,
 * which moves the echo the AEC has learnt. Keep it above two descriptors
 * (30 ms at 240 frames): below that the IDF writer can find its half-filled
 * descriptor next in line and skip the rest of it, a gap of zeros that is
 * just as invisible to the reference. */
#define APP_INTERCOM_PLAYOUT_GUARD_MS   35U

/* Number of encoded voice packets buffered between radio RX and playback.
 * Sized for the one-way A/V stream, which is the bursty consumer: the audio for
 * a whole image frame lands in a single batch (about 8 frames at 12 fps, more
 * when a frame needed NACK rounds), where the intercom delivers 5 at a time.
 * 12 would drop part of every batch; 40 is ~400 ms of headroom. The queue only
 * adds latency when it actually fills, so this does not slow the intercom. */
#define APP_VOICE_RX_QUEUE_LEN          40U

/* Number of encoded voice frames buffered between microphone and radio TX. */
#define APP_VOICE_TX_QUEUE_LEN          25U

/* Only print one TX queue overflow warning every N dropped voice frames. */
#define APP_TX_DROP_LOG_EVERY_N         25U

/* Opus decode plus I2S write run here so radio RX can re-arm quickly. */
#define APP_VOICE_PLAY_TASK_PRIORITY    3
/* Sized from measured task high-water usage. */
#define APP_VOICE_PLAY_TASK_STACK_BYTES 16384U

/* Let CPU0 radio priority preempt Opus decode/playback when TDD work arrives. */
#define APP_VOICE_PLAY_TASK_CORE        0

/* AEC runs on CPU1 so it cannot delay CPU0 radio TDD work. Playback and the
 * low-priority image feed run on CPU0; CPU1 is reserved mainly for AEC and
 * Opus encoding. */
#define APP_VOICE_TX_TASK_PRIORITY      4
/* Sized from measured task high-water usage. */
#define APP_VOICE_TX_TASK_STACK_BYTES   16384U

/* Keep AEC and direct RAL radio control on separate cores: a long AEC frame
 * must not delay radio RX re-arm, and radio IRQ handling must not starve AEC.
 * Playback shares CPU0 at lower priority so the radio can always preempt it. */
#define APP_VOICE_TX_TASK_CORE          1

/* ----- Audio DSP (noise suppression / voice enhancement) -------------------- */

#define APP_AUDIO_DSP_ENABLE              1
#define APP_AUDIO_DSP_PREEMPH_ALPHA_Q15   31785   /* 0.97 in Q15 — speech HF boost */
#define APP_AUDIO_DSP_NOISE_GATE_THRESH   200     /* RMS threshold to open gate */
#define APP_AUDIO_DSP_NOISE_GATE_ATTACK   3       /* frames above thresh to open */
#define APP_AUDIO_DSP_NOISE_GATE_RELEASE  4       /* smoothing shift (larger = slower) */
#define APP_AUDIO_DSP_NS_FLOOR_ADAPT      6       /* noise floor rise speed shift (larger = slower) */
#define APP_AUDIO_DSP_NS_FLOOR_DECAY      2       /* noise floor drop speed shift (larger = slower) */
#define APP_AUDIO_DSP_NS_MIN_GAIN_Q15     3277    /* minimum suppression gain ~0.10 (full atten floor) */
#define APP_AUDIO_DSP_NS_OVERSUBTRACT     3       /* over-subtraction factor shift (1=2x, 2=4x, 3=8x) */
#define APP_AUDIO_DSP_LIMITER_THRESHOLD   30000   /* soft-clip knee on RX playback */
#define APP_AUDIO_DSP_TX_MUTE_FRAMES      5       /* Mute initial TX frames to suppress echo tail */
#define APP_AUDIO_DSP_AGC_TARGET_Q15      22000   /* target RMS level in Q15 (~0.67 FS) */
#define APP_AUDIO_DSP_AGC_MAX_GAIN_Q15    (4 << 15) /* max 4x amplification */
#define APP_AUDIO_DSP_AGC_ATTACK_SHIFT    3       /* gain ramp-up speed (larger = slower) */
#define APP_AUDIO_DSP_AGC_RELEASE_SHIFT   5       /* gain ramp-down speed (larger = slower) */

/* ----- Local diagnostic tones ------------------------------------------------ */

#define APP_BEEP_FREQ_HZ                1800
#define APP_BEEP_ON_MS                  140
#define APP_BEEP_GAP_MS                 90
#define APP_BEEP_TAIL_MS                80
#define APP_BEEP_AMP                    12000
#define APP_PA_SETTLE_MS                40
/* Silence played AFTER the TX DMA ring has been flushed and BEFORE the PA is
 * switched off, so the speaker is provably sitting at zero when it happens.
 * Only shrinks the thump; the PA's own turn-off transient is hardware. */
#define APP_PA_DRAIN_MARGIN_MS          60
/* Set to 0 to make boot silent. */
#define APP_STARTUP_CHIME_ENABLE        1

/* Short low-amplitude boot chime. */
#define APP_STARTUP_CHIME_FREQ1_HZ      660
#define APP_STARTUP_CHIME_FREQ2_HZ      880
#define APP_STARTUP_CHIME_TONE_MS       90
#define APP_STARTUP_CHIME_GAP_MS        35
#define APP_STARTUP_CHIME_AMP           3500

/* ----- Doorbell ---------------------------------------------------------- */

/* K5 rings only while the door station and gateway are idle. */
#define APP_DOORBELL_ENABLE             1
#define APP_DOORBELL_KEY                BSP_BTN_VOL_DN
/* Repetition provides loss protection; the gateway de-duplicates event IDs. */
#define APP_DOORBELL_PACKET_REPEAT      3U
#define APP_DOORBELL_PACKET_GAP_MS      40U
/* This interval filters contact bounce without rate-limiting later presses. */
#define APP_DOORBELL_PRESS_COOLDOWN_MS  250U
/* Event IDs expire so a node reboot cannot collide with stale state. */
#define APP_DOORBELL_EVENT_DEDUP_MS     1000U
/* Block a ring while capture and the image-transfer handshake start. */
#define APP_DOORBELL_CAPTURE_GUARD_MS   3000U
/* Optional still-image push on each doorbell press. */
#define APP_DOORBELL_ALSO_CAPTURE       0

/* During the answer window, the gateway records door-side audio but plays only
 * the chime. An unanswered session becomes a visitor record. */
#define APP_DOORBELL_RING_MS            15000U
/* Silence between repeated chimes. */
#define APP_DOORBELL_RING_PAUSE_MS      1200U
/* Keep the node visitor session longer than the ring and wake-up latency. */
#define APP_DOORBELL_SESSION_MS         (APP_DOORBELL_RING_MS + 3000U)

/* Store compressed JPEG/Opus data in a PSRAM ring; records do not survive a
 * restart. New records evict the oldest. */
#define APP_VISITOR_RECORDS             2U
/* Capacity for one 15-second compressed visitor record. */
#define APP_VISITOR_RECORD_MAX_FRAMES   256U
#define APP_VISITOR_RECORD_MAX_BYTES    (1400U * 1024U)
/* First-frame thumbnail scale for the visitor list. */
#define APP_VISITOR_THUMB_SCALE         4U
/* PIR snapshots use a separate PSRAM ring and do not change the active page. */
#define APP_VISITOR_SNAPSHOTS           5U
/* Let stream audio release the PA before a pre-empting chime starts. */
#define APP_DOORBELL_PREEMPT_CHIME_DELAY_MS (APP_RX_AUDIO_TIMEOUT_MS + 100U)

/* Two-tone struck-tube door chime. */
#define APP_DOORBELL_DING_HZ            659   /* E5 */
#define APP_DOORBELL_DONG_HZ            523   /* C5 */
#define APP_DOORBELL_TONE_MS            620
#define APP_DOORBELL_ATTACK_MS          6     /* strike edge, avoids a click */
#define APP_DOORBELL_DECAY_MS           240   /* exponential decay constant */
/* Ramp to zero before muting the PA to avoid a click. */
#define APP_DOORBELL_RELEASE_MS         30
#define APP_DOORBELL_GAP_MS             40
#define APP_DOORBELL_AMP                14000
#define APP_DOORBELL_PARTIAL_PERCENT    30    /* 2.76f partial, % of fundamental */
#define APP_DOORBELL_REPEAT             1     /* ding-dong sequences per repetition */

/* ----- Node configuration keys (gateway -> node CONFIG packets) ------------ */

/* Retired key values stay reserved for compatibility with older firmware. */
#define APP_CFG_KEY_INTER_PACKET        0x02
#define APP_CFG_KEY_PIR_TRIGGER         0x05
#define APP_CFG_KEY_LOW_POWER           0x07
#define APP_CFG_KEY_INTERCOM            0x08
/* Value is the JPEG quality for every frame the node encodes; see
 * APP_IMAGE_JPEG_QUALITY. */
#define APP_CFG_KEY_JPEG_QUALITY        0x09
/* Node: two PIR detections closer than this fire one capture. */
#define APP_TRIGGER_COOLDOWN_SEC        15U
#define APP_PIR_GPIO                    GPIO_NUM_12

/* ----- One-way A/V stream (node video + node voice) ---------------------- */

/* In view/listen mode, node audio is appended to the JPEG payload and shares
 * image retransmission. The gateway microphone remains closed, so local AEC is
 * unnecessary; gain control and howl suppression limit cross-device feedback. */
#define APP_AV_STREAM_ENABLE            1

/* Buffer node audio between image payloads; overflow drops the oldest frame. */
#define APP_AV_AUDIO_RING_FRAMES        48U

/* Upper bound on the audio blob appended to one image payload. At the current
 * 32 kbps CBR / 10 ms this is 41 bytes per frame, so 2048 bytes is ~500 ms of
 * audio and about 4 extra fragments in the worst case. */
#define APP_AV_AUDIO_MAX_BLOB_BYTES     2048U

/* Stop the stream microphone after image activity ends so low power can resume. */
#define APP_AV_AUDIO_ACTIVE_WINDOW_MS   2000U

/* Apply direct attenuation because stream audio has no TX pre-emphasis. */
#define APP_AV_PLAYBACK_PERCENT         50U

/* Mute playback while decoded PCM exceeds both calibrated clipping and RMS
 * thresholds. Resume after the acoustic loop has decayed. */
#define APP_AV_HOWL_ENABLE              1
#define APP_AV_HOWL_CLIP_LEVEL          30000   /* |sample| at/over this is "clipped" */
#define APP_AV_HOWL_ON_CLIP_PERCENT     5U
#define APP_AV_HOWL_ON_RMS              6000U
#define APP_AV_HOWL_ON_FRAMES           3U      /* 30 ms */
#define APP_AV_HOWL_OFF_CLIP_PERCENT    2U
#define APP_AV_HOWL_OFF_RMS             4500U
#define APP_AV_HOWL_OFF_FRAMES          50U     /* 500 ms, > one loop round trip */
#define APP_AV_HOWL_MAX_MUTE_FRAMES     150U    /* 1.5 s hard cap on one mute */
#define APP_AV_HOWL_LOG_EVERY_FRAMES    100U    /* 1 s; 0 = no periodic print */

/* ----- Image transfer over FLRC ------------------------------------------ */

/* JPEG quality 1-100; lower values reduce encode time and radio airtime.
 * This is the door station's default; the gateway Settings page changes it
 * over a CONFIG packet (APP_CFG_KEY_JPEG_QUALITY) and the node keeps it in
 * NVS. The page steps through MIN..MAX; the node accepts any value in that
 * range. */
#define APP_IMAGE_JPEG_QUALITY          25
#define APP_IMAGE_JPEG_QUALITY_MIN      15
#define APP_IMAGE_JPEG_QUALITY_MAX      95
#define APP_IMAGE_JPEG_QUALITY_STEP     10
#define APP_IMAGE_FRAGMENT_DATA_SIZE    (APP_FLRC_MAX_PAYLOAD_BYTES - 16U)
#define APP_IMAGE_TX_INTER_PACKET_US    0U
#define APP_IMAGE_RX_TIMEOUT_MS         3000U
#define APP_IMAGE_RX_PROGRESS_INTERVAL_MS 100U
#define APP_IMAGE_NACK_MAX_RETRIES      400U
#define APP_IMAGE_NACK_MAX_INDICES      120U
#define APP_IMAGE_MAX_JPEG_SIZE         (400U * 1024U)
#define APP_IMAGE_EOT_RETRY_COUNT       10U
#define APP_IMAGE_EOT_RETRY_INTERVAL_MS 30U
/* Keep ImageStart and ImageCmd retry periods non-harmonic to avoid repeated
 * half-duplex collisions. */
#define APP_IMAGE_START_RETRY_COUNT     600U
#define APP_IMAGE_START_RETRY_INTERVAL_MS 50U
/* Low-power retries include a new LoRa wake-up preamble. */
#define APP_IMAGE_REQ_RETRY_INTERVAL_MS    30U
#define APP_IMAGE_REQ_RETRY_INTERVAL_LP_MS 1000U
/* One low-power request round spans the node communication window. */
#define APP_IMAGE_REQ_ROUND_MS          APP_LP_COMM_WINDOW_MS
/* Restart the request round if capture fails after ImageCmdAck. */
#define APP_IMAGE_START_WAIT_MS         4000U
/* Abort a low-power transfer after this long without gateway feedback. */
#define APP_LP_WAKE_WINDOW_MS           8000U
/* Hard deadline for all FLRC work after a CAD wake-up. Expiry stops active
 * media, releases the camera, and returns the node to CAD standby. */
#define APP_LP_COMM_WINDOW_MS           15000U
#define APP_IMAGE_TASK_STACK_BYTES      16384U
#define APP_IMAGE_TASK_PRIORITY         3
#define APP_IMAGE_TASK_CORE             1
/* TX preempts image encoding so packet service does not depend on codec yields. */
#define APP_IMAGE_TX_TASK_PRIORITY      4
/* Keep radio TX on core 0 while capture and JPEG encoding run on core 1. */
#define APP_IMAGE_TX_TASK_CORE          0

/* Continuous video stream: after a frame finishes displaying, wait this long
 * before auto-requesting the next one. Small gap lets the half-duplex radio
 * settle and the UI repaint; 0 = request immediately. Tune for frame rate. */
#define APP_STREAM_NEXT_FRAME_DELAY_MS  10U

/* ----- Camera node LCD (set 0 to skip LCD release/reinit for faster capture) */
#define APP_CAMERA_NODE_LCD_ENABLE      0

/* ----- Camera/LCD diagnostics -------------------------------------------- */

#define APP_CAMERA_LCD_BRINGUP          0

#define APP_CAMERA_UART_ENABLE          0

#define APP_CAMERA_ONLY_BRINGUP         0

#define APP_CON6_FORCE_CAMERA           0

#define APP_CAMERA_UART_BAUD            2000000

/* A 10 MHz sensor clock prevents DVP FIFO overruns while camera, PSRAM, AEC,
 * and Opus share memory bandwidth. */
#define APP_CAMERA_COLOR_ENABLE         1
#define APP_SP0A39_MCLK_HZ              10000000U
#define APP_SP0A39_I2C_ADDR             0x21U

/* LCD_CAM DVP input polarity. Toggle these when the sensor clocks/syncs are
 * visible but DMA captures only blank data. */
#define APP_CAMERA_DVP_VSYNC_INVERT     1
#define APP_CAMERA_DVP_HSYNC_INVERT     1
/* LCD_CAM DVP PCLK sampling edge. 0 = rising, 1 = falling. */
#define APP_CAMERA_DVP_PCLK_INVERT      0

/* VGA 640x480 from DVP. The sensor always outputs this native resolution; the
 * DVP controller and DMA capture buffers are sized to it and must not shrink. */
#define APP_CAMERA_SENSOR_WIDTH         640U
#define APP_CAMERA_SENSOR_HEIGHT        480U
#define APP_CAMERA_FRAME_BYTES          (APP_CAMERA_SENSOR_WIDTH * APP_CAMERA_SENSOR_HEIGHT * (APP_CAMERA_COLOR_ENABLE ? 2U : 1U))

/* Center-crop and 2x-decimate VGA to the 288x240 display area. Both output
 * dimensions stay aligned to 16-pixel JPEG MCUs and UYVY pixel pairs. */
#define APP_IMAGE_OUTPUT_WIDTH          288U
#define APP_IMAGE_OUTPUT_HEIGHT         240U
#define APP_IMAGE_OUTPUT_BYTES          (APP_IMAGE_OUTPUT_WIDTH * APP_IMAGE_OUTPUT_HEIGHT * (APP_CAMERA_COLOR_ENABLE ? 2U : 1U))

/* DMA buffer configuration for DVP capture. */
#define APP_CAMERA_DVP_DMA_WINDOW_BYTES (16U * 1024U)
#define APP_CAMERA_DVP_DMA_BUFFER_COUNT 4U

#define APP_CAMERA_UART_CHUNK_BYTES     2048U
/* SPI preprocessing is synchronous, so diagnostics run at idle priority. */
#define APP_CAMERA_TASK_PRIORITY        0
#define APP_CAMERA_TASK_STACK_BYTES     12288U
#define APP_CAMERA_TASK_CORE            1

/* ----- ST7789V3 LCD --------------------------------------------------------- */

#define APP_LCD_H_RES                   240U
#define APP_LCD_V_RES                   320U
#define APP_LCD_X_GAP                   0
#define APP_LCD_Y_GAP                   0
/* Commands use the conservative clock; APP_LCD_SPI_PCLK_HZ is pixel DMA only. */
#define APP_LCD_SPI_CMD_PCLK_HZ         10000000U
#define APP_LCD_SPI_PCLK_HZ             16000000U
#define APP_LCD_SPI_PIXEL_DUTY_CYCLE_POS 64U
#define APP_LCD_SPI_QUEUE_DEPTH         10U
#define APP_LCD_SPI_SCLK_DRIVE_CAP      3U
#define APP_LCD_SPI_MOSI_DRIVE_CAP      3U
#define APP_LCD_SPI_CS_DRIVE_CAP        3U
#define APP_LCD_SPI_DC_DRIVE_CAP        3U
#define APP_LCD_TEST_PATTERN_ROWS       20U
#define APP_LCD_LVGL_BUFFER_ROWS        40U
#define APP_LCD_LVGL_TICK_MS            2U
#define APP_LCD_LVGL_TASK_DELAY_MS      1U
#define APP_LCD_LVGL_TASK_STACK_BYTES   8192U
/* Keep LVGL below the voice tasks that share core 1. */
#define APP_LCD_LVGL_TASK_PRIORITY      2
#define APP_LCD_LVGL_TASK_CORE          1
#define APP_LCD_PHOTO_PREVIEW_H         180U

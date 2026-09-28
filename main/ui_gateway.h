#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_PAGE_IMAGE = 0,
    UI_PAGE_RX,
    UI_PAGE_VISITORS,   /* visitor records (took the place of the link page) */
    UI_PAGE_CONFIG,
    UI_PAGE_INTERCOM,
    UI_PAGE_PLAYBACK,   /* a visitor record playing full screen */
    UI_PAGE_COUNT
} ui_page_t;

typedef bool (*ui_gw_capture_cb_t)(void);
/* The node's PIR switch: on a detection the node pushes one snapshot, which
 * lands on the visitor page (see visitor_store_snapshot_add). */
typedef bool (*ui_gw_pir_trigger_cb_t)(uint32_t enable);
typedef bool (*ui_gw_low_power_cb_t)(uint32_t enable);
/* JPEG quality for the node's encoder (APP_IMAGE_JPEG_QUALITY_MIN..MAX). */
typedef bool (*ui_gw_jpeg_quality_cb_t)(uint32_t quality);
/* Start (1) / end (0) the two-way call. Not a setting: the capture key dials
 * it as the second step of a session (see ui_gw_key_event) and the UI ends it
 * when the user steps back to listening or leaves the page. */
typedef bool (*ui_gw_intercom_cb_t)(uint32_t enable);
// Called when the user leaves the transfer (RX) page: abort the current RX.
typedef void (*ui_gw_rx_abort_cb_t)(void);

esp_err_t ui_gw_init(void);
/* Keys. The session key (K5) runs the session the way a video door phone's
 * monitor key does: first press views the door and listens to it (node
 * microphone on the speaker, gateway microphone closed - the status bar shows
 * a muted mic), second press opens the two-way call (live mic), third press
 * drops back to listening. During a ring it answers straight into the call.
 * The page keys (K1/K2/K4) end whatever is up. */
void ui_gw_key_event(bsp_btn_id_t key, bool pressed);

void ui_gw_rx_begin(uint16_t session_id, uint16_t total_frags);
void ui_gw_rx_progress(uint16_t received, uint16_t total, int16_t rssi);
void ui_gw_rx_complete(const uint16_t *rgb565, uint32_t w, uint32_t h,
                       uint32_t jpeg_size, uint32_t elapsed_ms);
void ui_gw_rx_failed(const char *reason);

void ui_gw_rx_eot_nack(uint16_t missing_count, bool is_first_eot);

/* True while a continuous video stream is running (user pressed capture and has
 * not left the image page). app_main uses this to auto-request the next frame. */
bool ui_gw_stream_active(void);

/* Doorbell visitor session: the bell rings while the door is shown (video
 * only); the capture key ANSWERS by dialling the call; otherwise the ring
 * times out. All three take the LVGL lock and may be called from any task.
 *   preempt:  a ring arrived while the user was watching a stream - end that
 *             stream so the chime can own the speaker. Returns true if one was
 *             running.
 *   begin:    show the ring-time picture exactly as the capture key would,
 *             except it never dials by itself. Marks the session as on screen
 *             even if no request could be sent (returns false then), so the
 *             key still answers a ring without a picture.
 *   ring_end: not answered in time - stop streaming and leave the last frame
 *             on the image page. No-op once the user has answered or left. */
bool ui_gw_doorbell_preempt(void);
bool ui_gw_doorbell_stream_begin(void);
void ui_gw_doorbell_ring_end(void);

/* Visitor records. The list page reads the store directly; playback goes
 * through the store's task and comes back here when it is over.
 *   make_thumbnail: scale a decoded, unrotated RGB565 frame into the list's
 *                   thumbnail, in the image canvas pixel format (rotated the
 *                   way the screen shows it). Registered with the store.
 *   records_changed: a record was kept or evicted - rebuild the list if it
 *                   is on screen (its thumbnails point into the store).
 *   playback_ended: the store's playback task finished (or was stopped):
 *                   leave the playback page. No-op unless a playback page
 *                   is up. Takes the LVGL lock; call from any task.
 *   playback_active: a record is being shown; frames from the store bypass
 *                   the live-stream gate on their way to the panel. */
void ui_gw_make_thumbnail(const uint16_t *rgb565, uint32_t w, uint32_t h,
                          uint16_t *thumb);
void ui_gw_records_changed(void);
void ui_gw_playback_ended(void);
bool ui_gw_playback_active(void);
/* Something landed on the visitor page while the user was not looking at it
 * (a missed call kept, a motion snapshot filed): count it on the status bar's
 * unseen badge. Opening the visitor page clears the count. Any task. */
void ui_gw_note_unseen(void);

void ui_gw_set_capture_cb(ui_gw_capture_cb_t cb);
void ui_gw_set_pir_trigger_cb(ui_gw_pir_trigger_cb_t cb);
void ui_gw_set_low_power_cb(ui_gw_low_power_cb_t cb);
void ui_gw_set_jpeg_quality_cb(ui_gw_jpeg_quality_cb_t cb);
void ui_gw_set_intercom_cb(ui_gw_intercom_cb_t cb);
void ui_gw_set_intercom_active(bool active);
void ui_gw_set_rx_abort_cb(ui_gw_rx_abort_cb_t cb);

/* Update the node's battery voltage display (mV). Call with 0 to hide. */
void ui_gw_update_vbat(uint16_t vbat_mv);

/* Publish the link account (both directions) to the status bar. Called from the
 * radio task on every cleanly completed image frame and on the node's Vbat
 * heartbeat — never on a timer, so the numbers always describe real traffic.
 * Stores and flags only; the LVGL task does the drawing.
 *
 * No frame rate here: FPS is counted where frames are actually put on the panel
 * (see the present path), so the displayed rate can never exceed what a viewer
 * sees. */
void ui_gw_update_link_stats(uint32_t bytes_per_s, int16_t rssi,
                             bool rssi_valid);

/* Stamp "something a person would call an event just happened" (doorbell press,
 * delivered image). Drives the age field, which is deliberately separate from
 * link liveness: hours without a visitor is normal, hours without a heartbeat
 * is a fault, and the two must not read the same on screen. */
void ui_gw_note_event(void);

#ifdef __cplusplus
}
#endif

#include "ui_gateway.h"
#include "app_config.h"
#include "bsp.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_imgfx_scale.h"
#include "esp_heap_caps.h"
#include "visitor_store.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "ui_gw";
static const char *kGwNvs = "ui_gw";

static bool gw_nvs_save_u8(const char *key, uint8_t val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(kGwNvs, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: key=%s err=%s", key, esp_err_to_name(err));
        return false;
    }

    err = nvs_set_u8(h, key, val);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS save failed: key=%s value=%u err=%s",
                 key, val, esp_err_to_name(err));
        return false;
    }
    return true;
}

static uint8_t gw_nvs_load_u8(const char *key, uint8_t def)
{
    nvs_handle_t h;
    uint8_t val = def;
    if (nvs_open(kGwNvs, NVS_READONLY, &h) == ESP_OK) {
        (void)nvs_get_u8(h, key, &val);
        nvs_close(h);
    }
    return val;
}

/* ─── Colors ─── */
#define COL_STATUS_BG   lv_color_hex(0x263831)
#define COL_TITLE_BG    lv_color_hex(0xDFE9E2)
#define COL_BODY_BG     lv_color_hex(0xEEF3EF)
#define COL_BOTTOM_BG   lv_color_hex(0x263831)
#define COL_TEXT_MAIN   lv_color_hex(0x18231F)
#define COL_TEXT_LIGHT  lv_color_hex(0xEAF6EF)
#define COL_GREEN       lv_color_hex(0x2F7D5B)
#define COL_AMBER       lv_color_hex(0xB76A2C)
#define COL_ORANGE      lv_color_hex(0x008CFF)
#define COL_PANEL_BG    lv_color_hex(0xFFFFFF)
#define COL_PANEL_BORDER lv_color_hex(0xC9D8D0)
#define COL_KV_BORDER   lv_color_hex(0xE2EBE6)
#define COL_MUTED       lv_color_hex(0x6A7D75)

/* Battery voltage level colors (bright, for the dark status bar background):
 *   > 3.5V  green, 3.3~3.5V amber, < 3.3V red. */
#define COL_VBAT_GREEN  lv_color_hex(0x4CD98A)
#define COL_VBAT_AMBER  lv_color_hex(0xF2C14E)
#define COL_VBAT_RED    lv_color_hex(0xF25C54)

/* ─── Layout ─── */
/* Two-row status bar (voltages; link throughput / FPS / RSSI / age). 32 keeps
 * the picture at 320 - 32 = 288 rows, a multiple of 16 for the JPEG 4:2:0 MCU
 * grid. Changing this height changes the transmitted image size. */
#define STATUS_H    32
#define STATUS_ROW1_Y (-6)
#define STATUS_ROW2_Y (6)
#define TITLE_H     28
#define BODY_Y      (STATUS_H + TITLE_H)
#define BODY_H      232
#define BOTTOM_Y    292
#define BOTTOM_H    28
#define SCR_W       240
#define SCR_H       320
/* The picture arrives landscape and is rotated 90 degrees with no scaling, so
 * the canvas is the image's dimensions swapped. Derived so they cannot drift:
 * a mismatch silently reads past the decoded buffer. */
#define IMG_W       APP_IMAGE_OUTPUT_HEIGHT   /* 240 */
#define IMG_H       APP_IMAGE_OUTPUT_WIDTH    /* 288 */
/* The image sits directly below the status bar and fills the rest of the panel;
 * these must add up or the picture is either clipped or leaves a dead strip. */
_Static_assert(STATUS_H + IMG_H == SCR_H,
               "status bar height and image height must fill the panel");
#define IMG_Y       STATUS_H

/* ─── State ─── */
static ui_page_t s_page = UI_PAGE_IMAGE;
static ui_gw_capture_cb_t s_capture_cb = NULL;
static SemaphoreHandle_t s_lock = NULL; // points to bsp_lcd's LVGL lock

/* Latest node (camera) battery voltage in mV, 0 = unknown. Shown in status bar right. */
static uint16_t s_node_vbat_mv = 0;
/* Gateway's own supply voltage in mV, 0 = unknown. */
static uint16_t s_gw_vbat_mv = 0;
static lv_timer_t *s_gw_vbat_timer = NULL;

/* Shared layout objects */
static lv_obj_t *s_scr = NULL;
static lv_obj_t *s_status_bar = NULL;
static lv_obj_t *s_status_lbl_l = NULL;
static lv_obj_t *s_status_lbl_r = NULL;
static lv_obj_t *s_status_lbl_l2 = NULL;
static lv_obj_t *s_status_lbl_r2 = NULL;

/* Status bar microphone icon: blank when idle, muted while view-and-listen,
 * live during the call. Drawn into its own canvas (the symbol font has no
 * microphone) inside the status strip, which LVGL keeps flushing while video
 * owns the panel, so it reaches the screen mid-stream. */
typedef enum {
    MIC_ICON_NONE = 0,
    MIC_ICON_MUTED,
    MIC_ICON_LIVE,
} mic_icon_t;
#define MIC_ICON_W  16
#define MIC_ICON_H  20
static lv_obj_t *s_mic_canvas = NULL;
static lv_color_t s_mic_canvas_buf[MIC_ICON_W * MIC_ICON_H];
static mic_icon_t s_mic_icon = MIC_ICON_NONE;

/* Unseen badge: visitor-page items (missed calls, snapshots) since the user
 * last opened it. Producers run on the radio and image tasks; the count is
 * guarded by the LVGL lock where it is drawn. */
static lv_obj_t *s_unseen_badge = NULL;
static int s_unseen_count = 0;

/* Last radio statistics shown in the always-on status bar. */
static uint32_t s_stat_bytes_per_s = 0;
static int16_t s_stat_rssi = 0;
static bool s_stat_rssi_valid = false;

/* Timestamps of frames actually put on the panel, newest last. FPS is counted
 * here, not at radio completion: frames are still dropped afterwards (display
 * gate, decode failure, superseded pending buffer). */
#define STAT_FPS_MARKS      9U      /* 9 marks = 8 intervals */
#define STAT_FPS_WINDOW_MS  10000U
static uint32_t s_stat_disp_ms[STAT_FPS_MARKS];
static uint8_t s_stat_disp_count = 0;
/* Last traffic of any kind from the node (frame or Vbat heartbeat) vs last
 * user-visible event (doorbell / image): no visitor is normal, no heartbeat is
 * a fault. */
static uint32_t s_stat_last_traffic_ms = 0;
static uint32_t s_stat_last_event_ms = 0;
/* Two Vbat heartbeat periods (5 min each) plus margin. */
#define NODE_OFFLINE_MS  (12U * 60U * 1000U)

/* Producers only touch this mailbox under a short critical section. Rendering
 * uses the LVGL-owned s_stat_* snapshot, never live cross-core fields. */
static portMUX_TYPE s_stat_mux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint32_t bytes_per_s;
    int16_t rssi;
    bool rssi_valid;
    uint32_t traffic_ms;
    uint32_t event_ms;
    uint16_t node_vbat_mv;
    bool dirty;
} s_stat_pending;
static lv_timer_t *s_stat_timer = NULL;
#define STAT_RENDER_PERIOD_MS  200
static lv_obj_t *s_title_bar = NULL;
static lv_obj_t *s_title_lbl = NULL;
static lv_obj_t *s_title_chip = NULL;
static lv_obj_t *s_body = NULL;
static lv_obj_t *s_bottom_bar = NULL;
static lv_obj_t *s_bottom_lbl_l = NULL;
static lv_obj_t *s_bottom_lbl_m = NULL;
static lv_obj_t *s_bottom_lbl_r = NULL;

/* PAGE_IMAGE objects. The front buffer may be owned by LCD DMA, pending stays
 * ready for the next present, and back/spare are the free-buffer pool. */
static lv_obj_t *s_img_canvas = NULL;
static lv_color_t *s_img_canvas_buf = NULL;
static lv_color_t *s_img_canvas_back_buf = NULL;
static lv_color_t *s_img_canvas_spare_buf = NULL;
static lv_color_t *s_img_canvas_pending_buf = NULL;
static lv_timer_t *s_img_present_timer = NULL;
static portMUX_TYPE s_img_canvas_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_img_canvas_writing = false;
static bool s_img_buffers_initialized = false;
static uint8_t s_img_canvas_buffer_count = 0;

static lv_obj_t *s_img_placeholder = NULL;
static lv_obj_t *s_img_time_lbl = NULL;
static lv_obj_t *s_img_info_lbl = NULL;
static lv_obj_t *s_img_link_lbl = NULL;
static lv_obj_t *s_img_status_lbl = NULL;
static bool s_has_image = false;

/* Stream mode: the first frame goes through the RX progress page, every later
 * frame refreshes the image canvas in place while app_main auto-requests the
 * next. Cleared when the user leaves the image page. */
static bool s_stream_mode = false;
static bool s_stream_first_shown = false;
/* A visitor session is on screen (bell sounding, ring-time stream if it could
 * start). Its only effect: the capture key answers, dialing the call straight
 * away instead of stepping through listening first. */
static bool s_doorbell_ringing = false;
/* A visitor record is playing back full screen. Its frames use the live
 * stream's decode / present path; this flag lets them through the present gate
 * without switching pages. */
static bool s_playback_mode = false;
static int s_playback_index = -1;

/* PAGE_RX objects */
static lv_obj_t *s_rx_pct_lbl = NULL;
static lv_obj_t *s_rx_bar = NULL;
static lv_obj_t *s_rx_frag_lbl = NULL;
static lv_obj_t *s_rx_rate_lbl = NULL;
static lv_obj_t *s_rx_retry_lbl = NULL;
static lv_obj_t *s_rx_rssi_lbl = NULL;
static uint16_t s_rx_total = 0;
static uint32_t s_rx_start_ms = 0;
static int16_t s_rx_last_rssi = 0;

/* PAGE_VISITORS objects. Thumbnails are canvases over the store's own pixel
 * buffers; age labels are refreshed on a slow timer while the page is up. */
static lv_obj_t *s_visitor_age_lbls[APP_VISITOR_RECORDS] = {NULL};
static lv_obj_t *s_snapshot_age_lbls[APP_VISITOR_SNAPSHOTS] = {NULL};
static lv_timer_t *s_visitor_age_timer = NULL;

/* Transfer stats (updated per transfer) */
static uint16_t s_stats_total_frags = 0;
static uint16_t s_stats_first_missing = 0;
static uint16_t s_stats_total_retransmitted = 0;
static bool s_stats_first_eot_seen = false;

/* PAGE_CONFIG objects, indexed by control id: 0 capture, 3 volume, 5 PIR,
 * 7 low power, 8 JPEG quality; retired ids stay unused. cfg_create_toggle_row
 * writes both arrays, so both must cover the whole id space. */
#define CFG_CTRL_COUNT  9
#define CFG_CTRL_JPEG_QUALITY  8
static lv_obj_t *s_cfg_touch_btns[CFG_CTRL_COUNT] = {NULL};
static lv_obj_t *s_cfg_touch_lbls[CFG_CTRL_COUNT] = {NULL};
/* Volume 0..15 in steps of 8 on the DAC scale, so 15 is the codec maximum.
 * Default 11 (about 0 dB): higher levels feed back into the mic inside the
 * enclosure. */
#define VOLUME_LEVEL_MAX      15
#define VOLUME_LEVEL_STEP     (BSP_AUDIO_VOLUME_MAX / VOLUME_LEVEL_MAX)
#define VOLUME_LEVEL_DEFAULT  11
static int s_volume_level = VOLUME_LEVEL_DEFAULT;
static ui_gw_pir_trigger_cb_t s_pir_trigger_cb = NULL;
static bool s_pir_on = false;
static ui_gw_low_power_cb_t s_low_power_cb = NULL;
static bool s_low_power_on = false;
/* The node's JPEG quality as last accepted by it; the badge cycles MIN..MAX
 * in STEPs and wraps. The node owns the setting (its NVS); this copy only
 * drives the badge and is seeded from gateway NVS. */
static ui_gw_jpeg_quality_cb_t s_jpeg_quality_cb = NULL;
static uint8_t s_jpeg_quality = APP_IMAGE_JPEG_QUALITY;
static ui_gw_intercom_cb_t s_intercom_cb = NULL;
/* Display cache refreshed by ui_gw_set_intercom_active(). May be stale (the
 * node can end a call on its own); both stale readings are harmless because
 * the teardown paths are unconditional. */
static bool s_intercom_active = false;
/* The capture key is stepping the session (listen <-> call). Nothing releases
 * video ownership or rebuilds the page while set, so the new step's frames land
 * on the old picture. Set only by start_capture_action() around radio calls. */
static bool s_switching_in_place = false;
typedef enum {
    INTERCOM_UI_IDLE = 0,
    INTERCOM_UI_CONNECTING,
    INTERCOM_UI_ACTIVE,
    INTERCOM_UI_FAILED,
} intercom_ui_state_t;
static intercom_ui_state_t s_intercom_ui_state = INTERCOM_UI_IDLE;
static ui_gw_rx_abort_cb_t s_rx_abort_cb = NULL;

/* Forward declarations */
static void create_shared_layout(void);
static void show_page(ui_page_t page);
static void create_image_page(void);
static void create_rx_page(void);
static void create_visitors_page(void);
static void create_config_page(void);
static void create_intercom_page(void);
static void destroy_body_children(void);
static void stream_stop(void);
static void stream_end(bool keep_picture);
static void start_capture_action(void);
static void update_title(const char *text, const char *chip, lv_color_t chip_bg);
static lv_color_t vbat_level_color(uint16_t mv);
static void gw_vbat_refresh(void);
static void gw_vbat_timer_cb(lv_timer_t *t);
static void stat_timer_cb(lv_timer_t *t);
static void stat_note_displayed_frame(void);
static void stat_render_rate(void);
static void stat_render_age(void);
static void stat_render_now(void);
static void node_vbat_render(void);
static void image_present_timer_cb(lv_timer_t *t);
static void mic_icon_set(mic_icon_t icon);
static void intercom_end_local(const char *why);
static bool listen_stream_start(bool in_place);
static void unseen_badge_render(void);

static lv_color_t *alloc_image_canvas_buffer(size_t pixels)
{
    const size_t bytes = pixels * sizeof(lv_color_t);
    lv_color_t *buf = heap_caps_aligned_alloc(
        64U, bytes,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    }
    return buf;
}

/* The caller must hold s_img_canvas_mux. */
static void release_image_canvas_buffer(lv_color_t *buf)
{
    if (!buf) return;
    if (!s_img_canvas_back_buf) {
        s_img_canvas_back_buf = buf;
    } else if (!s_img_canvas_spare_buf) {
        s_img_canvas_spare_buf = buf;
    }
}

static void rotate_rgb565_to_canvas(const uint16_t *src, lv_color_t *dst)
{
    for (int out_y = 0; out_y < IMG_H; out_y++) {
        for (int out_x = 0; out_x < IMG_W; out_x++) {
            /* Pure 90-degree rotation, no scaling. */
            uint16_t px = src[(APP_IMAGE_OUTPUT_HEIGHT - 1 - out_x) *
                                  APP_IMAGE_OUTPUT_WIDTH + out_y];
            dst[out_y * IMG_W + out_x].full =
                (uint16_t)(((px & 0x001FU) << 11) |
                           (px & 0x07E0U) |
                           ((px & 0xF800U) >> 11));
        }
    }
}


/* ─── Shared layout ─── */
static void create_shared_layout(void)
{
    s_scr = lv_scr_act();
    lv_obj_clean(s_scr);
    lv_obj_set_style_bg_color(s_scr, COL_BODY_BG, 0);

    /* Status bar */
    s_status_bar = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_status_bar);
    lv_obj_set_size(s_status_bar, SCR_W, STATUS_H);
    lv_obj_set_pos(s_status_bar, 0, 0);
    lv_obj_set_style_bg_color(s_status_bar, COL_STATUS_BG, 0);
    lv_obj_set_style_bg_opa(s_status_bar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_status_lbl_l = lv_label_create(s_status_bar);
    lv_obj_set_style_text_color(s_status_lbl_l, COL_TEXT_LIGHT, 0);
    lv_obj_set_style_text_font(s_status_lbl_l, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_status_lbl_l, "GW --V");
    lv_obj_align(s_status_lbl_l, LV_ALIGN_LEFT_MID, 7, STATUS_ROW1_Y);

    s_status_lbl_r = lv_label_create(s_status_bar);
    lv_obj_set_style_text_color(s_status_lbl_r, COL_TEXT_LIGHT, 0);
    lv_obj_set_style_text_font(s_status_lbl_r, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_status_lbl_r, "NODE --V");
    lv_obj_align(s_status_lbl_r, LV_ALIGN_RIGHT_MID, -7, STATUS_ROW1_Y);

    /* Row 2: the link account. Refreshed only when traffic arrives, never on a
     * timer; the age field on the right is the one clock readout that ticks. */
    s_status_lbl_l2 = lv_label_create(s_status_bar);
    lv_obj_set_style_text_color(s_status_lbl_l2, COL_MUTED, 0);
    lv_obj_set_style_text_font(s_status_lbl_l2, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_status_lbl_l2, "--KB/s --FPS");
    lv_obj_align(s_status_lbl_l2, LV_ALIGN_LEFT_MID, 7, STATUS_ROW2_Y);

    s_status_lbl_r2 = lv_label_create(s_status_bar);
    lv_obj_set_style_text_color(s_status_lbl_r2, COL_MUTED, 0);
    lv_obj_set_style_text_font(s_status_lbl_r2, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_status_lbl_r2, "-- dBm");
    lv_obj_align(s_status_lbl_r2, LV_ALIGN_RIGHT_MID, -7, STATUS_ROW2_Y);

    /* Middle: the microphone icon, spanning both rows. Blank until a session
     * starts; mic_icon_set() paints it. */
    s_mic_canvas = lv_canvas_create(s_status_bar);
    lv_canvas_set_buffer(s_mic_canvas, s_mic_canvas_buf, MIC_ICON_W, MIC_ICON_H,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(s_mic_canvas, LV_ALIGN_CENTER, 0, 0);
    lv_canvas_fill_bg(s_mic_canvas, COL_STATUS_BG, LV_OPA_COVER);
    s_mic_icon = MIC_ICON_NONE;

    /* Unseen badge, right of the mic icon; x=142 clears the row's right label. */
    s_unseen_badge = lv_label_create(s_status_bar);
    lv_obj_set_style_text_font(s_unseen_badge, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_unseen_badge, lv_color_white(), 0);
    lv_obj_set_style_bg_color(s_unseen_badge, COL_VBAT_RED, 0);
    lv_obj_set_style_bg_opa(s_unseen_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_unseen_badge, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_pad_hor(s_unseen_badge, 5, 0);
    lv_obj_set_style_pad_ver(s_unseen_badge, 2, 0);
    lv_obj_set_style_min_width(s_unseen_badge, 16, 0);
    lv_obj_set_style_text_align(s_unseen_badge, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_unseen_badge, "0");
    lv_obj_align(s_unseen_badge, LV_ALIGN_CENTER, 22, STATUS_ROW1_Y);
    unseen_badge_render();

    /* Title bar */
    s_title_bar = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_title_bar);
    lv_obj_set_size(s_title_bar, SCR_W, TITLE_H);
    lv_obj_set_pos(s_title_bar, 0, STATUS_H);
    lv_obj_set_style_bg_color(s_title_bar, COL_TITLE_BG, 0);
    lv_obj_set_style_bg_opa(s_title_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_title_bar, 1, 0);
    lv_obj_set_style_border_color(s_title_bar, lv_color_hex(0xC3D2CA), 0);
    lv_obj_set_style_border_side(s_title_bar, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_clear_flag(s_title_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_title_lbl = lv_label_create(s_title_bar);
    lv_obj_set_style_text_color(s_title_lbl, COL_TEXT_MAIN, 0);
    lv_obj_set_style_text_font(s_title_lbl, &lv_font_montserrat_14, 0);
    lv_label_set_text(s_title_lbl, "");
    lv_obj_align(s_title_lbl, LV_ALIGN_LEFT_MID, 8, 0);

    s_title_chip = lv_label_create(s_title_bar);
    lv_obj_set_style_text_color(s_title_chip, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_title_chip, &lv_font_montserrat_10, 0);
    lv_obj_set_style_bg_color(s_title_chip, COL_GREEN, 0);
    lv_obj_set_style_bg_opa(s_title_chip, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_title_chip, 8, 0);
    lv_obj_set_style_pad_hor(s_title_chip, 6, 0);
    lv_obj_set_style_pad_ver(s_title_chip, 2, 0);
    lv_label_set_text(s_title_chip, "");
    lv_obj_align(s_title_chip, LV_ALIGN_RIGHT_MID, -8, 0);


    /* Body container */
    s_body = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_body);
    lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
    lv_obj_set_pos(s_body, 0, BODY_Y);
    lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);
    lv_obj_set_style_bg_opa(s_body, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_body, 0, 0);

    /* Bottom bar (hidden — all pages use touch now) */
    s_bottom_bar = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_bottom_bar);
    lv_obj_set_size(s_bottom_bar, SCR_W, BOTTOM_H);
    lv_obj_set_pos(s_bottom_bar, 0, BOTTOM_Y);
    lv_obj_set_style_bg_color(s_bottom_bar, COL_BOTTOM_BG, 0);
    lv_obj_set_style_bg_opa(s_bottom_bar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_bottom_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_bottom_bar, LV_OBJ_FLAG_HIDDEN);

    s_bottom_lbl_l = lv_label_create(s_bottom_bar);
    s_bottom_lbl_m = lv_label_create(s_bottom_bar);
    s_bottom_lbl_r = lv_label_create(s_bottom_bar);
}

static void update_title(const char *text, const char *chip, lv_color_t chip_bg)
{
    lv_label_set_text(s_title_lbl, text);
    if (chip && chip[0]) {
        lv_label_set_text(s_title_chip, chip);
        lv_obj_set_style_bg_color(s_title_chip, chip_bg, 0);
        lv_obj_clear_flag(s_title_chip, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_title_chip, LV_OBJ_FLAG_HIDDEN);
    }
}

/* Repaint the status bar microphone icon. Idempotent; caller holds the LVGL
 * lock. */
static void mic_icon_set(mic_icon_t icon)
{
    if (!s_mic_canvas || icon == s_mic_icon) return;
    s_mic_icon = icon;

    /* Painted on the bar's own colour: no alpha channel needed. */
    lv_canvas_fill_bg(s_mic_canvas, COL_STATUS_BG, LV_OPA_COVER);
    if (icon != MIC_ICON_NONE) {
        const lv_color_t col = (icon == MIC_ICON_LIVE) ? COL_VBAT_GREEN
                                                       : COL_TEXT_LIGHT;

        /* Body: a capsule. */
        lv_draw_rect_dsc_t body;
        lv_draw_rect_dsc_init(&body);
        body.bg_color = col;
        body.bg_opa = LV_OPA_COVER;
        body.radius = 3;
        lv_canvas_draw_rect(s_mic_canvas, 5, 1, 6, 11, &body);

        /* Cradle: the lower half of a circle around the body. LVGL angles run
         * clockwise from 3 o'clock, so 0..180 is the bottom half. */
        lv_draw_arc_dsc_t arc;
        lv_draw_arc_dsc_init(&arc);
        arc.color = col;
        arc.width = 2;
        lv_canvas_draw_arc(s_mic_canvas, 8, 8, 6, 0, 180, &arc);

        /* Stem and foot. */
        lv_draw_line_dsc_t line;
        lv_draw_line_dsc_init(&line);
        line.color = col;
        line.width = 2;
        const lv_point_t stem[] = {{8, 14}, {8, 18}};
        lv_canvas_draw_line(s_mic_canvas, stem, 2, &line);
        const lv_point_t foot[] = {{4, 18}, {12, 18}};
        lv_canvas_draw_line(s_mic_canvas, foot, 2, &line);

        if (icon == MIC_ICON_MUTED) {
            /* Struck through, in the red the bar already uses for alarms. */
            line.color = COL_VBAT_RED;
            const lv_point_t slash[] = {{2, 1}, {14, 19}};
            lv_canvas_draw_line(s_mic_canvas, slash, 2, &line);
        }
    }
    lv_obj_invalidate(s_mic_canvas);
}

/* Repaint the unseen badge from s_unseen_count. Caller holds the LVGL lock. */
static void unseen_badge_render(void)
{
    if (!s_unseen_badge) return;
    if (s_unseen_count <= 0) {
        lv_obj_add_flag(s_unseen_badge, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_label_set_text_fmt(s_unseen_badge, "%d", s_unseen_count > 9 ? 9 : s_unseen_count);
    lv_obj_clear_flag(s_unseen_badge, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(s_unseen_badge, LV_ALIGN_CENTER, 22, STATUS_ROW1_Y);
    lv_obj_invalidate(s_unseen_badge);
}

void ui_gw_note_unseen(void)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    /* Nothing to announce when the user is already looking at the list: the
     * rebuild that follows shows the newcomer directly. */
    if (s_page != UI_PAGE_VISITORS) {
        s_unseen_count++;
        unseen_badge_render();
    }
    xSemaphoreGiveRecursive(s_lock);
}

static void destroy_body_children(void)
{
    lv_obj_clean(s_body);

    lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_body, 0, BODY_Y);
    lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
    lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);

    s_img_canvas = NULL;
    s_img_placeholder = NULL;
    s_img_time_lbl = NULL;
    s_img_info_lbl = NULL;
    s_img_link_lbl = NULL;
    s_img_status_lbl = NULL;
    s_rx_pct_lbl = NULL;
    s_rx_bar = NULL;
    s_rx_frag_lbl = NULL;
    s_rx_rate_lbl = NULL;
    s_rx_retry_lbl = NULL;
    s_rx_rssi_lbl = NULL;
    memset(s_visitor_age_lbls, 0, sizeof(s_visitor_age_lbls));
    memset(s_snapshot_age_lbls, 0, sizeof(s_snapshot_age_lbls));
    memset(s_cfg_touch_btns, 0, sizeof(s_cfg_touch_btns));
    memset(s_cfg_touch_lbls, 0, sizeof(s_cfg_touch_lbls));
}


/* ─── Helper: create a kv row ─── */
static lv_obj_t *create_kv_row(lv_obj_t *parent, const char *key, const char *val,
                                lv_obj_t **val_out)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), 22);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, COL_KV_BORDER, 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_pad_hor(row, 8, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *k = lv_label_create(row);
    lv_obj_set_style_text_font(k, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(k, COL_MUTED, 0);
    lv_label_set_text(k, key);

    lv_obj_t *v = lv_label_create(row);
    lv_obj_set_style_text_font(v, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(v, COL_TEXT_MAIN, 0);
    lv_label_set_text(v, val);
    if (val_out) *val_out = v;
    return row;
}

/* ─── PAGE: Image (Home) ─── */
static void create_image_page(void)
{
    const size_t canvas_pixels = IMG_W * IMG_H;

    if (!s_img_buffers_initialized) {
        s_img_canvas_buf = alloc_image_canvas_buffer(canvas_pixels);
        if (s_img_canvas_buf) {
            s_img_canvas_buffer_count = 1;
            s_img_canvas_back_buf = alloc_image_canvas_buffer(canvas_pixels);
            if (s_img_canvas_back_buf) {
                s_img_canvas_buffer_count++;
                s_img_canvas_spare_buf = alloc_image_canvas_buffer(canvas_pixels);
                if (s_img_canvas_spare_buf) {
                    s_img_canvas_buffer_count++;
                }
            }

            memset(s_img_canvas_buf, 0, canvas_pixels * sizeof(lv_color_t));
            if (s_img_canvas_back_buf) {
                memset(s_img_canvas_back_buf, 0,
                       canvas_pixels * sizeof(lv_color_t));
            }
            if (s_img_canvas_spare_buf) {
                memset(s_img_canvas_spare_buf, 0,
                       canvas_pixels * sizeof(lv_color_t));
            }
        }
        s_img_buffers_initialized = s_img_canvas_buf != NULL;
        if (s_img_buffers_initialized) {
            ESP_LOGI(TAG, "image canvas buffers=%u bytes_each=%u",
                     s_img_canvas_buffer_count,
                     (unsigned)(canvas_pixels * sizeof(lv_color_t)));
        }
    }

    if (s_has_image && s_img_canvas_buf) {
        /* The status bar stays; the picture is sized to the rows below it. */
        lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_body, 0, IMG_Y);
        lv_obj_set_size(s_body, SCR_W, IMG_H);
        lv_obj_set_style_bg_color(s_body, lv_color_black(), 0);

        s_img_canvas = lv_canvas_create(s_body);
        lv_canvas_set_buffer(s_img_canvas, s_img_canvas_buf, IMG_W, IMG_H,
                             LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_pos(s_img_canvas, 0, 0);
        if (s_stream_mode) {
            lv_obj_add_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_body, 0, BODY_Y);
        lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
        lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);

        s_img_placeholder = lv_label_create(s_body);
        lv_obj_set_style_text_font(s_img_placeholder, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_img_placeholder, lv_color_hex(0xA0B8AC), 0);
        lv_label_set_text(s_img_placeholder, "Waiting for node...");
        lv_obj_align(s_img_placeholder, LV_ALIGN_CENTER, 0, 0);

        update_title("Latest", "", COL_GREEN);
    }
}


/* ─── PAGE: RX Progress ─── */
static void create_rx_page(void)
{
    /* Percentage */
    s_rx_pct_lbl = lv_label_create(s_body);
    lv_obj_set_style_text_font(s_rx_pct_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(s_rx_pct_lbl, COL_GREEN, 0);
    lv_label_set_text(s_rx_pct_lbl, "0%");
    lv_obj_set_pos(s_rx_pct_lbl, 0, 10);
    lv_obj_set_width(s_rx_pct_lbl, SCR_W);
    lv_obj_set_style_text_align(s_rx_pct_lbl, LV_TEXT_ALIGN_CENTER, 0);

    /* Progress bar */
    s_rx_bar = lv_bar_create(s_body);
    lv_obj_set_size(s_rx_bar, 208, 14);
    lv_obj_set_pos(s_rx_bar, 16, 62);
    lv_bar_set_range(s_rx_bar, 0, 100);
    lv_bar_set_value(s_rx_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_rx_bar, lv_color_hex(0xD9E4DE), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_rx_bar, COL_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_rx_bar, 7, LV_PART_MAIN);
    lv_obj_set_style_radius(s_rx_bar, 7, LV_PART_INDICATOR);

    /* Stats panel */
    lv_obj_t *panel = lv_obj_create(s_body);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 224, 110);
    lv_obj_set_pos(panel, 8, 88);
    lv_obj_set_style_bg_color(panel, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, COL_PANEL_BORDER, 0);
    lv_obj_set_style_radius(panel, 6, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_style_pad_row(panel, 0, 0);

    create_kv_row(panel, "Packets", "0 / 0", &s_rx_frag_lbl);
    create_kv_row(panel, "Rate", "-- kbps", &s_rx_rate_lbl);
    create_kv_row(panel, "Elapsed", "00:00.0", &s_rx_retry_lbl);
    create_kv_row(panel, "RSSI", "-- dBm", &s_rx_rssi_lbl);

    update_title("Receiving", "RX", COL_AMBER);
}

/* ─── PAGE: Visitor records ─── */

static uint32_t uptime_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* "now" / "5m" / "3h" / "2d" from an uptime difference (the gateway has no
 * wall clock). */
static void format_age_short(uint32_t age_ms, char *out, size_t cap)
{
    const uint32_t s = age_ms / 1000U;
    if (s < 60U) {
        snprintf(out, cap, "now");
    } else if (s < 3600U) {
        snprintf(out, cap, "%lum", (unsigned long)(s / 60U));
    } else if (s < 86400U) {
        snprintf(out, cap, "%luh", (unsigned long)(s / 3600U));
    } else {
        snprintf(out, cap, "%lud", (unsigned long)(s / 86400U));
    }
}

/* Caption under a missed-call cell: age and clip length, e.g. "2m . 15s". */
static void format_record_caption(const visitor_record_info_t *info, char *out,
                                  size_t cap)
{
    char age[16];
    format_age_short(uptime_ms() - info->start_uptime_ms, age, sizeof(age));
    snprintf(out, cap, "%s  %lus", age,
             (unsigned long)((info->duration_ms + 500U) / 1000U));
}

static void visitor_refresh_ages(void)
{
    for (int i = 0; i < (int)APP_VISITOR_RECORDS; i++) {
        if (!s_visitor_age_lbls[i]) continue;
        visitor_record_info_t info;
        if (!visitor_store_get(i, &info)) continue;
        char cap[32];
        format_record_caption(&info, cap, sizeof(cap));
        lv_label_set_text(s_visitor_age_lbls[i], cap);
    }
    for (int i = 0; i < (int)APP_VISITOR_SNAPSHOTS; i++) {
        if (!s_snapshot_age_lbls[i]) continue;
        visitor_snapshot_info_t info;
        if (!visitor_store_snapshot_get(i, &info)) continue;
        char age[16];
        format_age_short(uptime_ms() - info.uptime_ms, age, sizeof(age));
        lv_label_set_text(s_snapshot_age_lbls[i], age);
    }
}

static void visitor_age_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (s_page == UI_PAGE_VISITORS) visitor_refresh_ages();
}

static void playback_stop_internal(void);
static void create_playback_page(void);

/* A record row was tapped: play it full screen. */
static void visitor_row_clicked_cb(lv_event_t *e)
{
    const int index = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_stream_mode || s_intercom_active || s_doorbell_ringing) return;
    s_playback_mode = true;
    s_playback_index = index;
    show_page(UI_PAGE_PLAYBACK);
    if (!visitor_store_play(index)) {
        ESP_LOGW(TAG, "visitor record %d could not be played", index);
        playback_stop_internal();
        show_page(UI_PAGE_VISITORS);
        return;
    }
    ESP_LOGI(TAG, "visitor record %d: playback started", index);
}

/* A snapshot cell was tapped: show the still full screen through the playback
 * page and frame path; a tap or key ends it the ordinary way. */
static void snapshot_clicked_cb(lv_event_t *e)
{
    const int index = (int)(intptr_t)lv_event_get_user_data(e);
    if (s_stream_mode || s_intercom_active || s_doorbell_ringing) return;
    s_playback_mode = true;
    s_playback_index = -1;
    show_page(UI_PAGE_PLAYBACK);
    if (!visitor_store_snapshot_show(index)) {
        ESP_LOGW(TAG, "snapshot %d could not be shown", index);
        playback_stop_internal();
        show_page(UI_PAGE_VISITORS);
        return;
    }
    ESP_LOGI(TAG, "snapshot %d: shown", index);
}

/* Section header: uppercase caption with the item count on the right. */
static void visitors_section_header(lv_obj_t *parent, const char *caption, int count,
                                    int cap)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, 224, 16);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(row);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl, COL_MUTED, 0);
    lv_label_set_text(lbl, caption);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 2, 0);

    lv_obj_t *num = lv_label_create(row);
    lv_obj_set_style_text_font(num, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(num, COL_MUTED, 0);
    lv_label_set_text_fmt(num, "%d / %d", count, cap);
    lv_obj_align(num, LV_ALIGN_RIGHT_MID, -2, 0);
}

/* A muted one-liner where a section has nothing to list. */
static void visitors_empty_row(lv_obj_t *parent, const char *text)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, 224, 26);
    lv_obj_set_style_bg_color(row, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, COL_PANEL_BORDER, 0);
    lv_obj_set_style_radius(row, 6, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(row);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl, COL_MUTED, 0);
    lv_label_set_text(lbl, text);
    lv_obj_center(lbl);
}

/* Grid geometry shared by both sections: a cell is the thumbnail with a
 * caption under it, three cells to a 224 px row with 4 px gutters. */
#define VISITOR_CELL_W    72
#define VISITOR_CELL_H    (VISITOR_THUMB_H + 24)
#define VISITOR_CELL_GAP  4
#define VISITOR_COLS      3

/* One grid cell: thumbnail (or placeholder), caption, play mark for a clip.
 * The canvas draws straight from the store's buffer, which stays alive until
 * eviction rebuilds this page. Returns the caption label for the age tick. */
static lv_obj_t *visitors_cell(lv_obj_t *grid, int slot, const uint16_t *thumb,
                               const char *caption, bool is_clip,
                               lv_event_cb_t cb, int index)
{
    lv_obj_t *cell = lv_obj_create(grid);
    lv_obj_remove_style_all(cell);
    lv_obj_set_size(cell, VISITOR_CELL_W, VISITOR_CELL_H);
    lv_obj_set_pos(cell, (slot % VISITOR_COLS) * (VISITOR_CELL_W + VISITOR_CELL_GAP),
                   (slot / VISITOR_COLS) * (VISITOR_CELL_H + VISITOR_CELL_GAP));
    lv_obj_set_style_bg_color(cell, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cell, 1, 0);
    lv_obj_set_style_border_color(cell, COL_PANEL_BORDER, 0);
    lv_obj_set_style_radius(cell, 6, 0);
    lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(cell, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(cell, cb, LV_EVENT_CLICKED, (void *)(intptr_t)index);

    const int tx = (VISITOR_CELL_W - VISITOR_THUMB_W) / 2;
    if (thumb) {
        lv_obj_t *c = lv_canvas_create(cell);
        lv_canvas_set_buffer(c, (void *)thumb, VISITOR_THUMB_W, VISITOR_THUMB_H,
                             LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_pos(c, tx, 5);
        lv_obj_clear_flag(c, LV_OBJ_FLAG_CLICKABLE);
    } else {
        lv_obj_t *ph = lv_obj_create(cell);
        lv_obj_remove_style_all(ph);
        lv_obj_set_size(ph, VISITOR_THUMB_W, VISITOR_THUMB_H);
        lv_obj_set_pos(ph, tx, 5);
        lv_obj_set_style_bg_color(ph, COL_KV_BORDER, 0);
        lv_obj_set_style_bg_opa(ph, LV_OPA_COVER, 0);
        lv_obj_clear_flag(ph, LV_OBJ_FLAG_CLICKABLE);
    }
    if (is_clip) {
        /* Play mark: what tells a clip from a still at a glance. */
        lv_obj_t *play = lv_label_create(cell);
        lv_obj_set_style_text_font(play, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(play, lv_color_white(), 0);
        lv_obj_set_style_bg_color(play, COL_GREEN, 0);
        lv_obj_set_style_bg_opa(play, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(play, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_pad_all(play, 3, 0);
        lv_label_set_text(play, LV_SYMBOL_PLAY);
        lv_obj_set_pos(play, tx + VISITOR_THUMB_W - 18, 5 + VISITOR_THUMB_H - 18);
        lv_obj_clear_flag(play, LV_OBJ_FLAG_CLICKABLE);
    }

    lv_obj_t *lbl = lv_label_create(cell);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl, COL_TEXT_MAIN, 0);
    lv_label_set_text(lbl, caption);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -4);
    return lbl;
}

/* A grid sized for `count` cells, laid out VISITOR_COLS to a row. */
static lv_obj_t *visitors_grid(lv_obj_t *parent, int count)
{
    const int rows = (count + VISITOR_COLS - 1) / VISITOR_COLS;
    lv_obj_t *grid = lv_obj_create(parent);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, 224, rows * VISITOR_CELL_H + (rows - 1) * VISITOR_CELL_GAP);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    return grid;
}

/* Visitor page: MISSED CALLS (short clips with sound, tap to play) and MOTION
 * (PIR stills, tap to view), both grids of the same cells, newest first. */
static void create_visitors_page(void)
{
    const int rec_count = visitor_store_count();
    const int snap_count = visitor_store_snapshot_count();
    update_title("Visitors", (rec_count + snap_count) > 0 ? "" : "NONE", COL_MUTED);

    /* Opening the page is seeing it. */
    s_unseen_count = 0;
    unseen_badge_render();

    lv_obj_add_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(s_body, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_body, LV_SCROLLBAR_MODE_AUTO);

    lv_obj_t *cont = lv_obj_create(s_body);
    lv_obj_remove_style_all(cont);
    lv_obj_set_width(cont, SCR_W);
    lv_obj_set_height(cont, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_hor(cont, 8, 0);
    lv_obj_set_style_pad_top(cont, 6, 0);
    lv_obj_set_style_pad_bottom(cont, 8, 0);
    lv_obj_set_style_pad_row(cont, 4, 0);
    lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    /* ── Missed calls ── */
    visitors_section_header(cont, "MISSED CALLS", rec_count, (int)APP_VISITOR_RECORDS);
    if (rec_count == 0) {
        visitors_empty_row(cont, "No missed calls");
    } else {
        lv_obj_t *grid = visitors_grid(cont, rec_count);
        int slot = 0;
        for (int i = 0; i < (int)APP_VISITOR_RECORDS; i++) {
            visitor_record_info_t info;
            if (!visitor_store_get(i, &info)) continue;
            char cap[32];
            format_record_caption(&info, cap, sizeof(cap));
            s_visitor_age_lbls[i] = visitors_cell(grid, slot++, info.thumb, cap, true,
                                                  visitor_row_clicked_cb, i);
        }
    }

    /* ── Motion snapshots ── */
    visitors_section_header(cont, "MOTION", snap_count, (int)APP_VISITOR_SNAPSHOTS);
    if (snap_count == 0) {
        visitors_empty_row(cont, "No motion snapshots");
        return;
    }
    lv_obj_t *grid = visitors_grid(cont, snap_count);
    int slot = 0;
    for (int i = 0; i < (int)APP_VISITOR_SNAPSHOTS; i++) {
        visitor_snapshot_info_t info;
        if (!visitor_store_snapshot_get(i, &info)) continue;
        char age[16];
        format_age_short(uptime_ms() - info.uptime_ms, age, sizeof(age));
        s_snapshot_age_lbls[i] = visitors_cell(grid, slot++, info.thumb, age, false,
                                               snapshot_clicked_cb, i);
    }
}

/* ─── PAGE: Record playback ─── */

/* Tapping the picture stops the playback and returns to the list. */
static void playback_tap_cb(lv_event_t *e)
{
    (void)e;
    if (!s_playback_mode) return;
    ESP_LOGI(TAG, "playback: stopped by tap");
    playback_stop_internal();
    show_page(UI_PAGE_VISITORS);
}

static void create_playback_page(void)
{
    /* Same geometry as the image page in stream mode. */
    lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_body, 0, IMG_Y);
    lv_obj_set_size(s_body, SCR_W, IMG_H);
    lv_obj_set_style_bg_color(s_body, lv_color_black(), 0);

    lv_obj_t *lbl = lv_label_create(s_body);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl, COL_MUTED, 0);
    lv_label_set_text(lbl, "Playing...");
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);

    /* Transparent touch target as a child of s_body, so it leaves with the
     * page. */
    lv_obj_t *tap = lv_obj_create(s_body);
    lv_obj_remove_style_all(tap);
    lv_obj_set_size(tap, SCR_W, IMG_H);
    lv_obj_set_pos(tap, 0, 0);
    lv_obj_clear_flag(tap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(tap, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(tap, playback_tap_cb, LV_EVENT_CLICKED, NULL);
}

/* End a playback from the UI side. Safe when nothing is playing. */
static void playback_stop_internal(void)
{
    if (!s_playback_mode) return;
    s_playback_mode = false;
    s_playback_index = -1;
    visitor_store_stop();
    bsp_lcd_set_video_direct_owner(false);
}

static void cfg_style_value(int idx, const char *text);

/* Open the view-and-listen stream. Returns whether the request went out.
 * From nothing: RX page, then the first frame flips to the image page.
 * in_place (stepping back from the call): the call's last frame stays until
 * the stream's first replaces it; s_stream_first_shown is set up front to keep
 * ui_gw_rx_begin() off the RX page. On failure the image page is rebuilt. */
static bool listen_stream_start(bool in_place)
{
    if (!s_capture_cb) return false;
    s_stream_mode = true;
    s_stream_first_shown = in_place;
    if (!in_place) {
        show_page(UI_PAGE_RX);
    }
    const bool ok = s_capture_cb();
    if (ok) {
        if (!in_place) update_title("Waiting...", "RX", COL_AMBER);
        mic_icon_set(MIC_ICON_MUTED);
    } else {
        s_stream_mode = false;
        s_stream_first_shown = false;
        show_page(UI_PAGE_IMAGE);
        mic_icon_set(MIC_ICON_NONE);
    }
    return ok;
}

/* End the call from this side. Unconditional: s_intercom_active can be stale
 * and set_intercom(false) is a no-op when the radio is already idle. */
static void intercom_end_local(const char *why)
{
    /* Read before the callback: the radio reports the end through
     * ui_gw_set_intercom_active(false) from inside it, on this task. */
    const bool was_active = s_intercom_active;
    if (s_intercom_cb) (void)s_intercom_cb(0);
    if (was_active) {
        ESP_LOGI(TAG, "intercom stopped: %s", why);
    }
    s_intercom_active = false;
    s_intercom_ui_state = INTERCOM_UI_IDLE;
    mic_icon_set(MIC_ICON_NONE);
}

/* The capture key runs the whole session:
 *   nothing up    -> view and listen      listening     -> two-way call
 *   in the call   -> back to listening    bell sounding -> answer (call)
 * Leaving the session is the page keys' job (K1/K2/K4). */
static void start_capture_action(void)
{
    /* Hang up, keep watching. The call is torn down before the stream request
     * so the radio is free; s_switching_in_place keeps the picture up. */
    if (s_intercom_active && !s_doorbell_ringing) {
        ESP_LOGI(TAG, "capture action: call -> listen");
        s_switching_in_place = true;
        intercom_end_local("back to listening");
        (void)listen_stream_start(true);
        s_switching_in_place = false;
        return;
    }

    /* Dial, or answer a ring. stream_end(true) only drops the UI flags; the
     * dial runs the stream out at a frame boundary before its CONFIG goes on
     * air (which also closes a visitor session). Only a failed dial changes
     * the page. */
    const bool answering = s_doorbell_ringing;
    if (answering || s_stream_mode) {
        ESP_LOGI(TAG, "capture action: %s cached=%d",
                 answering ? "answering the door" : "listen -> call",
                 s_intercom_active ? 1 : 0);
        s_switching_in_place = true;
        stream_end(true);
        intercom_end_local("redial");

        s_intercom_ui_state = INTERCOM_UI_CONNECTING;
        bool started = s_intercom_cb && s_intercom_cb(1);
        s_switching_in_place = false;
        s_intercom_active = started;
        s_intercom_ui_state = started ? INTERCOM_UI_ACTIVE : INTERCOM_UI_FAILED;
        mic_icon_set(started ? MIC_ICON_LIVE : MIC_ICON_NONE);
        if (!started) {
            show_page(UI_PAGE_INTERCOM);
        }
        return;
    }

    /* First step. */
    ESP_LOGI(TAG, "capture action: view and listen");
    (void)listen_stream_start(false);
}

/* ─── Touch button clicked callback (value badges only) ─── */
static void cfg_btn_clicked_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);

    switch (idx) {
    case 0: /* Capture */
        start_capture_action();
        break;
    case 3: /* Volume +1 */
        s_volume_level = (s_volume_level + 1) % (VOLUME_LEVEL_MAX + 1);
        bsp_audio_set_volume((uint8_t)(s_volume_level * VOLUME_LEVEL_STEP));
        {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", s_volume_level);
            cfg_style_value(3, buf);
        }
        gw_nvs_save_u8("vol", (uint8_t)s_volume_level);
        break;
    case CFG_CTRL_JPEG_QUALITY: { /* next step, wrapping to MIN */
        unsigned next = (unsigned)s_jpeg_quality + APP_IMAGE_JPEG_QUALITY_STEP;
        if (next > APP_IMAGE_JPEG_QUALITY_MAX) next = APP_IMAGE_JPEG_QUALITY_MIN;
        /* Like the switches: the badge only moves once the node has ACKed.
         * A failed send leaves it where it was, which is the feedback. */
        if (s_jpeg_quality_cb && s_jpeg_quality_cb((uint32_t)next)) {
            s_jpeg_quality = (uint8_t)next;
            char buf[8];
            snprintf(buf, sizeof(buf), "%u", (unsigned)s_jpeg_quality);
            cfg_style_value(CFG_CTRL_JPEG_QUALITY, buf);
            gw_nvs_save_u8("jpegq", s_jpeg_quality);
        } else {
            ESP_LOGW(TAG, "JPEG quality %u not accepted by the node", next);
        }
        break;
    }
    }
}

/* ─── PAGE: Config (phone-settings style) ─── */

/* Helper: create a section card container */
static lv_obj_t *cfg_create_section(lv_obj_t *parent, const char *title)
{
    lv_obj_t *sec = lv_obj_create(parent);
    lv_obj_remove_style_all(sec);
    lv_obj_set_width(sec, 224);
    lv_obj_set_height(sec, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(sec, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(sec, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sec, 1, 0);
    lv_obj_set_style_border_color(sec, COL_PANEL_BORDER, 0);
    lv_obj_set_style_radius(sec, 8, 0);
    lv_obj_set_style_pad_top(sec, 4, 0);
    lv_obj_set_style_pad_bottom(sec, 0, 0);
    lv_obj_set_style_pad_hor(sec, 0, 0);
    lv_obj_set_layout(sec, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(sec, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(sec, 0, 0);
    lv_obj_clear_flag(sec, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(sec);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl, COL_MUTED, 0);
    lv_obj_set_style_pad_left(lbl, 10, 0);
    lv_label_set_text(lbl, title);

    return sec;
}

/* Helper: create a setting row with label, description, and a VALUE badge (tap to cycle) */
static lv_obj_t *cfg_create_row(lv_obj_t *section, const char *label, const char *desc,
                                 int btn_idx, lv_obj_t **val_out)
{
    lv_obj_t *row = lv_obj_create(section);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, 224);
    lv_obj_set_height(row, desc ? 36 : 30);
    lv_obj_set_style_pad_hor(row, 10, 0);
    lv_obj_set_style_pad_ver(row, 4, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(0xF0F5F2), 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name_lbl = lv_label_create(row);
    lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(name_lbl, COL_TEXT_MAIN, 0);
    lv_label_set_text(name_lbl, label);
    lv_obj_set_pos(name_lbl, 0, desc ? 2 : 5);

    if (desc) {
        lv_obj_t *desc_lbl = lv_label_create(row);
        lv_obj_set_style_text_font(desc_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(desc_lbl, COL_MUTED, 0);
        lv_label_set_text(desc_lbl, desc);
        lv_obj_set_pos(desc_lbl, 0, 18);
    }

    /* Right-side value badge (tap to cycle) */
    lv_obj_t *val_btn = lv_btn_create(row);
    lv_obj_set_size(val_btn, LV_SIZE_CONTENT, 26);
    lv_obj_set_style_radius(val_btn, 13, 0);
    lv_obj_set_style_pad_hor(val_btn, 12, 0);
    lv_obj_set_style_pad_ver(val_btn, 4, 0);
    lv_obj_set_style_border_width(val_btn, 0, 0);
    lv_obj_set_style_min_width(val_btn, 52, 0);
    lv_obj_align(val_btn, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *val_lbl = lv_label_create(val_btn);
    lv_obj_set_style_text_font(val_lbl, &lv_font_montserrat_10, 0);
    lv_obj_center(val_lbl);

    if (btn_idx >= 0) {
        lv_obj_add_event_cb(val_btn, cfg_btn_clicked_cb, LV_EVENT_CLICKED, (void *)(intptr_t)btn_idx);
        s_cfg_touch_btns[btn_idx] = val_btn;
        s_cfg_touch_lbls[btn_idx] = val_lbl;
    }
    if (val_out) *val_out = val_lbl;

    return row;
}

/* Helper: create a setting row with a TOGGLE SWITCH on the right */
static void cfg_switch_cb(lv_event_t *e);

static lv_obj_t *cfg_create_toggle_row(lv_obj_t *section, const char *label, const char *desc,
                                        int btn_idx, bool initial_state)
{
    lv_obj_t *row = lv_obj_create(section);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, 224);
    lv_obj_set_height(row, desc ? 36 : 30);
    lv_obj_set_style_pad_hor(row, 10, 0);
    lv_obj_set_style_pad_ver(row, 4, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(0xF0F5F2), 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name_lbl = lv_label_create(row);
    lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(name_lbl, COL_TEXT_MAIN, 0);
    lv_label_set_text(name_lbl, label);
    lv_obj_set_pos(name_lbl, 0, desc ? 2 : 5);

    if (desc) {
        lv_obj_t *desc_lbl = lv_label_create(row);
        lv_obj_set_style_text_font(desc_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(desc_lbl, COL_MUTED, 0);
        lv_label_set_text(desc_lbl, desc);
        lv_obj_set_pos(desc_lbl, 0, 18);
    }

    /* Toggle switch */
    lv_obj_t *sw = lv_switch_create(row);
    lv_obj_set_size(sw, 40, 22);
    lv_obj_align(sw, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, COL_AMBER, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, lv_color_white(), LV_PART_KNOB);

    if (initial_state) {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
    }

    lv_obj_add_event_cb(sw, cfg_switch_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)btn_idx);
    s_cfg_touch_btns[btn_idx] = sw;
    s_cfg_touch_lbls[btn_idx] = NULL;

    return row;
}

/* Helper: style a value badge (green background) */
static void cfg_style_value(int idx, const char *text)
{
    if (!s_cfg_touch_btns[idx] || !s_cfg_touch_lbls[idx]) return;
    lv_obj_set_style_bg_color(s_cfg_touch_btns[idx], lv_color_hex(0xE8F5EE), 0);
    lv_obj_set_style_bg_opa(s_cfg_touch_btns[idx], LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(s_cfg_touch_lbls[idx], COL_GREEN, 0);
    lv_label_set_text(s_cfg_touch_lbls[idx], text);
}

/* Switch toggle callback — handles all on/off toggles */
static void cfg_switch_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t *sw = lv_event_get_target(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    switch (idx) {
    case 5: /* PIR */
        if (s_pir_trigger_cb) {
            if (s_pir_trigger_cb(on ? 1 : 0)) {
                s_pir_on = on;
                gw_nvs_save_u8("pir", on ? 1 : 0);
            } else {
                if (on) lv_obj_clear_state(sw, LV_STATE_CHECKED);
                else lv_obj_add_state(sw, LV_STATE_CHECKED);
            }
        }
        break;
    case 7: /* Low power */
        if (s_low_power_cb) {
            if (s_low_power_cb(on ? 1 : 0)) {
                s_low_power_on = on;
                gw_nvs_save_u8("lowpwr", on ? 1 : 0);
            } else {
                if (on) lv_obj_clear_state(sw, LV_STATE_CHECKED);
                else lv_obj_add_state(sw, LV_STATE_CHECKED);
            }
        }
        break;
    }
}

static void create_config_page(void)
{
    /* Make body scrollable for this page */
    lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
    lv_obj_add_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_body, 0, 0);

    /* Scroll container */
    lv_obj_t *cont = lv_obj_create(s_body);
    lv_obj_remove_style_all(cont);
    lv_obj_set_width(cont, SCR_W);
    lv_obj_set_height(cont, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(cont, 8, 0);
    lv_obj_set_style_pad_row(cont, 8, 0);
    lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    /* ── CAPTURE button ── */
    lv_obj_t *cap_btn = lv_btn_create(cont);
    lv_obj_set_size(cap_btn, 224, 36);
    lv_obj_set_style_radius(cap_btn, 8, 0);
    lv_obj_set_style_bg_color(cap_btn, COL_GREEN, 0);
    lv_obj_set_style_bg_opa(cap_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cap_btn, 0, 0);
    lv_obj_add_event_cb(cap_btn, cfg_btn_clicked_cb, LV_EVENT_CLICKED, (void *)(intptr_t)0);
    lv_obj_t *cap_lbl = lv_label_create(cap_btn);
    lv_obj_set_style_text_font(cap_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(cap_lbl, lv_color_white(), 0);
    lv_label_set_text(cap_lbl, "CAPTURE");
    lv_obj_center(cap_lbl);
    s_cfg_touch_btns[0] = cap_btn;
    s_cfg_touch_lbls[0] = cap_lbl;

    /* TRIGGER section: the PIR switch is the node's one trigger setting. */
    lv_obj_t *sec_trig = cfg_create_section(cont, "TRIGGER");

    cfg_create_toggle_row(sec_trig, "PIR Motion", "Snapshot to Visitors", 5, s_pir_on);

    /* ── AUDIO section ── */
    lv_obj_t *sec_audio = cfg_create_section(cont, "AUDIO");

    char vol_buf[8];
    snprintf(vol_buf, sizeof(vol_buf), "%d", s_volume_level);
    cfg_create_row(sec_audio, "Volume", NULL, 3, NULL);
    cfg_style_value(3, vol_buf);

    /* ── IMAGE section: the node's encoder, sent as CONFIG ── */
    lv_obj_t *sec_img = cfg_create_section(cont, "IMAGE");

    char q_buf[8];
    char q_desc[32];
    snprintf(q_buf, sizeof(q_buf), "%u", (unsigned)s_jpeg_quality);
    snprintf(q_desc, sizeof(q_desc), "Door camera, %d-%d",
             APP_IMAGE_JPEG_QUALITY_MIN, APP_IMAGE_JPEG_QUALITY_MAX);
    cfg_create_row(sec_img, "JPEG Quality", q_desc, CFG_CTRL_JPEG_QUALITY, NULL);
    cfg_style_value(CFG_CTRL_JPEG_QUALITY, q_buf);

    /* ── SYSTEM section ── */
    lv_obj_t *sec_sys = cfg_create_section(cont, "SYSTEM");

    cfg_create_toggle_row(sec_sys, "Low Power", "CAD sleep standby", 7, s_low_power_on);

    update_title("Settings", "CFG", COL_GREEN);
}

static void create_intercom_page(void)
{
    /* Seen only around the call: dialling, dial failed, node hung up. */
    const char *headline = "CALL ENDED";
    const char *hint = "Press CAPTURE to view";
    const char *chip = "IDLE";
    lv_color_t color = COL_MUTED;

    if (s_intercom_ui_state == INTERCOM_UI_CONNECTING) {
        headline = "CONNECTING...";
        hint = "Starting two-way voice";
        chip = "WAIT";
        color = COL_AMBER;
    } else if (s_intercom_ui_state == INTERCOM_UI_ACTIVE) {
        headline = "INTERCOM ACTIVE";
        hint = "Two-way voice in progress";
        chip = "LIVE";
        color = COL_GREEN;
    } else if (s_intercom_ui_state == INTERCOM_UI_FAILED) {
        headline = "CONNECTION FAILED";
        hint = "CAPTURE to view, again to call";
        chip = "FAIL";
        color = lv_color_hex(0xB53A32);
    }

    lv_obj_t *panel = lv_obj_create(s_body);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 224, 176);
    lv_obj_set_pos(panel, 8, 24);
    lv_obj_set_style_bg_color(panel, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_border_color(panel, color, 0);
    lv_obj_set_style_radius(panel, 12, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *state = lv_label_create(panel);
    lv_obj_set_style_text_font(state, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(state, color, 0);
    lv_label_set_text(state, headline);
    lv_obj_align(state, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *desc = lv_label_create(panel);
    lv_obj_set_style_text_font(desc, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(desc, COL_MUTED, 0);
    lv_label_set_text(desc, hint);
    lv_obj_align(desc, LV_ALIGN_CENTER, 0, 22);

    update_title("Intercom", chip, color);
}

/* ─── Page switch ─── */
static void show_page(ui_page_t page)
{
    /* Catch-all release of video ownership; the present path re-takes it
     * before each frame. */
    bsp_lcd_set_video_direct_owner(false);
    destroy_body_children();
    s_page = page;

    switch (page) {
    case UI_PAGE_IMAGE:  create_image_page();  break;
    case UI_PAGE_RX:     create_rx_page();     break;
    case UI_PAGE_VISITORS: create_visitors_page(); break;
    case UI_PAGE_CONFIG: create_config_page(); break;
    case UI_PAGE_INTERCOM: create_intercom_page(); break;
    case UI_PAGE_PLAYBACK: create_playback_page(); break;
    default: break;
    }
}

/* End the video stream. Safe when no stream is active.
 *   keep_picture false: abort the in-flight RX, hand the panel back to LVGL.
 *   keep_picture true (listen -> call): only the UI flags change; the dial
 *     that follows quiesces the stream itself and the panel is left as is
 *     for the call's frames to land on. */
static void stream_end(bool keep_picture)
{
    bool was_streaming = s_stream_mode;
    /* A visitor session is closed through the same abort even when its
     * picture never started; abort_image_rx() is what ends the session. */
    const bool was_ringing = s_doorbell_ringing;
    s_doorbell_ringing = false;
    s_stream_mode = false;
    s_stream_first_shown = false;
    if (keep_picture) return;
    if ((was_streaming || was_ringing) && s_rx_abort_cb) {
        s_rx_abort_cb();
    }
    mic_icon_set(MIC_ICON_NONE);
    /* The panel still holds the last frame; release ownership and force one
     * full LVGL redraw. */
    bsp_lcd_set_video_direct_owner(false);
    if (was_streaming && s_scr) {
        lv_obj_invalidate(s_scr);
    }
}

static void stream_stop(void)
{
    stream_end(false);
}

bool ui_gw_stream_active(void)
{
    return s_stream_mode;
}

/* ─── Doorbell visitor session ─── */
bool ui_gw_doorbell_preempt(void)
{
    if (!s_lock) return false;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    /* A playback holds the panel and speaker; the ring pre-empts it like a
     * live stream. */
    const bool was_playing = s_playback_mode;
    if (was_playing) {
        ESP_LOGI(TAG, "doorbell: pre-empting the record playback");
        playback_stop_internal();
    }
    const bool was_streaming = s_stream_mode;
    if (was_streaming) {
        ESP_LOGI(TAG, "doorbell: pre-empting the running stream");
        stream_stop();
    }
    xSemaphoreGiveRecursive(s_lock);
    return was_streaming || was_playing;
}

bool ui_gw_doorbell_stream_begin(void)
{
    if (!s_lock || !s_capture_cb) return false;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    /* The session is on screen whether or not the picture comes. A ring
     * never dials by itself; only the user's answer does. */
    s_doorbell_ringing = true;
    s_stream_mode = true;
    s_stream_first_shown = false;
    show_page(UI_PAGE_RX);
    const bool ok = s_capture_cb();
    if (ok) {
        update_title("Doorbell", "RX", COL_AMBER);
    } else {
        s_stream_mode = false;
        show_page(UI_PAGE_IMAGE);
    }

    xSemaphoreGiveRecursive(s_lock);
    return ok;
}

void ui_gw_doorbell_ring_end(void)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    /* The deadline and the answer key can cross; the flag, not the caller,
     * decides. stream_stop() after an answer would pull the panel from under
     * the call's frames. */
    if (s_doorbell_ringing) {
        stream_stop();
        /* Rebuilding the image page puts the last frame back on screen. */
        show_page(UI_PAGE_IMAGE);
    }
    xSemaphoreGiveRecursive(s_lock);
}

/* Navigating away from the call ends it: nothing the user cannot see should
 * stay on air. Safe to call from any page. */
static void intercom_stop_for_navigation(void)
{
    if (!s_intercom_active) return;
    intercom_end_local("navigated away from the call");
}

/* ─── Swipe gesture ─── */
static const ui_page_t s_swipe_order[] = {UI_PAGE_IMAGE, UI_PAGE_VISITORS, UI_PAGE_CONFIG};
#define SWIPE_PAGE_COUNT 3

static void gesture_cb(lv_event_t *e)
{
    if (s_page == UI_PAGE_RX) return;

    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT) return;

    int cur = -1;
    for (int i = 0; i < SWIPE_PAGE_COUNT; i++) {
        if (s_swipe_order[i] == s_page) { cur = i; break; }
    }
    if (cur < 0) return;

    int next;
    if (dir == LV_DIR_LEFT) {
        next = (cur + 1) % SWIPE_PAGE_COUNT;
    } else {
        next = (cur + SWIPE_PAGE_COUNT - 1) % SWIPE_PAGE_COUNT;
    }

    /* Swiping off the image page ends the video stream. */
    if (s_swipe_order[next] != UI_PAGE_IMAGE) {
        stream_stop();
    }
    show_page(s_swipe_order[next]);
}

/* ─── Public API ─── */
esp_err_t ui_gw_init(void)
{
    s_lock = bsp_lcd_get_lvgl_lock();
    if (!s_lock) {
        ESP_LOGE(TAG, "LVGL lock not available");
        return ESP_ERR_INVALID_STATE;
    }

    s_volume_level = gw_nvs_load_u8("vol", VOLUME_LEVEL_DEFAULT);
    if (s_volume_level > VOLUME_LEVEL_MAX) s_volume_level = VOLUME_LEVEL_MAX;
    s_pir_on = gw_nvs_load_u8("pir", 0) != 0;
    s_low_power_on = gw_nvs_load_u8("lowpwr", 0) != 0;
    s_jpeg_quality = gw_nvs_load_u8("jpegq", APP_IMAGE_JPEG_QUALITY);
    if (s_jpeg_quality < APP_IMAGE_JPEG_QUALITY_MIN) s_jpeg_quality = APP_IMAGE_JPEG_QUALITY_MIN;
    if (s_jpeg_quality > APP_IMAGE_JPEG_QUALITY_MAX) s_jpeg_quality = APP_IMAGE_JPEG_QUALITY_MAX;
    /* Calls start only from the capture key and are never restored from NVS. */
    s_intercom_active = false;
    s_intercom_ui_state = INTERCOM_UI_IDLE;
    bsp_audio_set_volume((uint8_t)(s_volume_level * VOLUME_LEVEL_STEP));
    ESP_LOGI(TAG, "NVS load: vol=%d pir=%d lowpwr=%d jpegq=%u",
             s_volume_level, s_pir_on, s_low_power_on, (unsigned)s_jpeg_quality);

    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    create_shared_layout();
    lv_obj_add_event_cb(s_scr, gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(s_scr, LV_OBJ_FLAG_GESTURE_BUBBLE);
    show_page(UI_PAGE_IMAGE);
    if (!s_img_present_timer) {
        s_img_present_timer = lv_timer_create(image_present_timer_cb, 1, NULL);
    }
    /* Slow tick for the visitor list's "N min ago" captions. */
    if (!s_visitor_age_timer) {
        s_visitor_age_timer = lv_timer_create(visitor_age_timer_cb, 30000, NULL);
    }

    /* Force one fresh read so the bar is not blank until the background
     * sampler fires; afterwards the timer re-reads the cache. */
    (void)bsp_vbat_read_mv();
    gw_vbat_refresh();
    s_gw_vbat_timer = lv_timer_create(gw_vbat_timer_cb, 60000, NULL);
    /* Idle events only. Streaming status is drawn at frame completion. */
    if (!s_stat_timer) {
        s_stat_timer = lv_timer_create(stat_timer_cb, STAT_RENDER_PERIOD_MS, NULL);
    }

    xSemaphoreGiveRecursive(s_lock);

    ESP_LOGI(TAG, "Gateway UI initialized");
    return ESP_OK;
}

void ui_gw_set_capture_cb(ui_gw_capture_cb_t cb)
{
    s_capture_cb = cb;
}

void ui_gw_set_pir_trigger_cb(ui_gw_pir_trigger_cb_t cb)
{
    s_pir_trigger_cb = cb;
}

void ui_gw_set_low_power_cb(ui_gw_low_power_cb_t cb)
{
    s_low_power_cb = cb;
}

void ui_gw_set_jpeg_quality_cb(ui_gw_jpeg_quality_cb_t cb)
{
    s_jpeg_quality_cb = cb;
}

void ui_gw_set_intercom_cb(ui_gw_intercom_cb_t cb)
{
    s_intercom_cb = cb;
}

void ui_gw_set_intercom_active(bool active)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    s_intercom_active = active;
    s_intercom_ui_state = active ? INTERCOM_UI_ACTIVE : INTERCOM_UI_IDLE;
    mic_icon_set(active ? MIC_ICON_LIVE : MIC_ICON_NONE);
    ESP_LOGI(TAG, "intercom session UI: active=%d", active ? 1 : 0);
    /* Mid-step the key handler owns the screen; only the state above is
     * taken here. */
    if (s_switching_in_place) {
        xSemaphoreGiveRecursive(s_lock);
        return;
    }
    if (s_page == UI_PAGE_INTERCOM) {
        show_page(UI_PAGE_INTERCOM);
    } else if (!active && s_page == UI_PAGE_IMAGE) {
        /* The canvas was hidden for direct presents; rebuilding the page
         * shows the last frame again. */
        show_page(UI_PAGE_IMAGE);
    } else if (!active) {
        /* Hand the panel back to LVGL and repaint over the last frame. */
        bsp_lcd_set_video_direct_owner(false);
        if (s_scr) {
            lv_obj_invalidate(s_scr);
        }
    }

    xSemaphoreGiveRecursive(s_lock);
}

/* Run key actions on the LVGL task, which owns the UI lock and has enough stack
 * for call setup. */
static void key_action_cb(void *arg)
{
    const bsp_btn_id_t key = (bsp_btn_id_t)(intptr_t)arg;
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    /* Any key during a playback ends it; the capture key stops there, the
     * navigation keys go on to their pages. */
    if (s_playback_mode) {
        playback_stop_internal();
        if (key == BSP_BTN_VOL_DN) {
            show_page(UI_PAGE_VISITORS);
            xSemaphoreGiveRecursive(s_lock);
            return;
        }
    }

    if (key == BSP_BTN_VOL_DN) {
        /* K5 = Session; see start_capture_action(). */
        start_capture_action();
    } else if (key == BSP_BTN_VOL_UP) {
        /* K4 = Visitor records */
        stream_stop();
        intercom_stop_for_navigation();
        show_page(UI_PAGE_VISITORS);
    } else if (key == BSP_BTN_USER1) {
        /* K2 = Config page */
        stream_stop();
        intercom_stop_for_navigation();
        show_page(UI_PAGE_CONFIG);
    } else if (key == BSP_BTN_PTT) {
        /* K1 = Image page */
        stream_stop();
        intercom_stop_for_navigation();
        show_page(UI_PAGE_IMAGE);
    }

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_key_event(bsp_btn_id_t key, bool pressed)
{
    if (!pressed) return;
    if (!s_lock) return;

    /* Hand the key to the LVGL task; lv_async_call needs the LVGL lock. */
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    if (lv_async_call(key_action_cb, (void *)(intptr_t)key) != LV_RES_OK) {
        ESP_LOGW(TAG, "key %d dropped: no memory for the deferred action", key);
    }
    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_begin(uint16_t session_id, uint16_t total_frags)
{
    if (!s_lock) return;
    // Non-blocking: called from the radio RX hot path, blocking here would
    // delay the ready-ACK. The next progress call catches up.
    if (xSemaphoreTakeRecursive(s_lock, 0) != pdTRUE) {
        return;
    }

    s_rx_total = total_frags;
    s_rx_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_rx_last_rssi = 0;

    s_stats_total_frags = total_frags;
    s_stats_first_missing = 0;
    s_stats_total_retransmitted = 0;
    s_stats_first_eot_seen = false;

    // Stream mode after the first frame: keep the live image on screen.
    if ((s_stream_mode || s_intercom_active) && s_stream_first_shown) {
        xSemaphoreGiveRecursive(s_lock);
        return;
    }

    if (s_page != UI_PAGE_RX) {
        show_page(UI_PAGE_RX);
    }

    char title[32];
    snprintf(title, sizeof(title), "Receiving #%03u", session_id);
    update_title(title, "RX", COL_AMBER);

    // Show total frag count and force flush this label only
    if (s_rx_frag_lbl) {
        char buf[32];
        snprintf(buf, sizeof(buf), "0 / %u", total_frags);
        lv_label_set_text(s_rx_frag_lbl, buf);
        lv_obj_invalidate(s_rx_frag_lbl);
        lv_refr_now(NULL);
    }

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_progress(uint16_t received, uint16_t total, int16_t rssi)
{
    if (!s_lock || s_page != UI_PAGE_RX) return;
    // Non-blocking: called from the radio RX path, blocking here can delay
    // NACK sends. The next fragment retries and ui_gw_rx_done shows the end.
    if (xSemaphoreTakeRecursive(s_lock, 0) != pdTRUE) {
        return;
    }

    s_rx_last_rssi = rssi;
    uint32_t pct = total > 0 ? (uint32_t)received * 100 / total : 0;

    char buf[32];

    if (s_rx_rssi_lbl) {
        snprintf(buf, sizeof(buf), "%d dBm", rssi);
        lv_label_set_text(s_rx_rssi_lbl, buf);
    }
    snprintf(buf, sizeof(buf), "%lu%%", (unsigned long)pct);
    if (s_rx_pct_lbl) lv_label_set_text(s_rx_pct_lbl, buf);
    if (s_rx_bar) lv_bar_set_value(s_rx_bar, (int32_t)pct, LV_ANIM_OFF);

    snprintf(buf, sizeof(buf), "%u / %u", received, total);
    if (s_rx_frag_lbl) lv_label_set_text(s_rx_frag_lbl, buf);

    uint32_t elapsed_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_rx_start_ms;
    if (elapsed_ms > 0 && received > 0) {
        uint32_t bytes = (uint32_t)received * APP_IMAGE_FRAGMENT_DATA_SIZE;
        uint32_t rate_kbps = (uint32_t)((uint64_t)bytes * 8000 / elapsed_ms / 1000);
        snprintf(buf, sizeof(buf), "%lu kbps", (unsigned long)rate_kbps);
        if (s_rx_rate_lbl) lv_label_set_text(s_rx_rate_lbl, buf);
    }

    uint32_t secs = elapsed_ms / 1000;
    uint32_t tenths = (elapsed_ms % 1000) / 100;
    snprintf(buf, sizeof(buf), "%02lu:%02lu.%lu",
             (unsigned long)(secs / 60), (unsigned long)(secs % 60),
             (unsigned long)tenths);
    if (s_rx_retry_lbl) lv_label_set_text(s_rx_retry_lbl, buf);

    xSemaphoreGiveRecursive(s_lock);
}


static void image_present_timer_cb(lv_timer_t *t)
{
    (void)t;

    lv_color_t *pending = NULL;

    portENTER_CRITICAL(&s_img_canvas_mux);
    if (s_img_canvas_pending_buf) {
        pending = s_img_canvas_pending_buf;
        s_img_canvas_pending_buf = NULL;
    }
    portEXIT_CRITICAL(&s_img_canvas_mux);

    if (!pending) return;

    s_has_image = true;

    /* Live, in-call, and recorded frames share the same presentation path. */
    if (!s_stream_mode && !s_intercom_active && !s_playback_mode) {
        /* Single shot into the LVGL canvas: LVGL owns the panel again. */
        bsp_lcd_set_video_direct_owner(false);
        portENTER_CRITICAL(&s_img_canvas_mux);
        lv_color_t *old_front = s_img_canvas_buf;
        s_img_canvas_buf = pending;
        release_image_canvas_buffer(old_front);
        portEXIT_CRITICAL(&s_img_canvas_mux);
        if (s_img_canvas) {
            lv_canvas_set_buffer(s_img_canvas, s_img_canvas_buf, IMG_W, IMG_H,
                                 LV_IMG_CF_TRUE_COLOR);
            lv_obj_invalidate(s_img_canvas);
        } else if (s_page == UI_PAGE_RX || s_page == UI_PAGE_IMAGE) {
            show_page(UI_PAGE_IMAGE);
        }
        /* A snapshot is not a stream, and canvas submission is not DMA done. */
        s_stat_disp_count = 0;
        stat_render_now();
        return;
    }

    bool page_changed = false;
    /* A playback stays on its own page: the frame goes to the same rows the
     * image page would use, the page just has no canvas of its own. */
    if (s_playback_mode) {
        if (s_page != UI_PAGE_PLAYBACK) {
            show_page(UI_PAGE_PLAYBACK);
            page_changed = true;
        }
    } else if (s_page != UI_PAGE_IMAGE || !s_img_canvas) {
        show_page(UI_PAGE_IMAGE);
        page_changed = true;
    }
    if (s_img_canvas && !lv_obj_has_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_add_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(s_body);
        page_changed = true;
    }
    if (page_changed) {
        lv_refr_now(NULL);
    }

    /* Video owns the rows below the status strip; only frame completion
     * may flush the strip until a page transition releases ownership. */
    bsp_lcd_set_video_direct_owner(true);

    esp_err_t err = bsp_lcd_present_video_frame(
        (const uint16_t *)pending, IMG_W, IMG_H, IMG_Y);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "direct video present failed, keeping last good frame: %s",
                 esp_err_to_name(err));
        portENTER_CRITICAL(&s_img_canvas_mux);
        release_image_canvas_buffer(pending);
        portEXIT_CRITICAL(&s_img_canvas_mux);
    } else {
        portENTER_CRITICAL(&s_img_canvas_mux);
        lv_color_t *old_front = s_img_canvas_buf;
        s_img_canvas_buf = pending;
        release_image_canvas_buffer(old_front);
        portEXIT_CRITICAL(&s_img_canvas_mux);
        if (s_img_canvas) {
            lv_canvas_set_buffer(s_img_canvas, s_img_canvas_buf, IMG_W, IMG_H,
                                 LV_IMG_CF_TRUE_COLOR);
        }
        /* Count the displayed frame and repaint the status strip here: this
         * is the one path that reaches the LCD every frame during a stream. */
        stat_note_displayed_frame();
        stat_render_now();
    }
    s_stream_first_shown = true;
}

void ui_gw_rx_complete(const uint16_t *rgb565, uint32_t w, uint32_t h,
                       uint32_t jpeg_size, uint32_t elapsed_ms)
{
    if (!s_lock || !rgb565 || w != APP_IMAGE_OUTPUT_WIDTH ||
        h != APP_IMAGE_OUTPUT_HEIGHT || !s_img_canvas_buf) return;

    if (s_img_canvas_buffer_count >= 2) {
        lv_color_t *render_buf = NULL;

        portENTER_CRITICAL(&s_img_canvas_mux);
        if (!s_img_canvas_writing && s_img_canvas_back_buf) {
            render_buf = s_img_canvas_back_buf;
            s_img_canvas_back_buf = s_img_canvas_spare_buf;
            s_img_canvas_spare_buf = NULL;
            s_img_canvas_writing = true;
        }
        portEXIT_CRITICAL(&s_img_canvas_mux);

        if (!render_buf) return;

        /* Rotate and convert in the working buffer while the pending frame
         * remains available for presentation. */
        rotate_rgb565_to_canvas(rgb565, render_buf);

        portENTER_CRITICAL(&s_img_canvas_mux);
        lv_color_t *replaced_pending = s_img_canvas_pending_buf;
        s_img_canvas_pending_buf = render_buf;
        release_image_canvas_buffer(replaced_pending);
        s_img_canvas_writing = false;
        portEXIT_CRITICAL(&s_img_canvas_mux);

        return;
    }

    /* Keep the single-buffer fallback when PSRAM cannot hold the full pool. */
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    rotate_rgb565_to_canvas(rgb565, s_img_canvas_buf);

    s_has_image = true;

    if (s_playback_mode) {
        if (s_page != UI_PAGE_PLAYBACK) show_page(UI_PAGE_PLAYBACK);
    } else if (s_page == UI_PAGE_IMAGE && s_img_canvas) {
        lv_obj_invalidate(s_img_canvas);
    } else {
        show_page(UI_PAGE_IMAGE);
    }
    s_stream_first_shown = true;
    /* Low-memory single-buffer fallback obeys the same presentation boundary. */
    if (s_stream_mode || s_intercom_active || s_playback_mode) {
        if (s_img_canvas) lv_obj_add_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN);
        bsp_lcd_set_video_direct_owner(true);
        if (bsp_lcd_present_video_frame((const uint16_t *)s_img_canvas_buf,
                                        IMG_W, IMG_H, IMG_Y) == ESP_OK) {
            stat_note_displayed_frame();
            stat_render_now();
        }
    } else {
        s_stat_disp_count = 0;
        stat_render_now();
    }
    xSemaphoreGiveRecursive(s_lock);
}

/* ─── Visitor records ─── */
void ui_gw_make_thumbnail(const uint16_t *rgb565, uint32_t w, uint32_t h,
                          uint16_t *thumb)
{
    if (!rgb565 || !thumb || w != APP_IMAGE_OUTPUT_WIDTH ||
        h != APP_IMAGE_OUTPUT_HEIGHT) return;
    /* Every APP_VISITOR_THUMB_SCALE-th canvas pixel, same rotation and byte
     * order as rotate_rgb565_to_canvas(). */
    for (int ty = 0; ty < VISITOR_THUMB_H; ty++) {
        const int out_y = ty * APP_VISITOR_THUMB_SCALE;
        for (int tx = 0; tx < VISITOR_THUMB_W; tx++) {
            const int out_x = tx * APP_VISITOR_THUMB_SCALE;
            const uint16_t px = rgb565[(APP_IMAGE_OUTPUT_HEIGHT - 1 - out_x) *
                                           APP_IMAGE_OUTPUT_WIDTH + out_y];
            thumb[ty * VISITOR_THUMB_W + tx] =
                (uint16_t)(((px & 0x001FU) << 11) |
                           (px & 0x07E0U) |
                           ((px & 0xF800U) >> 11));
        }
    }
}

void ui_gw_records_changed(void)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    if (s_page == UI_PAGE_VISITORS) {
        show_page(UI_PAGE_VISITORS);
    }
    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_playback_ended(void)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    /* Only a playback still on screen returns to the list; after a tap or
     * key the UI has already left. */
    if (s_playback_mode) {
        ESP_LOGI(TAG, "playback: record over, back to the list");
        s_playback_mode = false;
        s_playback_index = -1;
        bsp_lcd_set_video_direct_owner(false);
        show_page(UI_PAGE_VISITORS);
    }
    xSemaphoreGiveRecursive(s_lock);
}

bool ui_gw_playback_active(void)
{
    return s_playback_mode;
}

void ui_gw_rx_failed(const char *reason)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    show_page(UI_PAGE_IMAGE);
    update_title("Latest", "FAIL", lv_color_hex(0xA94442));
    lv_label_set_text(s_status_lbl_r, reason ? reason : "RX Failed");

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_eot_nack(uint16_t missing_count, bool is_first_eot)
{
    if (is_first_eot) {
        s_stats_first_missing = missing_count;
        s_stats_first_eot_seen = true;
    }
    s_stats_total_retransmitted += missing_count;
}

void ui_gw_set_rx_abort_cb(ui_gw_rx_abort_cb_t cb)
{
    s_rx_abort_cb = cb;
}

/* Map a supply voltage (mV) to its status-bar text color.
 *   > 3.5V green, 3.3~3.5V amber, < 3.3V red. Unknown (0) stays light. */
static lv_color_t vbat_level_color(uint16_t mv)
{
    if (mv == 0)      return COL_TEXT_LIGHT;
    if (mv > 3500)    return COL_VBAT_GREEN;
    if (mv >= 3300)   return COL_VBAT_AMBER;
    return COL_VBAT_RED;
}

/* Refresh the gateway's own voltage on the status bar left label from the
 * bsp_vbat cache. Caller must hold s_lock. */
static void gw_vbat_refresh(void)
{
    if (!s_status_lbl_l) return;

    uint16_t mv = bsp_vbat_get_cached();
    s_gw_vbat_mv = mv;

    char buf[32];
    if (mv > 0) {
        /* e.g. 3982 mV -> "GW 3.98V" */
        snprintf(buf, sizeof(buf), "GW %u.%02uV", mv / 1000, (mv % 1000) / 10);
    } else {
        snprintf(buf, sizeof(buf), "GW --V");
    }
    lv_label_set_text(s_status_lbl_l, buf);
    lv_obj_set_style_text_color(s_status_lbl_l, vbat_level_color(mv), 0);
}

/* ─── Link account rendering ─── */

/* Record one frame reaching the panel. Called from the present path only. */
static void stat_note_displayed_frame(void)
{
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_stat_disp_count == STAT_FPS_MARKS) {
        for (uint8_t i = 1; i < STAT_FPS_MARKS; i++) {
            s_stat_disp_ms[i - 1U] = s_stat_disp_ms[i];
        }
        s_stat_disp_count--;
    }
    s_stat_disp_ms[s_stat_disp_count++] = now;
}

/* Displayed frames per second * 10, or 0 with fewer than two presents (a rate
 * from one frame would have to guess the interval). */
static uint16_t stat_displayed_fps_x10(void)
{
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    /* Expire marks that left the window. Keep the newest so a stopped stream
     * decays to "--" instead of freezing on its last rate. */
    while (s_stat_disp_count > 1U &&
           (now - s_stat_disp_ms[0]) > STAT_FPS_WINDOW_MS) {
        for (uint8_t i = 1; i < s_stat_disp_count; i++) {
            s_stat_disp_ms[i - 1U] = s_stat_disp_ms[i];
        }
        s_stat_disp_count--;
    }
    if (s_stat_disp_count < 2U) return 0U;

    /* A single stale mark left behind by a stream that stopped must not be read
     * as a live rate either. */
    if ((now - s_stat_disp_ms[s_stat_disp_count - 1U]) > STAT_FPS_WINDOW_MS) {
        return 0U;
    }

    const uint32_t dt =
        s_stat_disp_ms[s_stat_disp_count - 1U] - s_stat_disp_ms[0];
    if (dt == 0U) return 0U;
    return (uint16_t)((uint64_t)(s_stat_disp_count - 1U) * 10000ULL / dt);
}

static void stat_render_rate(void)
{
    if (!s_status_lbl_l2) return;
    char buf[32];
    const uint16_t fps_x10 = stat_displayed_fps_x10();

    if (s_stat_bytes_per_s > 0U) {
        /* KB is 1024 bytes, not 1000: dividing by 1000 would report a bigger
         * number for the same traffic. */
        const uint32_t kb_x10 = s_stat_bytes_per_s * 10U / 1024U;
        if (fps_x10 > 0U) {
            snprintf(buf, sizeof(buf), "%lu.%luKB/s %u.%uFPS",
                     (unsigned long)(kb_x10 / 10U), (unsigned long)(kb_x10 % 10U),
                     (unsigned)(fps_x10 / 10U), (unsigned)(fps_x10 % 10U));
        } else {
            snprintf(buf, sizeof(buf), "%lu.%luKB/s --FPS",
                     (unsigned long)(kb_x10 / 10U), (unsigned long)(kb_x10 % 10U));
        }
    } else {
        snprintf(buf, sizeof(buf), "--KB/s --FPS");
    }
    lv_label_set_text(s_status_lbl_l2, buf);
}

/* Snapshot pending events at an allowed presentation boundary; release the
 * mailbox lock before drawing under the LVGL lock. */
static void stat_render_now(void)
{
    portENTER_CRITICAL(&s_stat_mux);
    s_stat_bytes_per_s = s_stat_pending.bytes_per_s;
    s_stat_rssi = s_stat_pending.rssi;
    s_stat_rssi_valid = s_stat_pending.rssi_valid;
    s_stat_last_traffic_ms = s_stat_pending.traffic_ms;
    s_stat_last_event_ms = s_stat_pending.event_ms;
    s_node_vbat_mv = s_stat_pending.node_vbat_mv;
    s_stat_pending.dirty = false;
    portEXIT_CRITICAL(&s_stat_mux);
    gw_vbat_refresh();
    node_vbat_render();
    stat_render_rate();
    stat_render_age();
    /* A previous ordinary LVGL pass may have consumed invalidations while
     * video owned the panel. Repaint the whole strip at this allowed boundary. */
    if (s_status_bar) lv_obj_invalidate(s_status_bar);
    bsp_lcd_refresh_video_status();
}

/* Row-2 right: RSSI plus the age of the last real event. The offline notice
 * outranks both — if the heartbeat stopped, neither number means anything. */
static void stat_render_age(void)
{
    if (!s_status_lbl_r2) return;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    if (s_stat_last_traffic_ms != 0U &&
        (now - s_stat_last_traffic_ms) > NODE_OFFLINE_MS) {
        lv_label_set_text(s_status_lbl_r2, "NODE OFFLINE");
        lv_obj_set_style_text_color(s_status_lbl_r2, COL_VBAT_RED, 0);
        return;
    }
    lv_obj_set_style_text_color(s_status_lbl_r2, COL_MUTED, 0);

    char age[12];
    if (s_stat_last_event_ms == 0U) {
        snprintf(age, sizeof(age), "no events");
    } else {
        const uint32_t min = (now - s_stat_last_event_ms) / 60000U;
        if (min < 1U) {
            snprintf(age, sizeof(age), "now");
        } else if (min < 60U) {
            snprintf(age, sizeof(age), "%lum ago", (unsigned long)min);
        } else {
            snprintf(age, sizeof(age), ">1h");
        }
    }

    char buf[32];
    if (s_stat_rssi_valid) {
        snprintf(buf, sizeof(buf), "%ddBm %s", (int)s_stat_rssi, age);
    } else {
        snprintf(buf, sizeof(buf), "--dBm %s", age);
    }
    lv_label_set_text(s_status_lbl_r2, buf);
}

static void gw_vbat_timer_cb(lv_timer_t *t)
{
    (void)t;
    /* Cache/age maintenance is deferred to the next frame during transfer. */
    if (s_stream_mode || s_intercom_active || s_page == UI_PAGE_RX) return;
    stat_render_now();
}

/* Idle event delivery only. During transfer, pending values wait for the
 * next displayed frame even if this timer fires before decoding finishes. */
static void stat_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (s_stream_mode || s_intercom_active || s_page == UI_PAGE_RX) return;
    portENTER_CRITICAL(&s_stat_mux);
    const bool dirty = s_stat_pending.dirty;
    portEXIT_CRITICAL(&s_stat_mux);
    if (dirty) stat_render_now();
}

void ui_gw_update_link_stats(uint32_t bytes_per_s, int16_t rssi, bool rssi_valid)
{
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    portENTER_CRITICAL(&s_stat_mux);
    s_stat_pending.bytes_per_s = bytes_per_s;
    if (rssi_valid) {
        s_stat_pending.rssi = rssi;
        s_stat_pending.rssi_valid = true;
    }
    s_stat_pending.traffic_ms = now ? now : 1U;
    s_stat_pending.dirty = true;
    portEXIT_CRITICAL(&s_stat_mux);
}

void ui_gw_note_event(void)
{
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    portENTER_CRITICAL(&s_stat_mux);
    s_stat_pending.event_ms = now ? now : 1U;
    s_stat_pending.dirty = true;
    portEXIT_CRITICAL(&s_stat_mux);
}

void ui_gw_update_vbat(uint16_t vbat_mv)
{
    portENTER_CRITICAL(&s_stat_mux);
    s_stat_pending.node_vbat_mv = vbat_mv;
    s_stat_pending.dirty = true;
    portEXIT_CRITICAL(&s_stat_mux);
}

static void node_vbat_render(void)
{
    const uint16_t vbat_mv = s_node_vbat_mv;
    if (s_status_lbl_r) {
        char buf[32];
        if (vbat_mv > 0) {
            /* e.g. 3982 mV -> "NODE 3.98V" */
            snprintf(buf, sizeof(buf), "NODE %u.%02uV",
                     vbat_mv / 1000, (vbat_mv % 1000) / 10);
        } else {
            snprintf(buf, sizeof(buf), "NODE --V");
        }
        lv_label_set_text(s_status_lbl_r, buf);
        lv_obj_set_style_text_color(s_status_lbl_r, vbat_level_color(vbat_mv), 0);
    }

}

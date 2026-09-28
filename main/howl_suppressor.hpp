#pragma once

#include <cstddef>
#include <cstdint>

#include "app_config.h"

/* Feedback killer. Two instances: the one-way A/V playback on the gateway
 * (APP_AV_HOWL_*) and the microphone path of a call on both boxes
 * (APP_INTERCOM_HOWL_*). See those blocks in app_config.h for the loop each
 * one protects against and the calibration behind the thresholds.
 *
 * Per 10 ms frame it measures two features of the pcm it is given:
 *   clip_percent - share of samples at or beyond the clip level
 *   rms          - root-mean-square amplitude
 * and runs a two-state machine with hysteresis on both axes plus a hard cap on
 * the mute length. While muted the caller zeroes the frame; that opens the
 * loop, and the features fall back under the OFF limits by themselves one
 * round trip later.
 *
 * No allocation, no floating point, ~2 multiplies per sample: it runs inside
 * the playback task on core 0 next to the Opus decoder, and inside the
 * microphone task on core 1 next to the AEC. */
class HowlSuppressor {
public:
    struct Features {
        uint32_t clip_percent = 0;
        uint32_t rms = 0;
    };

    struct Config {
        int32_t clip_level;
        uint32_t on_clip_percent;
        uint32_t on_rms;
        uint32_t on_frames;
        uint32_t off_clip_percent;
        uint32_t off_rms;
        uint32_t off_frames;
        uint32_t max_mute_frames;
    };

    /* Decoded stream audio on the gateway, measured before the playback
     * attenuation. */
    static constexpr Config kAvPlayback = {
        APP_AV_HOWL_CLIP_LEVEL,
        APP_AV_HOWL_ON_CLIP_PERCENT,
        APP_AV_HOWL_ON_RMS,
        APP_AV_HOWL_ON_FRAMES,
        APP_AV_HOWL_OFF_CLIP_PERCENT,
        APP_AV_HOWL_OFF_RMS,
        APP_AV_HOWL_OFF_FRAMES,
        APP_AV_HOWL_MAX_MUTE_FRAMES,
    };

    /* AEC output of a call, before the Opus encoder. */
    static constexpr Config kCallCapture = {
        APP_INTERCOM_HOWL_CLIP_LEVEL,
        APP_INTERCOM_HOWL_ON_CLIP_PERCENT,
        APP_INTERCOM_HOWL_ON_RMS,
        APP_INTERCOM_HOWL_ON_FRAMES,
        APP_INTERCOM_HOWL_OFF_CLIP_PERCENT,
        APP_INTERCOM_HOWL_OFF_RMS,
        APP_INTERCOM_HOWL_OFF_FRAMES,
        APP_INTERCOM_HOWL_MAX_MUTE_FRAMES,
    };

    explicit HowlSuppressor(const Config &cfg = kAvPlayback) : cfg_(cfg) {}

    /* Feed one frame. Returns true when the frame must be muted. */
    bool process(const int16_t *pcm, size_t samples)
    {
        last_ = measure(pcm, samples, cfg_.clip_level);
        const bool over_on = last_.clip_percent >= cfg_.on_clip_percent &&
                             last_.rms >= cfg_.on_rms;
        const bool under_off = last_.clip_percent <= cfg_.off_clip_percent &&
                               last_.rms <= cfg_.off_rms;

        if (!muted_) {
            on_count_ = over_on ? on_count_ + 1U : 0U;
            if (on_count_ >= cfg_.on_frames) {
                muted_ = true;
                on_count_ = 0;
                off_count_ = 0;
                muted_frames_ = 0;
                mute_events_++;
            }
        } else {
            muted_frames_++;
            off_count_ = under_off ? off_count_ + 1U : 0U;
            // The cap is what keeps a loud but honest talker audible: without
            // it a voice that never drops under OFF_RMS would stay muted for
            // as long as it talks.
            if (off_count_ >= cfg_.off_frames ||
                muted_frames_ >= cfg_.max_mute_frames) {
                muted_ = false;
                on_count_ = 0;
                off_count_ = 0;
            }
        }
        return muted_;
    }

    bool muted() const { return muted_; }
    const Features &last() const { return last_; }
    uint32_t mute_events() const { return mute_events_; }
    uint32_t muted_frames() const { return muted_frames_; }
    /* Whether the last mute ended on the hard cap rather than on the OFF
     * limits: a loud talker held it open, not a decayed howl. */
    bool hit_cap() const { return muted_frames_ >= cfg_.max_mute_frames; }

    /* Forget the state, keep the event counter: a playback restart or a new
     * call is a new timeline, not a new device. */
    void reset()
    {
        muted_ = false;
        on_count_ = 0;
        off_count_ = 0;
        muted_frames_ = 0;
        last_ = {};
    }

    static Features measure(const int16_t *pcm, size_t samples, int32_t clip_level)
    {
        Features f;
        if (pcm == nullptr || samples == 0) return f;
        uint32_t clipped = 0;
        uint64_t sum_sq = 0;
        for (size_t i = 0; i < samples; ++i) {
            const int32_t s = pcm[i];
            const int32_t a = s < 0 ? -s : s;
            if (a >= clip_level) clipped++;
            // s*s <= 2^30, fits int32; the sum over a frame does not, hence 64.
            sum_sq += static_cast<uint32_t>(s * s);
        }
        f.clip_percent = static_cast<uint32_t>((clipped * 100U + samples / 2U) / samples);
        f.rms = isqrt32(static_cast<uint32_t>(sum_sq / samples));
        return f;
    }

private:
    static uint32_t isqrt32(uint32_t x)
    {
        uint32_t r = 0;
        uint32_t bit = 1UL << 30;
        while (bit > x) bit >>= 2;
        while (bit != 0) {
            if (x >= r + bit) {
                x -= r + bit;
                r = (r >> 1) + bit;
            } else {
                r >>= 1;
            }
            bit >>= 2;
        }
        return r;
    }

    Config cfg_;
    bool muted_ = false;
    uint32_t on_count_ = 0;
    uint32_t off_count_ = 0;
    uint32_t muted_frames_ = 0;
    uint32_t mute_events_ = 0;
    Features last_;
};

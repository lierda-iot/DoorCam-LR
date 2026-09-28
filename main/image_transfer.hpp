#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "app_config.h"

class ImageTransfer {
public:
    // TX side: encode YUV422 frame to JPEG. Caller frees *out_jpeg with heap_caps_free.
    esp_err_t encode_frame(const uint8_t *yuv422, size_t yuv_len,
                           uint32_t width, uint32_t height, uint32_t pixfmt,
                           uint8_t **out_jpeg, size_t *out_jpeg_len);

    // TX hot path: encode an aligned YCbYCr/GRAY buffer prepared by the camera.
    // The caller retains ownership of input and frees it with heap_caps_free().
    esp_err_t encode_prepared_frame(const uint8_t *input, size_t input_len,
                                    uint32_t width, uint32_t height, uint32_t pixfmt,
                                    uint8_t **out_jpeg, size_t *out_jpeg_len);

    // TX side: quality for the next encode, clamped to the CONFIG range. Set
    // from the radio task; each encode reads it once when it opens the encoder.
    void set_jpeg_quality(uint8_t quality);
    uint8_t jpeg_quality() const { return jpeg_quality_; }

    // RX side: start new reassembly session
    void rx_begin(uint16_t session_id, uint16_t total_fragments);

    // RX side: add a fragment. Returns true when all fragments received.
    bool rx_fragment(uint16_t session_id, uint16_t frag_index,
                     uint16_t total_fragments, const uint8_t *data, uint16_t len);

    // RX side: get list of missing fragment indices
    uint16_t rx_get_missing(uint16_t *out_indices, uint16_t max_count) const;

    // RX side: build bitmap where bit=1 means received. Returns false if no RX active.
    bool rx_build_bitmap(uint8_t *bitmap, uint16_t max_bytes,
                         uint16_t *out_first_missing, uint16_t *out_byte_count) const;

    // RX side: decode reassembled JPEG to RGB565. Caller frees *out_rgb565 with jpeg_free_align.
    esp_err_t decode_to_rgb565(uint8_t **out_rgb565, uint32_t *out_w, uint32_t *out_h);

    // RX side: reassemble fragments into contiguous buffer. Caller frees with heap_caps_free.
    esp_err_t rx_reassemble(uint8_t **out_buf, size_t *out_len);

    // RX side: free reassembly buffer
    void rx_reset();

    // RX side: clear the received-fragment map for the SAME session/size without
    // freeing and reallocating buffers. Used by the intercom frame-rate counter,
    // which reassembles the same repeating frame many times per second and would
    // otherwise thrash the heap with a full rx_begin() on every completed frame.
    void rx_restart();

    uint16_t rx_session_id() const { return rx_session_id_; }
    uint16_t rx_received_count() const { return rx_received_; }
    uint16_t rx_total_count() const { return rx_total_; }
    size_t rx_jpeg_size() const { return rx_jpeg_size_; }
    uint32_t rx_crc32() const;
    bool rx_active() const { return rx_buf_ != nullptr; }
    bool rx_complete() const { return rx_total_ > 0 && rx_received_ == rx_total_; }

private:
    volatile uint8_t jpeg_quality_ = APP_IMAGE_JPEG_QUALITY;
    uint8_t  *rx_buf_ = nullptr;
    uint16_t *rx_frag_lens_ = nullptr;
    bool     *rx_received_map_ = nullptr;
    uint16_t  rx_session_id_ = 0;
    uint16_t  rx_total_ = 0;
    uint16_t  rx_received_ = 0;
    size_t    rx_jpeg_size_ = 0;
};

//
//    openSAM: manage DGS for X Plane
//
//    Copyright (C) 2026 Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

#include <cassert>
#include <cctype>
#include <string>
#include <iostream>
#include <vector>
#include <queue>
#include <fstream>
#include <memory>
#include <print>
#include <bit>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "dgs/dyn_display.h"
#include "dgs/dgs_impl.h"
#include "time_code_block.h"

#include "log_msg.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"
#pragma GCC diagnostic pop

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

// stb_image_writer.h always compresses with at least level 5
// and spng allows uncompressed -> >4 times faster, 5 ms vs 28 ms
#include "spng.h"

namespace dgs {
namespace DynDisplay {

static bool SaveImagePng(const uint32_t* data, int width, int height, const std::string& png_path);

//------------------------------------------------------------------------------------
Font::Font(const std::string& font_path) {
    std::ifstream font_file(font_path, std::ios::binary | std::ios::ate);
    if (!font_file.is_open()) {
        throw std::runtime_error("Error: Could not open font file: " + font_path);
    }

    std::streamsize font_size = font_file.tellg();
    font_file.seekg(0, std::ios::beg);

    font_buffer_.resize(font_size);
    if (!font_file.read(reinterpret_cast<char*>(font_buffer_.data()), font_size)) {
        throw std::runtime_error("Error: Failed to read font file.");
    }

    font_info_ = new stbtt_fontinfo;
    if (!stbtt_InitFont(font_info_, font_buffer_.data(), 0)) {
        throw std::runtime_error("Error: Failed to initialize font.");
    }

    glyph_cache_.reserve(256);  // initial space for 256 glyphs in the cache
}

Font::~Font() {
    if (font_info_) {
        delete font_info_;
        font_info_ = nullptr;
    }

    for (auto& [key, gi] : glyph_cache_) {
        if (gi.bitmap)
            stbtt_FreeBitmap(gi.bitmap, nullptr);
    }

    glyph_cache_.clear();
}

float Font::ScaleForPixelHeight(float height) const noexcept {
    return stbtt_ScaleForPixelHeight(font_info_, height);
}

void Font::GetFontVMetrics(int& ascent, int& descent, int& line_gap) const noexcept {
    stbtt_GetFontVMetrics(font_info_, &ascent, &descent, &line_gap);
}

void Font::GetCodepointHMetrics(int codepoint, int& advance, int& lsb) const noexcept {
    stbtt_GetCodepointHMetrics(font_info_, codepoint, &advance, &lsb);
}

int Font::GetCodepointKernAdvance(int left_codepoint, int right_codepoint) const noexcept {
    return stbtt_GetCodepointKernAdvance(font_info_, left_codepoint, right_codepoint);
}

// Creation of a Glyph bitmap is very expensive so we create a cache by size/codepoint
const unsigned char* Font::GetCodepointBitmap(int codepoint, float scale_x, float scale_y, int& width, int& height,
                                              int& x_off, int& y_off) {
    assert(scale_x == scale_y);  // currently we only support uniform scaling

    // fabricate a unique key for the glyph cache based on the scale and codepoint
    uint64_t scale_ui = std::bit_cast<uint32_t>(scale_x);
    uint64_t cp_ui = static_cast<uint64_t>(static_cast<uint32_t>(codepoint));
    uint64_t key = (scale_ui << 32) | cp_ui;

    auto it = glyph_cache_.find(key);
    if (it != glyph_cache_.end()) {
        GlyphInfo& gi = it->second;
        //LogMsg("Font::GetCodepointBitmap: returning cached glyph for codepoint %c, scale=%f", codepoint, gi.scale);
        width = gi.width;
        height = gi.height;
        x_off = gi.x_off;
        y_off = gi.y_off;
        return gi.bitmap;
    }

    GlyphInfo gi;
    gi.scale = scale_x;
    gi.bitmap =
        stbtt_GetCodepointBitmap(font_info_, scale_x, scale_y, codepoint, &gi.width, &gi.height, &gi.x_off, &gi.y_off);

    width = gi.width;
    height = gi.height;
    x_off = gi.x_off;
    y_off = gi.y_off;
    // even with negative caching for a non-existent glyph
    glyph_cache_[key] = gi;  // cache the glyph
    return gi.bitmap;
}

//------------------------------------------------------------------------------------
Image::Image(const std::string& filename) {
    int n;
    unsigned char* data = stbi_load(filename.c_str(), &width_, &height_, &n, STBI_rgb_alpha);
    if (!data || n != 4) {
        throw std::runtime_error("Failed to load image: " + filename);
    }

    pixels_ = std::make_unique_for_overwrite<unsigned char[]>(width_ * height_ * 4);
    memcpy(pixels_.get(), data, width_ * height_ * 4);
    stbi_image_free(data);
}

void Image::Text(int x, int y, Font& font, float height, Color color, const std::string& txt) {
    float scale = font.ScaleForPixelHeight(static_cast<float>(height));

    int ascent, descent, line_gap;
    font.GetFontVMetrics(ascent, descent, line_gap);

    int x_position = x;

    for (int i = 0; i < (int)txt.size(); i++) {
        unsigned char c = txt[i];
        if (!std::isprint(c))
            c = '.';

        int advance, lsb;
        font.GetCodepointHMetrics(c, advance, lsb);

        int glyph_width, glyph_height, x_off, y_off;
        // glyph_bitmap is a non-owning pointer, owned by the font's glyph_cache_
        const unsigned char* glyph_bitmap =
            font.GetCodepointBitmap(c, scale, scale, glyph_width, glyph_height, x_off, y_off);

        if (glyph_bitmap) {
            int render_x = x_position + x_off;
            int render_y = y + y_off;

            for (int gy = 0; gy < glyph_height; ++gy) {
                for (int gx = 0; gx < glyph_width; ++gx) {
                    int px = render_x + gx;
                    int py = render_y + gy;

                    if (px >= 0 && px < width_ && py >= 0 && py < height_) {
                        unsigned char alpha = glyph_bitmap[gy * glyph_width + gx];
                        if (alpha > 0) {
                            int pixel_idx = (py * width_ + px) * 4;
                            pixels_[pixel_idx + 0] = color.r;  // R
                            pixels_[pixel_idx + 1] = color.g;  // G
                            pixels_[pixel_idx + 2] = color.b;  // B
                            pixels_[pixel_idx + 3] = alpha;    // A
                        }
                    }
                }
            }
        }

        x_position += static_cast<int>(advance * scale);

        if (i + 1 < (int)txt.size()) {
            int kern = font.GetCodepointKernAdvance(c, txt[i + 1]);
            x_position += static_cast<int>(kern * scale);
        }
    }
}

// height in pixels -> pixels
int Image::MeasureText(Font& font, float height, const std::string& txt) {
    float scale = font.ScaleForPixelHeight(static_cast<float>(height));
    int width = 0;
    for (int i = 0; i < (int)txt.size(); i++) {
        char c = txt[i];
        int advance, lsb;
        font.GetCodepointHMetrics(c, advance, lsb);
        width += static_cast<int>(advance * scale);
        if (i + 1 < (int)txt.size()) {
            int kern = font.GetCodepointKernAdvance(c, txt[i + 1]);
            width += static_cast<int>(kern * scale);
        }
    }
    return width;
}

void Image::Paste(const Image& src, int x, int y) {
    for (int j = 0; j < src.height_; ++j) {
        for (int i = 0; i < src.width_; ++i) {
            int dst_x = x + i;
            int dst_y = y + j;
            if (0 <= dst_x && dst_x < width_ && 0 <= dst_y && dst_y < height_) {
                int dst_idx = (dst_y * width_ + dst_x) * 4;
                int src_idx = (j * src.width_ + i) * 4;
                pixels_[dst_idx + 0] = src.pixels_[src_idx + 0];
                pixels_[dst_idx + 1] = src.pixels_[src_idx + 1];
                pixels_[dst_idx + 2] = src.pixels_[src_idx + 2];
                pixels_[dst_idx + 3] = src.pixels_[src_idx + 3];
            }
        }
    }
}

bool Image::Save(const std::string& filename) const {
    return SaveImagePng((const uint32_t*)pixels_.get(), width_, height_, filename);
}

//------------------------------------------------------------------------------------
static int seqno_base = 0;
static std::queue<DynDisplay::Display*> in_q, out_q;
static std::mutex q_mtx;                 // protects both queues
static std::condition_variable in_q_cv;  // input available
static std::thread bg_thread;
static bool bg_stop_thread;  // stop thread on next occasion

int Display::active_displays_ = 0;  // for leak detection

Display::Display(float width_m, float height_m, float x0, float y0, float z0)
    : id_(++seqno_base), width_m_(width_m), height_m_(height_m) {
    m_2_pix = kImgHeight / height_m_;           // Conversion factor from meters to pixels
    s_max_ = width_m_ * m_2_pix / kImgWidth;    // Normalized texture coordinate max
    t_max_ = height_m_ * m_2_pix / kImgHeight;  // Normalized texture coordinate max
    r_margin_pix_ = static_cast<int>(width_m_ * m_2_pix);
    x0_ = x0;  // Blender to OGL
    y0_ = z0;
    z0_ = -y0 + 0.003f;  // Slightly in front of the display plane
    image_ = std::make_unique<Image>(kImgWidth, kImgHeight);
    active_displays_++;

    if (id_ == 1) {
        bg_stop_thread = false;
        bg_thread = std::thread(BgWorker);
        LogMsg("DynDisplay: Started background thread for baking of objects");
    }
}

Display::~Display() {
    assert(!async_in_progress_);

    if (xplm_obj_ != nullptr)
        XPLMUnloadObject(xplm_obj_);

#if 1
    if (baked_) {
        std::filesystem::remove(png_pathname_);
        std::filesystem::remove(obj_pathname_);
    }
#endif

    active_displays_--;
}

// centered = true means center the text horizontally
void Display::TextLine(Color color, Font& font, const std::string& txt, bool centered) {
    assert(!baked_);

    int height_pix = (txt_scale_ * txt_height_ / height_m_) * kImgHeight;  // Convert m to pixels
    assert(image_ != nullptr);
    // LogMsg("Display::TextLine: Writing text '%s' at (%.3f, %.3f) m, height=%.3f m, scale=%.3f, centered=%d", txt.c_str(),
    //       txt_x_, txt_y_, txt_height_, txt_scale_, centered);
    assert(0 <= txt_x_ && txt_x_ < width_m_);
    assert(0 <= txt_y_ && txt_y_ < height_m_);
    auto [x_pix, y_pix] = ToPix(txt_x_, txt_y_);

    int text_width = image_->MeasureText(font, height_pix, txt);
    if (centered) {
        // std::cout << "Text width in pixels: " << text_width << std::endl;
        x_pix = (r_margin_pix_ - text_width) / 2;
    }

    image_->Text(x_pix, y_pix, font, height_pix, color, txt);

    prev_txt_y_ = txt_y_;
    prev_txt_x_ = txt_x_ + text_width;

    txt_x_ = 0.0f;
    txt_y_ -= txt_scale_ * line_spacing_;
}

void Display::Paste(const Image& src, float x, float y) {
    assert(!baked_);
    assert(image_ != nullptr);
    int x_pix = static_cast<int>(x * m_2_pix);
    int y_pix = static_cast<int>((1.0f - (y / height_m_)) * kImgHeight);
    y_pix -= src.height();  // Adjust y to paste from the bottom left corner
    image_->Paste(src, x_pix, y_pix);
}

void Display::Bake(const std::string& dirname, const std::string& filename) {
    assert(!baked_);
    assert(xplm_obj_ == nullptr);

    png_filename_ = filename + std::format("-{:04d}.png", id_);
    png_pathname_ = dirname + png_filename_;
    obj_pathname_ = dirname + filename + std::format("-{:04d}.obj", id_);
    baked_ = true;  // no double baking

    // LogMsg("Bake: Creating display object '%s' and texture '%s'", obj_pathname_.c_str(),
    //        png_pathname_.c_str());
    {
        std::lock_guard<std::mutex> lock(q_mtx);
        png_ok_ = false;
        on_bg_q = true;
        in_q.push(this);
        in_q_cv.notify_one();
    }
}

// Start async load of xp object after the PNG file has been created in the background thread
void Display::BakePhase2() {
    if (!png_ok_)
        throw std::runtime_error("Bake: Failed to save image: " + png_pathname_);

    async_in_progress_ = true;
    XPLMLoadObjectAsync(obj_pathname_.c_str(), ObjectLoadedCb, this);
    LogMsg("Bake: Started async loading of display object '%s'", obj_pathname_.c_str());
}

void Display::ObjectLoadedCb(XPLMObjectRef inObject, void* inRefcon) {
    Display* display = reinterpret_cast<Display*>(inRefcon);
    display->xplm_obj_ = inObject;
    display->async_in_progress_ = false;
    LogMsg("Display::ObjectLoaded: Async load callback for display %d, object=%p", display->id_, inObject);

    if (display->retired_) {
        LogMsg("Display::ObjectLoaded: Display was marked for retirement, deleting now");
        delete display;
        return;
    }

    if (display->xplm_obj_) {
        LogMsg("Display::ObjectLoaded: Successfully loaded object asynchronously");
    } else {
        LogMsg("Display::ObjectLoaded: Error - Failed to load object asynchronously");
    }
}

void Display::Deleter(Display* display) {
    if (display == nullptr) // can that ever happen?
        return;

    if (display->async_in_progress_ || display->on_bg_q) {
        display->retired_ = true;
        LogMsg("Display::Deleter: Display is still loading asynchronously or on background queue, will delete later");
    } else {
        LogMsg("Display::Deleter: Deleting display immediately");
        delete display;
    }
}

const char* Page::dlist_dr_[] = {"opensam/dgs/vdgs_brightness", nullptr};

bool Page::Show() {
    if (dd_) {
        auto obj_ref = dd_->GetXplmObj();
        if (obj_ref == nullptr) {
            LogMsg("Page::Show: Object not yet available");
            return false;
        }
        inst_ref_ = CreateInstance(obj_ref, dlist_dr_);
        SetPosition();
        ts_ = now;
        return true;
    }

    return false;
}

void Page::SetPosition() {
    if (inst_ref_) {
        float brightness = dgs::VDGSBrightness();
        inst_ref_->SetPosition(&drawinfo_, &brightness);
    }
}

static bool SaveImagePng(const uint32_t* data, int width, int height, const std::string& png_path) {
    // create .png
    struct spng_ihdr ihdr = {};

    // Creating an encoder context requires a flag
    spng_ctx* ctx = spng_ctx_new(SPNG_CTX_ENCODER);

    // Encode to internal buffer managed by the library
    spng_set_option(ctx, SPNG_ENCODE_TO_BUFFER, 1);
    spng_set_option(ctx, SPNG_IMG_COMPRESSION_LEVEL, 0);  // save CPU

    // Set image properties, this determines the destination image format
    ihdr.width = width;
    ihdr.height = height;
    ihdr.color_type = SPNG_COLOR_TYPE_TRUECOLOR_ALPHA;
    ihdr.bit_depth = 8;

    spng_set_ihdr(ctx, &ihdr);

    // SPNG_ENCODE_FINALIZE will finalize the PNG with the end-of-file marker
    int ret = spng_encode_image(ctx, data, width * height * sizeof(uint32_t), SPNG_FMT_PNG, SPNG_ENCODE_FINALIZE);
    if (ret) {
        LogMsg("spng_encode_image() error: %s", spng_strerror(ret));
        spng_ctx_free(ctx);
        return false;
    }

    size_t png_size;
    void* png_buf = spng_get_png_buffer(ctx, &png_size, &ret);
    // User owns the buffer after a successful call

    if (png_buf == NULL) {
        LogMsg("spng_get_png_buffer() error: %s", spng_strerror(ret));
        return false;
    }

    std::ofstream f(png_path, std::ios::binary);
    if (f.fail()) {
        LogMsg("Can't open '%s'", png_path.c_str());
        return false;
    }

    f.write((const char*)png_buf, png_size);
    f.close();
    if (f.fail())
        LogMsg("write to png failed");
    else {
        //LogMsg("PNG '%s' created", png_path.c_str());
    }

    free(png_buf);
    spng_ctx_free(ctx);
    return true;
}

// static
void Display::BgWorker() {
    std::unique_lock<std::mutex> lock(q_mtx);
    while (true) {
        // come here with the lock held
        in_q_cv.wait(lock, [] {
            if (bg_stop_thread || !in_q.empty())
                return true;
            LogMsg("BgWorker waiting on cv");
            return false;
        });

        if (bg_stop_thread) {
            LogMsg("Background thread stop requested, thread exiting");
            return;
        }

        // still locked
        DynDisplay::Display* display = in_q.front();
        in_q.pop();
        lock.unlock();

        display->BakeBg();      // needs some time

        lock.lock();
        out_q.push(display);
        // keep the lock while going to cv_wait
    }
}

void Display::BakeBg() {
    std::ofstream obj_file(obj_pathname_);
    if (!obj_file.is_open())
        throw std::runtime_error("Bake: Failed to open file for writing: " + obj_pathname_);

    auto vt = [this](float x, float y, float z, float s, float t) {
        return std::format("VT {:8.5f} {:8.5f} {:8.5f} 0 0 1 {:6.5f} {:6.5f}\n", x + x0_, y + y0_, z + z0_, s, t);
    };

    obj_file << "I\n"
                "800\n"
                "OBJ\n"
                << "TEXTURE     " << png_filename_ << "\n"
                << "TEXTURE_LIT " << png_filename_ << "\n\n"
                << "POINT_COUNTS 4 0 0 6\n"
                << vt(0, 0, 0, 0, 0)
                << vt(0, height_m_, 0, 0, t_max_)
                << vt(width_m_, 0, 0, s_max_, 0)
                << vt(width_m_, height_m_, 0, s_max_, t_max_)
                << "IDX  0\n"
                "IDX  1\n"
                "IDX  2\n"
                "IDX  1\n"
                "IDX  3\n"
                "IDX  2\n\n"
                "GLOBAL_no_blend 0.5\n"
                "ATTR_LOD 0 200\n"
                "ATTR_no_shadow\n"
                "ATTR_light_level 0.0 1.0 opensam/dgs/vdgs_brightness 7000\n"
                "TRIS 0 6\n";
    obj_file.close();

    png_ok_ = image_->Save(png_pathname_);
    // LogMsg("Display::BakeBg: PNG file '%s' written, success=%d", png_pathname_.c_str(), png_ok_);
}

// static
void Display::BgPostProcess() {
    // LogMsg("Display::BgPostProcess: Processing completed background tasks");
    std::lock_guard<std::mutex> lock(q_mtx);
    while (!out_q.empty()) {
        Display* display = out_q.front();
        out_q.pop();
        //LogMsg("Display::BgPostProcess: Processing display %p", (void *)display);
        //LogMsg("Display::BgPostProcess: Processing display %d", display->id_);
        display->on_bg_q = false;
        if (display->retired_) {
            LogMsg("Display::BgPostProcess: Display was marked for retirement, deleting now");
            delete display;
            continue;
        }

        display->BakePhase2();      // that is fast so we keep the lock
    }
}

// static
bool Initialize() {
    return true;
}

// static
void Display::Finalize() {
    LogMsg("DynDisplay::Finalize: Finalizing DynDisplay subsystem");
    if (bg_thread.get_id() == std::thread::id())
        return;

    LogMsg("Draining background thread for writing PNG files");

    {
        std::lock_guard<std::mutex> lock(q_mtx);
        while (!in_q.empty()) {
            Display* display = in_q.front();
            in_q.pop();
            if (!display->retired_) {
                LogMsg("Display %d was on the background queue but not retired", display->id_);
            } else
                delete display;
        }
    }

    bg_stop_thread = true;
    in_q_cv.notify_one();  // Wake up the thread to process any remaining displays

    std::this_thread::sleep_for(std::chrono::seconds(1));  // 1 s should be plenty
    bg_thread.join();

    while (!out_q.empty()) {
        Display* display = out_q.front();
        out_q.pop();
        if (!display->retired_) {
            LogMsg("Display %d was on the background queue but not retired", display->id_);
        } else
            delete display;
    }
}

// static
void Finalize() {
    Display::Finalize();
    // that's all for now
}

}  // namespace DynDisplay

} // namespace dgs

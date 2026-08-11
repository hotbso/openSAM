//
//    openSAM: manage DGS X Plane
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

#pragma once

#include <cassert>
#include <vector>
#include <unordered_map>
#include <string>
#include <memory>

#include "XPLMScenery.h"

#include "dgs_impl.h"
#include "log_msg.h"

struct stbtt_fontinfo;

namespace dgs {
namespace DynDisplay {

struct Color {
    unsigned char r, g, b, a;
};

class Font {

    // GlyphInfo is a wrapper around data OWNED BY glyph_cache_.
    // Don't be tempted to pass that around without considering all create/copy/move/delete semantics.
    struct GlyphInfo {
        unsigned char* bitmap = nullptr;
        float scale;
        int width;
        int height;
        int x_off;
        int y_off;
    };

    std::vector<unsigned char> font_buffer_;
    stbtt_fontinfo* font_info_ = nullptr;
    std::unordered_map<uint64_t, GlyphInfo> glyph_cache_;  // cache for the last glyph bitmap

   public:
    Font(const std::string& font_path);
    ~Font();

    float ScaleForPixelHeight(float height) const noexcept;
    void GetFontVMetrics(int& ascent, int& descent, int& line_gap) const noexcept;

    void GetCodepointHMetrics(int codepoint, int& advance, int& lsb) const noexcept;

    int GetCodepointKernAdvance(int left_codepoint, int right_codepoint) const noexcept;

    // return non-owning pointer.
    const unsigned char* GetCodepointBitmap(int codepoint, float scale_x, float scale_y, int& width, int& height,
                                            int& x_off, int& y_off);
};

class Image {
    std::unique_ptr<unsigned char[]> pixels_;
    int width_, height_;

   public:
    // create empty
    Image(int width, int height) : width_(width), height_(height) {
        pixels_ = std::make_unique<unsigned char[]>(width_ * height_ * 4);
   }

    // create from data
    Image(const unsigned char* data, int width, int height, int channels) : width_(width), height_(height) {
        assert(channels == 4);  // Ensure the input data has 4 channels (RGBA)
        pixels_ = std::make_unique_for_overwrite<unsigned char[]>(width_ * height_ * 4);
        memcpy(pixels_.get(), data, width_ * height_ * 4);
    }

    // create from file
    Image(const std::string& filename);
    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;
    Image(Image&&) = delete;
    Image& operator=(Image&&) = delete;

    void Text(int x, int y, Font& font, float height, Color color, const std::string& txt);
    int MeasureText(Font& font, float height, const std::string& txt);

    void Paste(const Image& src, int x, int y);  // paste src image into this image at (x,y)
    bool Save(const std::string& filename) const;

    int width() const { return width_; }
    int height() const { return height_; }
};

// a poor man's imgui for a dynamic display
class Display {
    static constexpr int kImgWidth = 1024;
    static constexpr int kImgHeight = kImgWidth;
    static constexpr float kMargin = 0.02f;  // Margin in meters

    const int id_;                    // unique ID for the display, used for naming the texture file
    static int active_displays_;

    bool baked_ = false;              // true if the display has been baked up to a file
    bool async_in_progress_ = false;  // true if XPLMLoadObjectAsync is in progress
    bool on_bg_q = false;             // true if the display is on the background thread queue
    bool retired_ = false;  // true if the display has been retired, will be deleted once the async loading is complete
    bool png_ok_ = false;   // true if the PNG file was successfully created in background thread
    std::string png_filename_;  // relative to the obj file, e.g. "X-eq-0001.png"
    std::string png_pathname_;  // relative to <xp-root>
    std::string obj_pathname_;

    float width_m_, height_m_;
    float x0_, y0_, z0_;        // lower left corner of the display in meters, local coord system of the display object
    float txt_x_{}, txt_y_{};  // 'cursor' position for next text, in meters from lower left corner
    float line_spacing_{};
    float prev_txt_x_{}, prev_txt_y_{};
    float m_2_pix, s_max_, t_max_;
    float txt_height_ = 0.1f;  // height of the font in meters
    float txt_scale_ = 1.0f;        // scale factor for the text and line spacing

    int r_margin_pix_;
    std::unique_ptr<Image> image_;
    XPLMObjectRef xplm_obj_ = nullptr;
    std::pair<int, int> ToPix(float x_m, float y_m) const {
        int xp = static_cast<int>(x_m * m_2_pix);
        int yp = static_cast<int>((1.0f - (y_m / height_m_)) * kImgHeight);
        return {xp, yp};
    }

    static void ObjectLoadedCb(XPLMObjectRef inObject, void* inRefcon);

    // x0, y0, z0 are the Blender coordinates of the lower left corner of the display in meters
    Display(float width_m, float height_m, float x0, float y0, float z0);
    ~Display();

    static void BgWorker();         // the actual worker

    void BakeBg();  // on background thread
    // Start async load of xp object after the PNG file has been created in the background thread
    void BakePhase2();  // on main thread

  public:
    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;
    Display(Display&&) = delete;
    Display& operator=(Display&&) = delete;

    // set 'cursor' position for next text line, in meters from lower left corner
    // -1 keeps previous value
    void SetPos(float x, float y) {
        if ( x >= 0.0f)
            txt_x_ = x;
        if (y >= 0.0f)
            txt_y_ = y;
    }

    void SetPosDelta(float dx, float dy) {
        txt_x_ += dx;
        txt_y_ += dy;
    }

    std::pair<float, float> GetPos() const { return {txt_x_, txt_y_}; }

    void SameLine() { txt_x_ = prev_txt_x_; txt_y_ = prev_txt_y_; }  // next text line will be on the same line as previous

    void SetTxtHeight(float height) { txt_height_ = height; }       // m
    void SetLineSpacing(float spacing) { line_spacing_ = spacing; } // m
    // scale factor for the text and line spacing, 1.0 = no scaling
    void SetScale(float scale) { txt_scale_ = scale; }

    // centered = true means center the text horizontally
    void TextLine(Color color, Font& font, const std::string& txt, bool centered = false);

    // place text at (x,y) in meters from lower left corner
    void TextAt(float x, float y, Color color, Font& font, const std::string& txt) {
        SetPos(x, y);
        TextLine(color, font, txt, false);
    }

    void Paste(const Image& src, float x, float y);  // paste src image into this image at (x,y) in meters from lower left corner

    void Bake(const std::string& dirname, const std::string& filename);  // without extension

    XPLMObjectRef GetXplmObj() const { return xplm_obj_; }

    static void Deleter(Display* display);

    // create a unique_ptr to a Display object, with a custom deleter for async processing
    static std::unique_ptr<Display, void(*)(Display*)> Create(float width_m, float height_m, float x0, float y0, float z0) {
        return std::unique_ptr<Display, void(*)(Display*)>(new Display(width_m, height_m, x0, y0, z0), &Deleter);
    }

    static void Finalize();  // call at plugin stop to clean up any remaining displays

    // call regularly in flight loop context to process any completed background tasks
    static void BgPostProcess();

    static int active_displays() { return active_displays_; }
};

using DisplayPtr = std::unique_ptr<Display, void(*)(Display*)>;

// the page is a generic class agnostic of the DGS implementation
class Page {
    XPLMDrawInfo_t& drawinfo_;

   protected:
    static const char* dlist_dr_[];  // dataref "opensam/dgs/vdgs_brightness"

    DynDisplay::DisplayPtr dd_{nullptr, DynDisplay::Display::Deleter};
    ObjInstRef inst_ref_;
    float ts_ = 0.0f;  // time when this page was shown

   public:
    Page(XPLMDrawInfo_t& drawinfo) : drawinfo_(drawinfo) {}
    Page(const Page&) = delete;
    Page(Page&&) = delete;
    Page& operator=(const Page&) = delete;
    Page& operator=(Page&&) = delete;

    virtual ~Page() {};

    bool Show();

    void Hide() { inst_ref_ = nullptr; }

    // called from from Clear() to allow a derived class to clear any user data it may have
    virtual void UserClearCb() {}

    // remove the page from the display and delete the display
    void Clear() {
        inst_ref_ = nullptr;
        dd_ = nullptr;
        UserClearCb();
    }

    void SetPosition();
    virtual bool Update() = 0;  // return true if the page is still active
};
using PagePtr = std::unique_ptr<Page>;

extern bool Initialize();
extern void Finalize();

} // namespace DynDisplay

}  // namespace dgs

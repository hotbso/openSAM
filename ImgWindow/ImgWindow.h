//
// ImgWindow.h
//
// Integration for dear imgui into X-Plane.
//
// Copyright (C) 2018,2020 Christopher Collins
// Copyright (C) 2026, Holger Teutsch
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#pragma once

#include <climits>
#include <string>
#include <memory>
#include <vector>

#include <XPLMDisplay.h>
#include <XPLMProcessing.h>
#include <XPLMPanelGraphics.h>

#include <imgui.h>

class ImgWindow {
   public:
    static bool Initialize();
    static void Finalize();
    static ImFontAtlas* GetSharedFontAtlas() { return shared_font_atlas_; }

    virtual ~ImgWindow();

    void GetWindowGeometry(int& left, int& top, int& right, int& bottom) const noexcept;
    void SetWindowGeometry(int left, int top, int right, int bottom) noexcept;
    void GetWindowGeometryOS(int& left, int& top, int& right, int& bottom) const noexcept;
    void SetWindowGeometryOS(int left, int top, int right, int bottom) noexcept;
    void GetWindowGeometryVR(int& width, int& height) const noexcept;
    void SetWindowGeometryVR(int width, int height) noexcept;
    void GetCurrentWindowGeometry(int& left, int& top, int& right, int& bottom) const;
    void SetWindowResizingLimits(int minW, int minH, int maxW, int maxH) noexcept;

    virtual void SetVisible(bool inIsVisible);
    bool GetVisible() const noexcept;
    bool IsPoppedOut() const noexcept;
    bool IsInVR() const noexcept;
    bool IsInsideSim() const;

    void SetWindowPositioningMode(XPLMWindowPositioningMode inPosMode, int inMonitorIdx = -1) noexcept;
    void BringWindowToFront() noexcept;
    bool IsWindowInFront() const noexcept;

    void SetWindowDragArea(int left = 0, int top = 0, int right = INT_MAX, int bottom = INT_MAX);
    void ClearWindowDragArea();
    bool HasWindowDragArea(int* pL = nullptr, int* pT = nullptr, int* pR = nullptr, int* pB = nullptr) const;
    bool IsInsideWindowDragArea(int x, int y) const;

    // called pre rendering, in flight loop context.
    // Actually this is not really needed as BuildInterface() is called in flight loop ctx
    virtual void FlightLoopUserCb() noexcept {
        // default: do nothing
    }

   protected:
    const int id_;      // serial no. for logging
    bool first_render_;

    ImgWindow(int left, int top, int right, int bottom,
              XPLMWindowDecoration decoration = xplm_WindowDecorationRoundRectangle,
              XPLMWindowLayer layer = xplm_WindowLayerFloatingWindows);

    ImgWindow(const ImgWindow&) = delete;
    ImgWindow(ImgWindow&&) = delete;
    ImgWindow& operator=(const ImgWindow&) = delete;
    ImgWindow& operator=(ImgWindow&&) = delete;

    void SetWindowTitle(const std::string& title);

    void MoveForVR();

    virtual ImGuiWindowFlags_ BeforeBegin() { return ImGuiWindowFlags_None; }

    // Build the ImGui interface for the window. *This is called in the flight loop context!*
    virtual void BuildInterface() = 0;

    virtual void AfterRendering() {}

    virtual bool OnShow();

    // Hhhm, what is the use case here?
    // It won't work for any window that has some external reference that will be left behind dangling.
    void SafeDelete();

    XPLMWindowID GetWindowId() const { return window_id_; }

   private:
    static void UpdateTexture(ImTextureData* tex);

    static int HandleMouseClickCB(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon);
    static void HandleKeyFuncCB(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey,
                                void* inRefcon, int losingFocus);
    static XPLMCursorStatus HandleCursorFuncCB(XPLMWindowID inWindowID, int x, int y, void* inRefcon);
    static int HandleMouseWheelFuncCB(XPLMWindowID inWindowID, int x, int y, int wheel, int clicks, void* inRefcon);
    static int HandleRightClickFuncCB(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon);

    // states for the draw pass, consider that as coroutines switching between flight loop and draw callback contexts.
    enum State {
        kPreDraw,  // run imgui, prep textures, in flight loop ctx
        kDraw,     // draw pass, emit the draw calls, in draw callback ctx
        kPostDraw  // cleanup, flight loop ctx
    };

    State state_ = kPreDraw;

    std::vector<XPLMDrawCall_t> draw_calls_;
    void DrawPass();

    // one global flight loop for all windows
    static float XPFlightLoopCb(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop,
                                int inCounter, void* inRefcon);

    bool FlightLoopCb();    // -> window is visible
    static void DrawWindowCB(XPLMWindowID inWindowID, void* inRefcon);

    int HandleMouseClickGeneric(int x, int y, XPLMMouseStatus inMouse, int button = 0);
    void UpdateImgui();
    void TranslateImguiToBoxel(float inX, float inY, int& outX, int& outY);
    void TranslateToImguiSpace(int inX, int inY, float& outX, float& outY);

    static ImFontAtlas* shared_font_atlas_;
    static ImGuiContext* global_context_;

    std::string window_title_;

    XPLMWindowID window_id_;
    ImGuiContext* imgui_context_;

    int top_, bottom_, left_, right_;

    XPLMWindowLayer preferred_layer_;

    bool reset_backspace_ = false;
    const bool handle_wnd_resize_;

    int min_width_ = 100;
    int min_height_ = 100;
    int max_width_ = INT_MAX;
    int max_height_ = INT_MAX;

    int drag_left_ = -1;
    int drag_top_ = -1;
    int drag_right_ = -1;
    int drag_bottom_ = -1;

    int last_mouse_drag_x_ = -1;
    int last_mouse_drag_y_ = -1;

    struct DragTy {
        bool wnd : 1;
        bool left : 1;
        bool top : 1;
        bool right : 1;
        bool bottom : 1;

        DragTy() { clear(); }
        void clear() { wnd = left = top = right = bottom = false; }
        operator bool() const { return wnd || left || top || right || bottom; }
    } drag_what_;
};

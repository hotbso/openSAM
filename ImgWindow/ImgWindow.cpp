//
// ImgWindow.cpp
//
// Integration for dear imgui into X-Plane.
//
// Copyright (C) 2018,2020, Christopher Collins
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

#include <string>
#include <vector>
#include <unordered_map>

#include <XPLMDataAccess.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMPanelGraphics.h>

#include "ImgWindow.h"
#include "imgui_internal.h"

#include "log_msg.h"

// size of "frame" around a resizable window, by which its size can be changed
constexpr int kWndResizeLeftWidth = 15;
constexpr int kWndResizeTopWidth = 5;
constexpr int kWndResizeRightWidth = 15;
constexpr int kWndResizeBottomWidth = 15;

static XPLMDataRef vr_enabled_dr = nullptr;
static XPLMDataRef frame_rate_period_dr = nullptr;

ImFontAtlas* ImgWindow::shared_font_atlas_ = nullptr;
ImGuiContext* ImgWindow::global_context_ = nullptr;
static int id_base = 0;

static std::unordered_map<ImgWindow*, bool> active_window_map_; // ptr -> visible
static XPLMFlightLoopID fltl_id = nullptr;
static bool fl_running = false;
static std::vector<ImgWindow*> pending_destruction;

static ImGuiKey TranslateXPLMKeyToImGui(unsigned char inVirtualKey) {
    switch (inVirtualKey) {
        case XPLM_VK_TAB:
            return ImGuiKey_Tab;
        case XPLM_VK_LEFT:
            return ImGuiKey_LeftArrow;
        case XPLM_VK_RIGHT:
            return ImGuiKey_RightArrow;
        case XPLM_VK_UP:
            return ImGuiKey_UpArrow;
        case XPLM_VK_DOWN:
            return ImGuiKey_DownArrow;
        case XPLM_VK_PRIOR:
            return ImGuiKey_PageUp;
        case XPLM_VK_NEXT:
            return ImGuiKey_PageDown;
        case XPLM_VK_HOME:
            return ImGuiKey_Home;
        case XPLM_VK_END:
            return ImGuiKey_End;
        case XPLM_VK_INSERT:
            return ImGuiKey_Insert;
        case XPLM_VK_DELETE:
            return ImGuiKey_Delete;
        case XPLM_VK_BACK:
            return ImGuiKey_Backspace;
        case XPLM_VK_SPACE:
            return ImGuiKey_Space;
        case XPLM_VK_RETURN:
            return ImGuiKey_Enter;
        case XPLM_VK_ESCAPE:
            return ImGuiKey_Escape;
        case XPLM_VK_ENTER:
            return ImGuiKey_KeypadEnter;
        case XPLM_VK_NUMPAD0:
            return ImGuiKey_Keypad0;
        case XPLM_VK_NUMPAD1:
            return ImGuiKey_Keypad1;
        case XPLM_VK_NUMPAD2:
            return ImGuiKey_Keypad2;
        case XPLM_VK_NUMPAD3:
            return ImGuiKey_Keypad3;
        case XPLM_VK_NUMPAD4:
            return ImGuiKey_Keypad4;
        case XPLM_VK_NUMPAD5:
            return ImGuiKey_Keypad5;
        case XPLM_VK_NUMPAD6:
            return ImGuiKey_Keypad6;
        case XPLM_VK_NUMPAD7:
            return ImGuiKey_Keypad7;
        case XPLM_VK_NUMPAD8:
            return ImGuiKey_Keypad8;
        case XPLM_VK_NUMPAD9:
            return ImGuiKey_Keypad9;

        // these are used to generate events like Ctrl-C, Ctrl-V, Ctrl-X, Ctrl-Z, etc. in the X-Plane key handler
        case XPLM_VK_A:
            return ImGuiKey_A;
        case XPLM_VK_C:
            return ImGuiKey_C;
        case XPLM_VK_V:
            return ImGuiKey_V;
        case XPLM_VK_X:
            return ImGuiKey_X;
        case XPLM_VK_Y:
            return ImGuiKey_Y;
        case XPLM_VK_Z:
            return ImGuiKey_Z;
        case XPLM_VK_0:
            return ImGuiKey_0;
        case XPLM_VK_1:
            return ImGuiKey_1;
        case XPLM_VK_2:
            return ImGuiKey_2;
        case XPLM_VK_3:
            return ImGuiKey_3;
        case XPLM_VK_4:
            return ImGuiKey_4;
        case XPLM_VK_5:
            return ImGuiKey_5;
        case XPLM_VK_6:
            return ImGuiKey_6;
        case XPLM_VK_7:
            return ImGuiKey_7;
        case XPLM_VK_8:
            return ImGuiKey_8;
        case XPLM_VK_9:
            return ImGuiKey_9;
    }
    return ImGuiKey_None;
}

ImgWindow::ImgWindow(int left, int top, int right, int bottom, XPLMWindowDecoration decoration, XPLMWindowLayer layer)
    : id_(id_base++), first_render_(true),
      preferred_layer_(layer),
      handle_wnd_resize_(xplm_WindowDecorationSelfDecoratedResizable == decoration) {
    IM_ASSERT(shared_font_atlas_ != nullptr &&
              "ImgWindow::ImgWindow: shared_font_atlas_ is nullptr, call ImgWindowLoadFonts() first");

    XPLMCreateWindow_t windowParams = {sizeof(windowParams),
                                       left,
                                       top,
                                       right,
                                       bottom,
                                       0,
                                       DrawWindowCB,
                                       HandleMouseClickCB,
                                       HandleKeyFuncCB,
                                       NULL,  // HandleCursorFuncCB
                                       HandleMouseWheelFuncCB,
                                       reinterpret_cast<void*>(this),
                                       decoration,
                                       layer,
                                       HandleRightClickFuncCB,
                                       xplm_WindowContentTypePanelGraphics,
                                       nullptr,
                                       nullptr};

    window_id_ = XPLMCreateWindowEx(&windowParams);
    draw_calls_.reserve(50);  // reserve some space to avoid reallocations

    imgui_context_ = ImGui::CreateContext(shared_font_atlas_);
    ImGui::SetCurrentContext(imgui_context_);
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // is not compatible with imgWindow, disable imgui.ini file

    // disable window rounding since we're not rendering the frame anyway.
    auto& style = ImGui::GetStyle();
    style.WindowRounding = 0;

    // disable OSX-like keyboard behaviours always - we don't have the keymapping for it.
    io.ConfigMacOSXBehaviors = false;

    // try to inhibit a few resize/move behaviours that won't play nice with our window control.
    io.ConfigWindowsResizeFromEdges = false;
    io.ConfigWindowsMoveFromTitleBarOnly = true;

    io.BackendFlags |=
        ImGuiBackendFlags_RendererHasTextures;  // We can honor ImGuiPlatformIO::Textures[] requests during render.
    // keep the current context so the constructor of the derived class can use ImGui functions to set up the interface.

    active_window_map_[this] = true;
}

ImgWindow::~ImgWindow() {
    XPLMDestroyWindow(window_id_);
    LogMsg("draw_calls_.capacity(): %zu", draw_calls_.capacity());

    ImGui::SetCurrentContext(imgui_context_);

    LogMsg("ImgWindow::~ImgWindow: destroying ImGui textures");
    for (ImTextureData* tex : ImGui::GetPlatformIO().Textures)
        if (tex->RefCount == 1) {
            tex->SetStatus(ImTextureStatus_WantDestroy);
            UpdateTexture(tex);
        }
    LogMsg("ImgWindow::~ImgWindow: destroying ImGui context %p", (void*)imgui_context_);
    ImGui::DestroyContext(imgui_context_);
    active_window_map_.erase(this);
}

void ImgWindow::GetWindowGeometry(int& left, int& top, int& right, int& bottom) const noexcept {
    XPLMGetWindowGeometry(window_id_, &left, &top, &right, &bottom);
}

void ImgWindow::SetWindowGeometry(int left, int top, int right, int bottom) noexcept {
    XPLMSetWindowGeometry(window_id_, left, top, right, bottom);
}

void ImgWindow::GetWindowGeometryOS(int& left, int& top, int& right, int& bottom) const noexcept {
    XPLMGetWindowGeometryOS(window_id_, &left, &top, &right, &bottom);
}

void ImgWindow::SetWindowGeometryOS(int left, int top, int right, int bottom) noexcept {
    XPLMSetWindowGeometryOS(window_id_, left, top, right, bottom);
}

void ImgWindow::GetWindowGeometryVR(int& width, int& height) const noexcept {
    XPLMGetWindowGeometryVR(window_id_, &width, &height);
}

void ImgWindow::SetWindowGeometryVR(int width, int height) noexcept { XPLMSetWindowGeometryVR(window_id_, width, height); }

bool ImgWindow::IsPoppedOut() const noexcept { return XPLMWindowIsPoppedOut(window_id_) != 0; }

bool ImgWindow::IsInVR() const noexcept { return XPLMWindowIsInVR(window_id_) != 0; }

bool ImgWindow::IsInsideSim() const { return !IsPoppedOut() && !IsInVR(); }

void ImgWindow::SetWindowPositioningMode(XPLMWindowPositioningMode inPosMode, int inMonitorIdx) noexcept {
    XPLMSetWindowPositioningMode(window_id_, inPosMode, inMonitorIdx);
}

void ImgWindow::BringWindowToFront() noexcept { XPLMBringWindowToFront(window_id_); }

bool ImgWindow::IsWindowInFront() const noexcept { return XPLMIsWindowInFront(window_id_) != 0; }

void ImgWindow::GetCurrentWindowGeometry(int& left, int& top, int& right, int& bottom) const {
    if (IsPoppedOut())
        GetWindowGeometryOS(left, top, right, bottom);
    else if (IsInVR()) {
        left = bottom = 0;
        GetWindowGeometryVR(right, top);
    } else {
        GetWindowGeometry(left, top, right, bottom);
    }
}

void ImgWindow::SetWindowResizingLimits(int minW, int minH, int maxW, int maxH) noexcept {
    min_width_ = minW;
    min_height_ = minH;
    max_width_ = maxW;
    max_height_ = maxH;
    XPLMSetWindowResizingLimits(window_id_, minW, minH, maxW, maxH);
}

// static
void ImgWindow::UpdateTexture(ImTextureData* tex) {
    if (tex->Status == ImTextureStatus_WantCreate) {
        const unsigned char* pixels = static_cast<const unsigned char*>(tex->GetPixels());

        void* pg_tex_id = XPLMCreateTexture(pixels, tex->Width, tex->Height);
        tex->SetTexID((ImTextureID)(intptr_t)pg_tex_id);  // specify backend-specific ImTextureID identifier
        tex->SetStatus(ImTextureStatus_OK);
        // LogMsg("ImgWindow::UpdateTexture: Created texture %p for ImTextureData %p", pg_tex_id, (void*)tex);
    }

    if (tex->Status == ImTextureStatus_WantUpdates) {
        // I assume update is not supported by X-Plane, so we destroy and recreate the texture instead.
        void* pg_tex_id = (void*)(intptr_t)tex->GetTexID();
        if (pg_tex_id) {
            XPLMDestroyTexture(pg_tex_id);
            // LogMsg("ImgWindow::UpdateTexture: Destroyed texture %p for ImTextureData %p", pg_tex_id, (void*)tex);
        }
        const unsigned char* pixels = static_cast<const unsigned char*>(tex->GetPixels());
        pg_tex_id = XPLMCreateTexture(pixels, tex->Width, tex->Height);
        tex->SetTexID((ImTextureID)(intptr_t)pg_tex_id);  // specify backend-specific ImTextureID identifier
        tex->SetStatus(ImTextureStatus_OK);
        // LogMsg("ImgWindow::UpdateTexture: Created texture %p for ImTextureData %p", pg_tex_id, (void*)tex);
    }

    if (tex->Status == ImTextureStatus_WantDestroy) {
        void* pg_tex_id = (void*)(intptr_t)tex->GetTexID();
        if (pg_tex_id) {
            XPLMDestroyTexture(pg_tex_id);
            // LogMsg("ImgWindow::UpdateTexture: Destroyed texture %p for ImTextureData %p", pg_tex_id, (void*)tex);
        }

        tex->SetTexID(ImTextureID_Invalid);
        tex->SetStatus(ImTextureStatus_Destroyed);
    }
}

void ImgWindow::TranslateToImguiSpace(int inX, int inY, float& outX, float& outY) {
    outX = static_cast<float>(inX - left_);
    if (outX < 0.0f || outX > (float)(right_ - left_)) {
        outX = -FLT_MAX;
        outY = -FLT_MAX;
        return;
    }
    outY = static_cast<float>(top_ - inY);
    if (outY < 0.0f || outY > (float)(top_ - bottom_)) {
        outX = -FLT_MAX;
        outY = -FLT_MAX;
        return;
    }
}

void ImgWindow::TranslateImguiToBoxel(float inX, float inY, int& outX, int& outY) {
    outX = (int)(left_ + inX);
    outY = (int)(top_ - inY);
}

void ImgWindow::UpdateImgui() {
    // if we use a shared font atlas, we need to update the textures before each frame.
    if (shared_font_atlas_) {
        ImGuiContext* ctx = shared_font_atlas_->OwnerContext;
        ctx->FrameCount++;
        ImGui::SetCurrentContext(ctx);
        // ImGuiIO& io = ImGui::GetIO();
        ImFontAtlasUpdateNewFrame(shared_font_atlas_, ctx->FrameCount, true);
    }

    ImGui::SetCurrentContext(imgui_context_);
    auto& io = ImGui::GetIO();

    // transfer the window geometry to ImGui
    XPLMGetWindowGeometry(window_id_, &left_, &top_, &right_, &bottom_);

    float win_width = static_cast<float>(right_ - left_);
    float win_height = static_cast<float>(top_ - bottom_);

    // Needed to add this to prevent io.DeltaTime causing a CTD because when X-Plane starts FrameRatePeriod is equal
    // to 0.0f
    float FrameRatePeriod = XPLMGetDataf(frame_rate_period_dr);
    if (FrameRatePeriod > 0.0f) {
        io.DeltaTime = XPLMGetDataf(frame_rate_period_dr);
    }
    io.DisplaySize = ImVec2(win_width, win_height);
    // in boxels, we're always scale 1, 1.
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2((float)0.0, (float)0.0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(win_width, win_height), ImGuiCond_Always);

    // and construct the window
    ImGui::Begin(window_title_.c_str(), nullptr,
                 BeforeBegin() | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    BuildInterface();
    ImGui::End();

    // finally, handle window focus.
    int hasKeyboardFocus = XPLMHasKeyboardFocus(window_id_);
    if (io.WantTextInput && !hasKeyboardFocus) {
        XPLMTakeKeyboardFocus(window_id_);
    } else if (!io.WantTextInput && hasKeyboardFocus) {
        XPLMTakeKeyboardFocus(nullptr);
        // reset keysdown otherwise we'll think any keys used to defocus the keyboard are still down!
        io.ClearInputKeys();
    }

    // X-Plane does not make a reliable callback when the mouse leaves a Window so we query the mouse here and feed
    // it to ImGui.
    int m_x, m_y;
    XPLMGetMouseLocationGlobal(&m_x, &m_y);
    float outX, outY;
    TranslateToImguiSpace(m_x, m_y, outX, outY);
    io.AddMousePosEvent(outX, outY);

    first_render_ = false;
}

void ImgWindow::DrawPass() {
    // LogMsg("ImgWindow::DrawPass: window %d, state %d", id_, state_);

    // runs in flight loop ctx
    if (state_ == kPreDraw) {
        UpdateImgui();

        ImGui::SetCurrentContext(imgui_context_);
        ImGui::Render();

        auto draw_data = ImGui::GetDrawData();
        if (draw_data->Textures != nullptr)
            for (ImTextureData* tex : *draw_data->Textures)
                if (tex->Status != ImTextureStatus_OK)
                    UpdateTexture(tex);

        state_ = kDraw;
        return;     // cocall to next draw callback for drawing
    }

    // runs in draw callback ctx
    if (state_ == kDraw) {
        ImGui::SetCurrentContext(imgui_context_);
        ImGuiIO& io = ImGui::GetIO();

        auto draw_data = ImGui::GetDrawData();
        // may be multiple windows fought against each other for the shared font atlas, so we need to check if all
        // textures are ready before we can draw.
        if (draw_data->Textures != nullptr)
            for (ImTextureData* tex : *draw_data->Textures)
                if (tex->Status != ImTextureStatus_OK) {
                    state_ = kPostDraw;     // skip the draw pass, textures are not ready yet
                    return;
                }

        if (io.DisplayFramebufferScale.x != 1.0 || io.DisplayFramebufferScale.y != 1.0) {
            draw_data->ScaleClipRects(io.DisplayFramebufferScale);
        }

        for (int n = 0; n < draw_data->CmdListsCount; n++) {
            // LogMsg("ImgWindow::DrawPass: processing draw list %d of %d", n, draw_data->CmdListsCount);
            const ImDrawList* cmd_list = draw_data->CmdLists[n];
            const ImDrawVert* vtx_buffer = cmd_list->VtxBuffer.Data;
            const ImDrawIdx* idx_buffer = cmd_list->IdxBuffer.Data;

            XPLMMesh_t mesh;
            mesh.vertex_count = cmd_list->VtxBuffer.Size;
            mesh.vertices = (const float*)vtx_buffer;
            mesh.index_count = cmd_list->IdxBuffer.Size;
            mesh.indices = idx_buffer;

            int idx_ofs = 0;
            draw_calls_.clear();
            for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++) {
                const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
                XPLMDrawCall_t drc;
                drc.tex_ref = (void*)(intptr_t)pcmd->GetTexID();
                drc.scissors[0] = pcmd->ClipRect.x;
                drc.scissors[1] = pcmd->ClipRect.y;
                drc.scissors[2] = pcmd->ClipRect.z;
                drc.scissors[3] = pcmd->ClipRect.w;
                drc.idx_offset = idx_ofs;
                drc.element_count = pcmd->ElemCount;
                drc.vtx_offset = 0;  // since we're using a single mesh for the entire draw list
                draw_calls_.push_back(drc);
                idx_ofs += pcmd->ElemCount;
            }

            XPLMTransformPush();
            XPLMTransformTranslate(left_, top_);
            XPLMTransformScale(1.0f, -1.0f);    // ImGui is Y-down
            XPLMDrawCalls(&mesh, draw_calls_.size(), draw_calls_.data());
            XPLMTransformPop();
        }

        // Give subclasses a chance to do something after all rendering
        AfterRendering();
        state_ = kPostDraw;
        return;     // cocall to next flight loop callback for cleanup
    }

    // runs in flight loop ctx
    if (state_ == kPostDraw) {
        // Hack: Reset the Backspace key if in VR (see HandleKeyFuncCB for details)
        if (reset_backspace_) {
            ImGuiIO& io = ImGui::GetIO();
            io.AddKeyEvent(ImGuiKey_Backspace, false);
            reset_backspace_ = false;
        }
        state_ = kPreDraw;
        return;     // stays in flight loop callback.
    }

    assert(false && "ImgWindow::DrawPass: invalid state");
}

void ImgWindow::DrawWindowCB([[maybe_unused]] XPLMWindowID inWindowID, void* inRefcon) {
    auto* iw = reinterpret_cast<ImgWindow*>(inRefcon);
    active_window_map_[iw] = true;

    if (!fl_running) {
        // obviously the window is visible so we kick off the flight loop to do the actual drawing.
        // LogMsg("ImgWindow::DrawWindowCB: window %d, scheduled flight loop", iw->id_);
        XPLMScheduleFlightLoop(fltl_id, -1.0f, 1);  // schedule the flight loop to run immediately
        fl_running = true;
        iw->state_ = kPreDraw;
        return;
    }

    if (iw->state_ == kDraw) {
        iw->DrawPass();
        // return;
    }
}

// run stuff that is not allowed in the draw context, like texture updates.
bool ImgWindow::FlightLoopCb() {
    // LogMsg("ImgWindow::XPFlightLoopCb window %d, state %d", id_, state_);
    if (!GetVisible()) {
        // LogMsg("ImgWindow::XPFlightLoopCb window %d: window not visible, unscheduling flight loop", id_);
        state_ = kPreDraw;
        active_window_map_[this] = false;
        return false;  // unschedule the flight loop if the window is not visible
    }

    if (state_ == kDraw)    // obviously we missed a draw CB, just skip it
        state_ = kPostDraw;

    // cleanup of the previous draw pass.
    if (state_ == kPostDraw)
        DrawPass();

    if (state_ == kPreDraw) {
        FlightLoopUserCb();
        DrawPass();
    }

    return true;
}

// static
float ImgWindow::XPFlightLoopCb([[maybe_unused]] float inElapsedSinceLastCall,
                                [[maybe_unused]] float inElapsedTimeSinceLastFlightLoop, [[maybe_unused]] int inCounter,
                                [[maybe_unused]] void* inRefcon) {
    for (ImgWindow* iw : pending_destruction) {
        // LogMsg("ImgWindow::XPFlightLoopCb: destroying window %d", iw->id_);
        delete iw;
    }
    pending_destruction.clear();

    bool have_active_window = false;
    for (auto& [window, visible] : active_window_map_) {
        if (!visible)
            continue;
        if (window->FlightLoopCb())
            have_active_window = true;
    }

    if (!have_active_window) {
        // LogMsg("ImgWindow::XPFlightLoopCb: no active windows, unscheduling flight loop");
        fl_running = false;
        return 0;  // unschedule the flight loop if there are no active windows
    }

    return -1.0f;
}

int ImgWindow::HandleMouseClickCB(XPLMWindowID /* inWindowID */, int x, int y, XPLMMouseStatus inMouse,
                                  void* inRefcon) {
    auto* thisWindow = reinterpret_cast<ImgWindow*>(inRefcon);
    return thisWindow->HandleMouseClickGeneric(x, y, inMouse, 0);
}

int ImgWindow::HandleMouseClickGeneric(int x, int y, XPLMMouseStatus inMouse, int button) {
    ImGui::SetCurrentContext(imgui_context_);
    ImGuiIO& io = ImGui::GetIO();

    // Tell ImGui the mous position relative to the window
    float outX, outY;
    TranslateToImguiSpace(x, y, outX, outY);
    io.AddMousePosEvent(outX, outY);
    const int loc_x = int(outX);  // local x, relative to top/left corner
    const int loc_y = int(outY);
    const int dx = x - last_mouse_drag_x_;  // dragged how far since last down/drag event?
    const int dy = y - last_mouse_drag_y_;

    auto modifiers = XPLMGetModifierKeys();
    bool shift = (modifiers & xplm_ShiftFlag);
    bool ctrl = (modifiers & xplm_ControlFlag);

    switch (inMouse) {
        case xplm_MouseDrag:
            io.AddMouseButtonEvent(button, true);

            // Any kind of self-dragging/resizing only happens with a floating window in the sim
            if (button == 0 &&    // left button
                IsInsideSim() &&  // floating window in sim
                drag_what_ &&     // and if there actually _is_ dragging
                (dx != 0 || dy != 0)) {
                // shall we drag the entire window?
                if (drag_what_.wnd) {
                    left_ += dx;  // move the wdinow
                    right_ += dx;
                    top_ += dy;
                    bottom_ += dy;
                } else {
                    // do we need to handle window resize?
                    if (drag_what_.left)
                        left_ += dx;
                    if (drag_what_.top)
                        top_ += dy;
                    if (drag_what_.right)
                        right_ += dx;
                    if (drag_what_.bottom)
                        bottom_ += dy;

                    // Make sure resizing limits are honored
                    if (right_ - left_ < min_width_) {
                        if (drag_what_.left)
                            left_ = right_ - min_width_;
                        else
                            right_ = left_ + min_width_;
                    }
                    if (right_ - left_ > max_width_) {
                        if (drag_what_.left)
                            left_ = right_ - max_width_;
                        else
                            right_ = left_ + max_width_;
                    }
                    if (top_ - bottom_ < min_height_) {
                        if (drag_what_.top)
                            top_ = bottom_ + min_height_;
                        else
                            bottom_ = top_ - min_height_;
                    }
                    if (top_ - bottom_ > max_height_) {
                        if (drag_what_.top)
                            top_ = bottom_ + max_height_;
                        else
                            bottom_ = top_ - max_height_;
                    }
                    // FIXME: If we had to apply resizing restricitons, then mouse and window frame will now be out
                    // of synch
                }

                // Change window geometry
                SetWindowGeometry(left_, top_, right_, bottom_);
                // now that the window has moved under the mouse we need to update relative mouse pos
                float newOutX, newOutY;
                TranslateToImguiSpace(x, y, newOutX, newOutY);
                io.AddMousePosEvent(newOutX, newOutY);
                // Update the last handled position
                last_mouse_drag_x_ = x;
                last_mouse_drag_y_ = y;
            }
            break;

        case xplm_MouseDown:
            if (ctrl)
                io.AddKeyEvent(ImGuiMod_Ctrl, true);
            if (shift)
                io.AddKeyEvent(ImGuiMod_Shift, true);

            io.AddMouseButtonEvent(button, true);

            // Which part of the window would we drag, if any?
            drag_what_.clear();
            if (button == 0 &&             // left button
                IsInsideSim() &&           // floating window in simulator
                loc_x >= 0 && loc_y >= 0)  // valid local position
            {
                // shall we drag the entire window?
                if (IsInsideWindowDragArea(loc_x, loc_y)) {
                    drag_what_.wnd = true;
                }
                // do we need to handle window resize?
                else if (handle_wnd_resize_) {
                    drag_what_.left = loc_x <= kWndResizeLeftWidth;
                    drag_what_.top = loc_y <= kWndResizeTopWidth;
                    drag_what_.right = loc_x >= (right_ - left_) - kWndResizeRightWidth;
                    drag_what_.bottom = loc_y >= (top_ - bottom_) - kWndResizeBottomWidth;
                }
                // Anything to drag?
                if (drag_what_) {
                    // Remember pos in case of dragging
                    last_mouse_drag_x_ = x;
                    last_mouse_drag_y_ = y;
                }
            }
            break;

        case xplm_MouseUp:
            io.AddMouseButtonEvent(button, false);
            last_mouse_drag_x_ = last_mouse_drag_y_ = -1;
            drag_what_.clear();
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
            io.AddKeyEvent(ImGuiMod_Shift, false);
            break;
        default:
            // dunno!
            break;
    }

    return 1;
}

void ImgWindow::HandleKeyFuncCB(XPLMWindowID /*inWindowID*/, char inKey, XPLMKeyFlags inFlags, char inVirtualKey,
                                void* inRefcon, int blosingFocus) {
    // LogMsg("ImgWindow::HandleKeyFuncCB: inKey=%d, inFlags=%08x, inVirtualKey=%d, blosingFocus=%d", (unsigned)inKey,
    //        (unsigned)inFlags, (unsigned)inVirtualKey, blosingFocus);
    auto* thisWindow = reinterpret_cast<ImgWindow*>(inRefcon);
    ImGui::SetCurrentContext(thisWindow->imgui_context_);
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) {
        // Loosing focus? That's not exactly something ImGui allows us to do...
        // we try convincing ImGui to let it go by sending an [Esc] key
        if (blosingFocus) {
            io.AddKeyEvent(ImGuiKey_Escape, true);
            io.AddKeyEvent(ImGuiKey_Escape, false);
        } else {
            // Hack for the Backspace key in VR:
            // Apparently, the virtual VR keyboard sends both the Up and the Down
            // event within the same drawing cycle, which would overwrite
            // io.KeyDown[XPLM_VK_BACK] with false again before we could pass on true.
            // Also see
            // https://forums.x-plane.org/index.php?/forums/topic/147139-dear-imgui-x-plane/&do=findComment&comment=2032062
            // though I am following a different solution:
            // So we ignore the "up" event (release key) here, and do the actual
            // release only after the next drawing cycle (flag reset_backspace_).
            // (And this little delay doesn't hurt in non-VR either, so we don't even test for VR.)

            // If Backspace is _released_ ...
            if (inVirtualKey == XPLM_VK_BACK && !(inFlags & xplm_DownFlag)) {
                thisWindow->reset_backspace_ = true;  // have it reset only later in DrawWindowCB
            } else {
                io.AddKeyEvent(ImGuiMod_Shift, (inFlags & xplm_ShiftFlag) == xplm_ShiftFlag);
                io.AddKeyEvent(ImGuiMod_Alt, (inFlags & xplm_OptionAltFlag) == xplm_OptionAltFlag);
                io.AddKeyEvent(ImGuiMod_Ctrl, (inFlags & xplm_ControlFlag) == xplm_ControlFlag);

                // in all normal cases: save the up/down flag as it comes from XP
                ImGuiKey key = TranslateXPLMKeyToImGui(static_cast<unsigned char>(inVirtualKey));
                if (key != ImGuiKey_None)
                    io.AddKeyEvent(key, (inFlags & xplm_DownFlag) == xplm_DownFlag);

                // inKey will only include ASCII printable characters,
                if ((inFlags & xplm_DownFlag) == xplm_DownFlag && inKey > '\0')
                    io.AddInputCharacter(inKey);
            }
        }
    }
}

int ImgWindow::HandleMouseWheelFuncCB(XPLMWindowID /*inWindowID*/, int x, int y, int wheel, int clicks,
                                      void* inRefcon) {
    auto* thisWindow = reinterpret_cast<ImgWindow*>(inRefcon);
    ImGui::SetCurrentContext(thisWindow->imgui_context_);
    ImGuiIO& io = ImGui::GetIO();

    float outX, outY;
    thisWindow->TranslateToImguiSpace(x, y, outX, outY);
    io.AddMousePosEvent(outX, outY);
    switch (wheel) {
        case 0:
            io.AddMouseWheelEvent(0.0f, static_cast<float>(clicks));
            break;
        case 1:
            io.AddMouseWheelEvent(static_cast<float>(clicks), 0.0f);
            break;
        default:
            // unknown wheel
            break;
    }
    return 1;
}

int ImgWindow::HandleRightClickFuncCB(XPLMWindowID /* inWindowID */, int x, int y, XPLMMouseStatus inMouse,
                                      void* inRefcon) {
    auto* thisWindow = reinterpret_cast<ImgWindow*>(inRefcon);
    return thisWindow->HandleMouseClickGeneric(x, y, inMouse, 1);
}

void ImgWindow::SetWindowTitle(const std::string& title) {
    window_title_ = title;
    XPLMSetWindowTitle(window_id_, window_title_.c_str());
}

void ImgWindow::SetVisible(bool inIsVisible) {
    if (inIsVisible)
        MoveForVR();
    if (GetVisible() == inIsVisible) {
        // if the state is already correct, no-op.
        return;
    }
    if (inIsVisible) {
        if (!OnShow()) {
            // chance to early abort.
            return;
        }
    }
    XPLMSetWindowIsVisible(window_id_, inIsVisible);
}

void ImgWindow::MoveForVR() {
    // if we're trying to display the window, check the state of the VR flag
    // - if we're VR enabled, explicitly move the window to the VR world.
    if (XPLMGetDatai(vr_enabled_dr)) {
        XPLMSetWindowPositioningMode(window_id_, xplm_WindowVR, 0);
    } else {
        if (IsInVR()) {
            XPLMSetWindowPositioningMode(window_id_, preferred_layer_, -1);
        }
    }
}

bool ImgWindow::GetVisible() const noexcept { return XPLMGetWindowIsVisible(window_id_) != 0; }

bool ImgWindow::OnShow() { return true; }

void ImgWindow::SetWindowDragArea(int left, int top, int right, int bottom) {
    drag_left_ = left;
    drag_top_ = top;
    drag_right_ = right;
    drag_bottom_ = bottom;
}

void ImgWindow::ClearWindowDragArea() { drag_left_ = drag_top_ = drag_right_ = drag_bottom_ = -1; }

bool ImgWindow::HasWindowDragArea(int* pL, int* pT, int* pR, int* pB) const {
    // return definition if requested
    if (pL)
        *pL = drag_left_;
    if (pT)
        *pT = drag_top_;
    if (pR)
        *pR = drag_right_;
    if (pB)
        *pB = drag_bottom_;

    // is a valid drag area defined?
    return drag_left_ >= 0 && drag_top_ >= 0 && drag_right_ > drag_left_ && drag_bottom_ >= drag_top_;
}

bool ImgWindow::IsInsideWindowDragArea(int x, int y) const {
    // values outside the window aren't valid
    if (x == -FLT_MAX || y == -FLT_MAX)
        return false;

    // is a drag area defined in the first place?
    if (!HasWindowDragArea())
        return false;

    // inside the defined drag area?
    return drag_left_ <= x && x <= drag_right_ && drag_top_ <= y && y <= drag_bottom_;
}

void ImgWindow::SafeDelete() {
    pending_destruction.push_back(this);
}


static bool init_done;
// static
bool ImgWindow::Initialize() {
    if (init_done)
        return true;
    init_done = true;
    LogMsg("ImgWindow::Initialize: initializing ImGui context and shared font atlas");

    vr_enabled_dr = XPLMFindDataRef("sim/graphics/VR/enabled");
    frame_rate_period_dr = XPLMFindDataRef("sim/operation/misc/frame_rate_period");

    global_context_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(global_context_);
    auto& io = ImGui::GetIO();
    io.BackendFlags |=
        ImGuiBackendFlags_RendererHasTextures;  // We can honor ImGuiPlatformIO::Textures[] requests during render.
    shared_font_atlas_ = io.Fonts;

    // Create a flight loop id, but don't schedule it yet
    XPLMCreateFlightLoop_t loop_params = {
        sizeof(loop_params),                      // structSize
        xplm_FlightLoop_Phase_BeforeFlightModel,  // phase
        XPFlightLoopCb,                             // callbackFunc
        nullptr,                              // refcon
    };

    fltl_id = XPLMCreateFlightLoop(&loop_params);
    fl_running = false;
    return true;
}

void ImgWindow::Finalize() {
    if (global_context_ == nullptr)
        return;
    ImGui::SetCurrentContext(global_context_);
    LogMsg("ImgWindow::Finalize: destroying ImGui textures");
    for (ImTextureData* tex : ImGui::GetPlatformIO().Textures)
        if (tex->RefCount == 1) {
            tex->SetStatus(ImTextureStatus_WantDestroy);
            UpdateTexture(tex);
        }

    LogMsg("ImgWindow::Finalize: destroying ImGui context %p", (void*)global_context_);
    ImGui::DestroyContext(global_context_);
    global_context_ = nullptr;
    shared_font_atlas_ = nullptr;

   if (fltl_id)
        XPLMDestroyFlightLoop(fltl_id);

}

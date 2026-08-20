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

#include <string>
#include <fstream>
#include <memory>
#include <future>
#include <chrono>
#include <thread>

#include "dyn_display.h"
#include "log_msg.h"
#include "http_get.h"

#include "stb_image.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

namespace dgs {
namespace logo {

// A note on async processing:
// Everything is synchronously fired by the flightloop so we don't need mutexes

// these variables are owned and written by the main (= flightloop) thread
static bool download_active;
std::unique_ptr<DynDisplay::Image> airline_logo;

// use of this variable is alternate
// If download_active:
//  true:  written by the download thread
//  false: read and written by the main thread
static std::unique_ptr<DynDisplay::Image> logo_new;
static std::string icao_new;

// variable under system control
static std::future<bool> download_future;
static bool init_done;
static std::string api_url;
static std::string api_key{"openSAM-logos-2026"};

static bool DownloadLogo(int wh) {
    if (!init_done) {
        init_done = true;
        LogMsg("DownloadLogo: Initializing logo download subsystem");
        std::ifstream cfg(user_cfg_dir + "/logostream.cfg");
        if (!cfg)
            cfg.open(base_dir + "/logostream.cfg");
        if (!cfg) {
            LogMsg("DownloadLogo: no config files found, bye");
            return false;
        }

        std::string line;
        line.reserve(80);
        // we go the brute force way, just read the whole file and look for the url= and key= line
        while (std::getline(cfg, line)) {
            if (line.empty())
                continue;
            if (line.back() == '\r')
                line.pop_back();

            if (line.starts_with("url="))
                api_url = line.substr(4);
            if (line.starts_with("key="))
                api_key = line.substr(4);
        }
    }

    if (api_url.empty() || api_key.empty()) {
        LogMsg("DownloadLogo: no url or key configured, bye");
        return false;
    }

    std::string url = api_url + "/airlines/icao/" + icao_new + "?format=png&key=" + api_key;
    // LogMsg("%s", url.c_str());

    std::string png_str;
    png_str.reserve(20 * 1024);
    bool res = HttpGet(url, png_str, 5);

    if (!res) {
        LogMsg("DownloadLogo: HttpGet failed for '%s'", url.c_str());
        return false;
    }

    LogMsg("got png %d bytes", (int)png_str.length());

    int width, height, channels;
    unsigned char* data = stbi_load_from_memory(reinterpret_cast<const unsigned char*>(png_str.data()), (int)png_str.size(), &width, &height, &channels, 4);
    if (data == nullptr) {
        LogMsg("stbi_load_from_memory failed");
        return false;
    }

    if (width != height) {
        LogMsg("Downloaded image is not square: %dx%d", width, height);
        stbi_image_free(data);
        return false;
    }

    if (width != wh) {
        unsigned char* res = stbir_resize_uint8_linear(data, width, height, 0, nullptr, wh, wh, 0, STBIR_RGBA);
        logo_new = std::make_unique<DynDisplay::Image>(res, wh, wh, 4);
        STBIR_FREE(res, nullptr);
    } else {
        logo_new = std::make_unique<DynDisplay::Image>(data, wh, wh, 4);
    }

    stbi_image_free(data);

    LogMsg("Downloaded airline logo successfully");
    return true;
}

// is asynchronous, will return immediately, the logo will be available later
// resize it to width = height = wh, if the source is not square, reject it
void LoadAirlineLogo(const std::string& airline_icao, int wh) {
    if (download_active) {
        LogMsg("Logo download is already in progress, request ignored");
        return;
    }

    if (airline_logo && airline_icao == icao_new) {
        LogMsg("Logo for airline '%s' is already loaded, request ignored", airline_icao.c_str());
        return;
    }

    airline_logo = nullptr;   // clear the current logo, will be replaced by the new one when download is complete
    icao_new = airline_icao;  // put to static storage protected by 'download_active'
    download_future = std::async(std::launch::async, DownloadLogo, wh);
    download_active = true;
}

//
// Check for download and activate the new logo
// return true if download is still in progress
// if the logo download succeeded airline_logo is != nullptr
bool CheckAsyncDownload() {
    if (download_active) {
        if (std::future_status::ready != download_future.wait_for(std::chrono::seconds::zero()))
            return true;

        download_active = false;
        [[maybe_unused]] bool res = download_future.get();
        airline_logo = std::move(logo_new);
        return false;
    }

    return false;
}


} // namespace logo
} // namespace dgs

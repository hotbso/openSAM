//
//    openSAM: manage DGS and jetways for X Plane
//
//    Copyright (C) 2026  Holger Teutsch
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

#include <string>
#include <chrono>

#include "log_msg.h"

// A simple RAII class to measure the CPU and elapsed time of a code block, and log it on destruction.
class TimeCodeBlock {
    const std::string name_;
    const std::clock_t c_start;
    const std::chrono::high_resolution_clock::time_point t_start;

   public:
    TimeCodeBlock(const std::string& name)
        : name_(name), c_start(std::clock()), t_start(std::chrono::high_resolution_clock::now()) {}
    TimeCodeBlock(const TimeCodeBlock&) = delete;
    TimeCodeBlock& operator=(const TimeCodeBlock&) = delete;
    TimeCodeBlock(TimeCodeBlock&&) = delete;
    TimeCodeBlock& operator=(TimeCodeBlock&&) = delete;

    ~TimeCodeBlock() {
        const std::clock_t c_end = std::clock();
        auto t_end = std::chrono::high_resolution_clock::now();
        LogMsg("TimeCodeBlock '%s': CPU: %1.6fs, elapsed: %1.6fs", name_.c_str(), (double)(c_end - c_start) / CLOCKS_PER_SEC,
               std::chrono::duration<double>(t_end - t_start).count());
    }
};

//
//    Copyright (C) 2023, 2025 Holger Teutsch
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

#include <cstdio>
#include <cstring>
#include <format>

#include "log_msg.h"

#ifdef LOCAL_DEBUGSTRING
void XPLMDebugString(const char *str) {
    fputs(str, stdout); fflush(stdout);
}
#else
#include "XPLMUtilities.h"
#endif

// This function can be called from anywhere anytime (e.g. from destructors of static objects).
// Avoid using static objects here that might already be gone when LogMsg is still be called.

void LogMsgImpl(const std::string& line) noexcept {
    XPLMDebugString(log_msg_prefix);
    XPLMDebugString(line.c_str());
}

void LogMsgRawImpl(const char *file, int line_no, const char *str) noexcept {
    char line[1024];
    snprintf(line, sizeof(line) - 3, "%s%s:%d: *raw*\n", log_msg_prefix, file, line_no);
    XPLMDebugString(line);
    XPLMDebugString(str);
    XPLMDebugString("\n");
}

void LogMsgRawImpl(const char *file, int line_no, const std::string& str) noexcept {
    LogMsgRawImpl(file, line_no, str.c_str());
}

/*
 * ImgFreeTypeFontAtlas.cpp
 *
 * Integration for dear imgui into X-Plane: Free-type Font Rasterisation Shim
 *
 * Copyright (C) 2020, Christopher Collins
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

*/

#include "ImgFreeTypeFontAtlas.h"

#include <XPLMGraphics.h>

void
ImgFreeTypeFontAtlas::bindTexture()
{
    if (mTextureBound)
        return;

    XPLMGenerateTextureNumbers(&mGLTextureNum, 1);

    unsigned char *pixData = nullptr;
    int width, height;
    ImGuiFreeType::BuildFontAtlas(mOurAtlas, mGlobalFlags);
    mOurAtlas->GetTexDataAsRGBA32(&pixData, &width, &height);

    XPLMBindTexture2d(mGLTextureNum, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixData);

    mOurAtlas->SetTexID(reinterpret_cast<void *>(static_cast<intptr_t>(mGLTextureNum)));
    mTextureBound = true;
}

ImgFreeTypeFontAtlas::ImgFreeTypeFontAtlas(unsigned int globalFlags):
    ImgFontAtlas(),
    mGlobalFlags(globalFlags)
{
}

ImgFreeTypeFontAtlas::~ImgFreeTypeFontAtlas()
{
}

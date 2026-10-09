# XSB Public Sources

The sources in this repository are shared with the greater X-Plane developer
community in the hope that it may save somebody a headache some day.

## Licensing Note

New sources are released under the BSD 3-Clause license.  Older sources are
released under their respective licenses. 

## Prerequisites

Components in this library assume the availability of the X-Plane SDK and may
rely upon unstable/still-in-development features.

## Components in this Repository

* `ImgWindow` and `ImgFontAtlas` - Wrapper for the
  [dear imgui](https://github.com/ocornut/imgui) Immediate Mode GUI library

* `WavFile` - Simple PCM Wavfile loader as used by XSB 1.4 onwards.

* `XOGLUtils` - the libxplanemp1 OpenGL2 binding code. (See license in header file)

## XOGLUtils - isn't that part of xplanemp?

XOGLUtils is being removed from libxplanemp2 as it's moving to modern XPSDK
methods and doesn't need direct rendering support, but we still need it in
XSquawkBox.  Because it's lost it's old home, it now lives in here.


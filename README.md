# openSAM
An open source implementation of SAM that drives jetways and provides VDGS and Marshaller service.

## Credits
Jonathan Harris (aka Marginal) (https://github.com/Marginal) for creating Autogate
@Papickx + @cxn0026 for providing better textures and day + night lighting

## License
Please observe that this material is covered by various licenses.

### The objects and textures and source code of Autogate by Marginal:
-- copy of license remark from https://github.com/Marginal/AutoGate ---\
The plugin code in the src directory is licensed under the GNU LGPL v2.1 license.\
The rest of the kit is licensed under the Creative Commons Attribution license. In short, you can use any part of this kit (including the 3D objects and their textures) in original or modified form in a free or commerical scenery package, but you must give the author credit.

### The alerting sound for jetways:
pixabay.com "8 royalty-free reverse-beep sound effects"
https://pixabay.com/sound-effects/backing-up-beepwav-14889/

### Contributions by hotbso:
This is in part a derived work from Autogate so the above mentioned licenses apply accordingly to the components of this project.

### Home on x-plane.org
https://forums.x-plane.org/index.php?/files/file/90865-opensam-an-open-source-replacement-for-sam-on-xp12/

## Build

### Windows
The build process is performed on msys2 with the mingw64 personality.\
Install expat with "pacman -S expat"
make -f Makefile.mgw64

### Linux
make -f Makefile.lin64

### macOS on Linux
The build process is performed on Linux with an osxcross environment.\
Install expat, -arm64 installs universal libraries. "-s" install static libraries only.

export MACOSX_DEPLOYMENT_TARGET=12.0
omp install -s -arm64 expat

make -f Makefile.osxcross

### macOS on macOS
port install expat +universal
make -f Makefile.mac64

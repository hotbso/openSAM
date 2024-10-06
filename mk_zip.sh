#!/bin/bash
set -v
VERSION=$(cat version.mak |sed -e 's/.*=//')

(cd openSAM-pkg && /c/Program\ Files/7-Zip/7z.exe a "../openSAM-$VERSION.zip" *)

./sync_xp11_pkg.sh
(cd openSAM-pkg_XP11 && /c/Program\ Files/7-Zip/7z.exe a "../openSAM_XP11-$VERSION.zip" *)

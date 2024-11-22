#!/bin/bash
set -v
VERSION=$(cat version.mak |sed -e 's/.*=//')

(cd openSAM-pkg && 7z a "../openSAM-$VERSION.zip" *)

./sync_xp11_pkg.sh
(cd openSAM-pkg_XP11 && 7z a "../openSAM_XP11-$VERSION.zip" *)

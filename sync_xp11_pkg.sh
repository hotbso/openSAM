#!/bin/bash
#
# Sync openSAM-pkg to openSAM-pkg_X11 removing XP12 lighting attributes
#
XP11=../openSAM-pkg_XP11

cd openSAM-pkg
find . \! -name '*.xpl' |\
while read f
do
    xp11=$XP11/$f
    if [[ -d $f ]]
    then
        if [[ ! -d $xp11 ]]; then mkdir -p $xp11; fi
    elif [[ ! -e $xp11 || $xp11 -ot $f ]]
    then
        if [[ $f == *.obj ]]
        then
            cat $f |\
                sed -e 's/ATTR_light_level/#XP11 ATTR_light_level/g' \
                    -e 's/GLOBAL_luminance/#XP11 GLOBAL_luminance/g' > $xp11
        else
            cp -p $f $xp11
        fi
    fi
done


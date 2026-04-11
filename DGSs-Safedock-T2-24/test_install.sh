RESDIR=/e/X-Plane-12-test/Resources/plugins/AutoDGS/resources
if [ -d $RESDIR ]
then
    for f in Safedock-T2-24* pole_base*
    do
        [[ $f == *-base.obj ]] && continue
        if [[ $f == *.obj || $f == *.png || $f == *.dds ]]
        then
            echo "Copy \"$f\""
            cp -p $f $RESDIR/.
        fi
    done
fi

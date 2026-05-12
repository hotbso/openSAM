export PYTHONPATH := ../tools

%.obj: %.obj-base gen_display.py ../tools/vdgstools/display.py
	py gen_display.py $<

all: Safedock-T2-24-display.obj

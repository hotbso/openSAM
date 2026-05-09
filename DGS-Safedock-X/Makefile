export PYTHONPATH := ../tools

all: Safedock-X-display.obj Safedock-X-idle.obj

DEPENDENCIES = Safedock-X-display.obj-base gen_display.py ../tools/vdgstools/display.py
Safedock-X-display.obj:  $(DEPENDENCIES)
	python gen_display.py $<

Safedock-X-idle.obj:  $(DEPENDENCIES)
	python gen_display.py $< -idle

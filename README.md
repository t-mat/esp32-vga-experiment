# esp32-vga-experiment

Experiment for output VGA signal from ESP32.


## How to build and run

+ Check and set `YOUR_GPIO_NUM_xxx` in `main/my_config.h`.  You should define and connect 3 GPIO pins: V-Sync, H-Sync, Video (single pin).
+ Run `make menuconfig` for basic setup.
+ Run `make -j flash monitor` for build & run.

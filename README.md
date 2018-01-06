# esp32-vga-experiment

Experiment for output VGA signal from ESP32.


## How to build and run

+ Check and set `GpioPins{}` and `Peripherals` in `main/app_main.cpp`.  You should define and connect at least 3 GPIO pins: V-Sync, H-Sync, Video (single pin).
+ Run `make menuconfig` for basic setup.
+ Run `make -j flash monitor` for build & run.

# Flowberry

Software described in "Ego-motion Sensor for Unmanned Aerial Vehicles Based on a
Single-Board Computer"

## Hardware

 - Raspberry Pi 3
 - Raspberry Pi Camera Module v2
 - L3GD20H (gyro) connected to I2C
 - HRLV-EZ4 (ultrasonic sensor) connected to UART

## Compilation

First, [build the OpenCV library][1] (version 3.1.0) and make sure it is
registered by `pkg-config`:

```
pkg-config --libs opencv
```

Then, make sure all submodules are loaded:

```
git submodule init
git submodule update
```

Finally, build the application by running `make`.

## Usage

Obtain `gyro_calib.txt` using the `gyro` utility in `tools/gyro`:

```
./gyro -c
```

Calibrate camera using using [RaspiCalib][2] and save its output as
`flowberry_camera_calib.xml`:

```
./raspicalib default.xml
```

Finally, run flowberry at 30 FPS:

```
./flowberry 30
```

This starts Mavlink server at `192.168.42.42:14550`. The data can be observed
using [QGroundControl][3].

[1]: http://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/
[2]: https://github.com/adamheinrich/RaspiCalib
[3]: http://qgroundcontrol.com/

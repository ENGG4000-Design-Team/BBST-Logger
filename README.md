# BBST-Logger
Data logging application for the Balloon Borne Solar Telescope. This application will capture data from the IMU, photodiodes, and camera sensor atteached to the payload and send this data to a log file(s), as well as gimbal controller over serial communication. This application is meant to run as a Cron Job on a Raspberry Pi. All testing is performed on a Raspberry Pi 4 Mode B.

# Dependencies
- [WiringPi Unofficial](https://github.com/WiringPi/WiringPi)
- [OpenCV](https://opencv.org/)
- [CMPS14](https://github.com/EthanG78/CMPS14)

# Libraries
A modified version of Ken Willmott's [SolarPosition](https://github.com/KenWillmott/SolarPosition) C++ library is used to determine the azimuth and elevation of the Sun at given times, latitutes, and longitudes. This library was built for Arduino devices, made changes to have running on Linux.

## Building
To build the executable, first clone the repository:
```console
user@machine ~$ git clone https://github.com/ENGG4000-Design-Team/BBST-Logger.git
```
Enter the repository directory and create a build directory:
```console
user@machine ~$ cd BBST-Logger/
user@machine BBST-Logger$ mkdir build
```
Enter the build directory and configure the project using CMake:
```console
user@machine BBST-Logger$ cd build/
user@machine build$ cmake -S .. -B .
```
Build the project using CMake:
```console
user@machine build$ cmake --build .
```

## Running application as Cron Job
todo:

Author: Ethan Garnier
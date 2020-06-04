# DeftWind Project

> 中级无人飞行自动驾驶仪

[![release](https://img.shields.io/badge/release Beta-v2.2.0-ff0000.svg)](http://192.168.0.12/uav/DeftWind/tags/Version2.2.0_beta)
[![license](https://img.shields.io/badge/license-GPL v3.0-blue.svg)](http://192.168.0.12/uav/DeftWind/blob/plane/LICENSE)
[![build](https://img.shields.io/badge/build-makefile-green.svg)](https://www.gnu.org/software/make/manual/make.html)

## The DeftWind project is made up of: ##
- DeftCopter : [code](http://192.168.0.12/uav/DeftWind/tree/copter), [wiki](http://192.168.0.12/uav/DeftWind/tree/copter)
- DeftPlane  : [code](http://192.168.0.12/uav/DeftWind/tree/plane), [wiki](http://192.168.0.12/uav/DeftWind/tree/plane)

## Board information ##
- UAVRS-V1 : STM32F427(Flash: 2M, RAM: 256Kbyte)
- UAVRS-V2 : NXP IMXRT1052(RAM: 512Kbyte)
    - V1.001Rev EXT FLASH: Winbond QFlash 32M
    - V1.002Rev EXT FLASH: Cypress HyperFlash 64M

## License ##
The DeftWind project is licensed under the GNU General Public
License, version 3.

## How to build ##

Step 1:
- uavrs-v1 : make configure board=uavrs-v1
- uavrs-v2 : make configure board=uavrs-v2
- sitl : make configure board=sitl

Step 2:
- plane : make plane
- copter : make copter

## Maintainers ##

- **Jin.Jing**
- **WenYi.Jing**
- **XianXin.Hou**

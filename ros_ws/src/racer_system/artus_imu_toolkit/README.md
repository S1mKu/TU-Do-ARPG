# ARTUS IMU TOOLKIT
This library contains packages for connecting and interacting 
with IMU Sensor-Modules/-Systems to provide additional sources 
of orientation and movement data.
Therefore, Data (Gyroscope, Accelerometer & Magnetometer := 9DOF) 
is collected by a microcontroller-unit and will be already fused 
on that IMU-MCU Sensor-Module/-System. After fusing this imu data
an orientation as well as angular velocities and linear acceleration 
are provided including some extra Data like relative Speed 
or relative traveled distance are sent to the main computing unit. 
By modifying and re-flashing the provided arduino_firmware other 
already precalculated Data can be published to the main computing 
unit like an NVIDIA Jetson board. 
(eg. Yaw, Roll, Pitch instead of Quaternions)

## Structural Overview 
![txt](img/overview.png)

## arduino_firmware
This Package provides the firmware to publish IMU data to a main 
computing unit like a NVIDIA Jetson Boards as well as additional 
firmware to perform calibration procedures.

### AHRS firmware
This firmware type is used to read raw IMU sensor module data, 
process and correct this information to obtain orientation, 
angular velocity, linear acceleration as well as additional useful 
data. Afterwards the desired as well as required data is transmitted
to the main computing unit over a serial connection.
Beside the ability to collect (more or less) raw IMU data, one major 
part of this AHRS is the correction and processing step of IMU sensor 
data.
As a correction step sensor-module-specific offset and gain values 
identified by calibration procedures are applied to the raw IMU 
sensor values.
To process and smooth the corrected imu data filter algorithms are
used. Therefore, a small subset of possible filter algorithms are 
available (Mahony | Madgwick | NXP fusion/Kalman).
As a result of the filter process orientation, linear acceleration
and angular velocity as well as useful additional data is generated,
that will be sent over the serial-over-USB interface.

#### Additional Information:
- **Firmware-Filter-Options**:
    - Mahony [lowest memory/computation]
    - Madgwick [fair memory/computation]
    - NXP fusion/Kalman [highest memory/computation].

### calibration firmware
Beside the basic AHRS firmware additional calibration firmware for
specific IMU sensor module components are provided.
Therefore, for every component of an IMU sensor module 
(Accelerometer|Gyroscope|Magnetometer) a dedicated calibration 
firmware is provided. So only one single IMU sensor module component
can be calibrated at the same time. To perform a calibration for a
specific sensor module component the corresponding calibration 
firmware needed to be flashed to the MCU.

### Recommendations regarding MCUs
It is recommended to run this AHRS firmware on Hardware, which is at 
least similar or better than SAMD21 (ARM Cortex M0+) MCUs.
Considerable suggestions are for example RP2040 based MCUs because of
their relative fast (133 MHz) Dual-Core ARM Cortex M0+ processor.
In addition, also a SAMD51 (ARM Cortex M4) MCUs is advisable choice
because of its Floating-Point-Unit provided natively.

It's not approved to use any kind of AVR MCUs or anything with 
similar capabilities (whether 8-Bit or 32-Bit). 
This kind of algorithms as well as their implementations are 
consuming a lot of space in memory and RAM.
And especially regarding 8-Bit Architectures, their limited 
computational capability and processing of word-length per cycle 
do also affect the value ranges of basic variable types. 

## artus_imu_publisher [ROS1]
This Package provides a ROS1 Node to read messages from a (emulated) 
Serial(-USB)-Interface to publish IMU data as well as som extra 
information to a ROS1 System on the main computing unit like a 
NVIDIA Jetson Boards.

## Installation
In order to use all features of this package it is necessary to 
install additional non-ROS related dependencies.

### arduino_firmware
- ***Arduino IDE*** (or for experts: after some adjustments any other suitable microcontroller-Cross-compiling Tool-Chain)
- ***Board-Definitions***: Depending on used MCU adding additional board description to Arduino IDE Boardmanager
  - 9DOF Razor M0 IMU: https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json (May not work on ARM architectures because of missing cross-compiling tool-chain parts)
  - Seeed Studio XIAO: https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
- ***Arduino Libraries***: By using the Library-Manager of Arduino IDE
  - ***MPU9250*** & ***9Dof Razor M0***: 
    - https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/archive/master.zip
  - (Adafruit) ***LSM6DSOX_LIS3MDL***: 
    - Adafruit AHRS
    - Adafruit LIS3MDL
    - Adafruit LSM6DS
    - Adafruit Unified Sensor
- ***Magnetometer Calibration:***
  - Motion Cal (https://www.pjrc.com/store/prop_shield.html)
- ***Accelerometer Calibration:***
  - Octave / Matlab

## Usage
### Serial Interface:
Usually Linux will assign a port numbered in ascending order to a 
newly plugged in Serial-Over-USB device since startup.
In most cases the serial Device can be found at `/dev/ttyACM¿` 
where `¿` will be a number. In some cases eg. using a non-official
Arduino board also that serial device can be found at `/dev/ttyUSB¿`,
where `¿` will also be a number.
For easy distinguishing between Serial devices (eg. a VESC and IMU)
one can use both Arduino IDE Serial Monitor (or Tools > Ports) 
and Python-Serial. For using Python-Serial you can execute following
command to print a (detailed) list of all active Serial-Devices 
found at the system:

***Python 2***:
```
python -m serial.tools.list_ports -v  
```
or
***Python 3***:
```
python3 -m serial.tools.list_ports -v 
```
Otherwise, it could be considered to define some udev rules, so the 
Linux system will assign fixed virtual device names, that will be 
always identical and independent of how and when the devices where 
plugged-in.

### arduino_firmware
To easily use this package zip the parent directory containing 
`artus_imu_toolkit_firmware` 
(eg. using nautilus > right-click on `arduino_firmware` > compress…)
After generating the zip-file start the Arduino IDE and select 
`Sketch > Include Library > Add .ZIP Library...`. Then choose the 
freshly generated .ZIP file and add it to your internal Arduino IDE 
library. After this step (and after installing all other arduino 
dependencies) you find both includable header-files 
(`Sketch > Include Library > artus_imu_toolkit_firmware`) and 
executable example-files (`File > Examples > artus_imu_toolkit_firmware`)
installed to your Arduino IDE.

In case to modify or adding new IMU sensor systems or algorithms 
provided by this package, it is necessary to apply this changes to 
the library directory directly. 
(Otherwise the changes are not used while recompiling by Arduino-IDE-ToolChain)
Usually the Arduino Libraries can be found here:
`~/Arduino/libraries/artus_imu_toolkit_firmware ` 

### artus_imu_publisher
To run the publisher node dedicated to a supported IMU-Sensor system
start one of the provided launchfiles.

This package provides some specific config.yaml for every supported 
IMU-Sensor System. This configurations are loaded into the python 
publisher when starting the publisher node.
In this way every offline adjustments or modifications can be 
applied more easily compared to a hardcoded solution.

In theory there is also the possibility to use the ***dynamic 
reconfigure*** procedures to update these data while the node is 
running. 
It seems that actually this feature isn't working or not fully 
implemented jet.

## Package Structure
- **arduino_firmware**
  - artus_imu_toolkit_firmware
    - examples
      - 9DOF_Razor_M0_IMU
        - artus_9DOF_Razor_M0_acceleration_calibration
        - artus_9DOF_Razor_M0_ahrs_ROS
        - artus_9DOF_Razor_M0_compass_calibration
        - artus_9DOF_Razor_M0_gyro_calibration
      - adafruit_LSM6DSOX_LIS3MDL
        - artus_LSM6DSOX_LIS3MDL_acceleration_calibration
        - artus_LSM6DSOX_LIS3MDL_ahrs_ROS
        - artus_LSM6DSOX_LIS3MDL_compass_calibration
        - artus_LSM6DSOX_LIS3MDL_gyro_calibration
      - MPU9250
        - artus_MPU9250_acceleration_calibration
        - artus_MPU9250_ahrs_ROS
        - artus_MPU9250_compass_calibration
        - artus_MPU9250_gyro_calibration
    - script_matlab : useful for accelerometer calibration
    - src


- **artus_imu_publisher**
  - cfg : configuartion files fpr dynamic reconfiguration
  - config : offline configurations files for each supported IMU-Sensor system
  - launch : Launchfile for each supported IMU-Sensor system
  - nodes : Publisher-Node for each supported IMU-Sensor system


- **doc** : Document and Resource selection for used Hardware and Software


- **img** : Images of Documentation

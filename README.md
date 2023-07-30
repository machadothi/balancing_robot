# Balancing robot
This is a C project developed to control a balancing robot car. This project uses FreeRTOS and a STM32 Bluepill.

![Balancing Robot](img/balancing_robot.png)

# Build PC Project

```
$ mkdir build

$ cd build

$ cmake ..

$ make
```
# Building Tests

Follow the same steps as above changing the last command by:

```
$ make unit_tests
```

# Setup Bluepill environment

- Clone reference project

```
$ git clone https://github.com/ve3wwg/stm32f103c8t6.git
```

- Clone `libopencm3`

```
$ cd stm32f103c8t6
$ git clone https://github.com/libopencm3/libopencm3.git
```

- Access `https://www.freertos.org` and download the latest LTS version, then

```
$ cd rtos
$ unzip ~/<Download dir>/FreeRTOSv<VERSION>-LTS.zip 
```

- Edit `Project.mk` file and the following line inside of with the correct 
FreeRTOS version:

```
FREERTOS? = FreeRTOSv10.0.1
```

- Download the cross-compiler from `https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads`
and install it.

```
# cd /opt

# tar xjf ~myuserid/Downloads/gcc-arm-none-eabi-6-2017-q2-update-mac.tar.bz2

# mv gcc-arm-none-eabi-6-2017-q2-update gcc-arm

$ export PATH="/opt/gcc-arm/bin:$PATH"

$ arm-none-eabi-gcc --version
```

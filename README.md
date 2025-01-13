
# Raspberry Pi  📚

Welcome to the **Raspberry Pi** This repository is designed to help you explore and learn about Raspberry Pi, covering a wide range of topics to build a solid foundation.

---

## 📚 Table of Contents

1. [Introduction](#introduction)
2. [History](#history)
3. [Versions](#versions)
4. [Quick Setup](#quick-setup)
5. [Remote Control](#remote-control)
6. [File Sharing](#file-sharing)
7. [GPIO Control](#gpio-control)
8. [Programming Examples](#programming-examples)
9. [Pulse Width Modulation (PWM)](#pulse-width-modulation-pwm)
10. [Components](#components)
11. [Further Resources](#further-resources)

---

## 🖥️ Introduction

This guide covers the **setup and usage of Raspberry Pi** for various purposes, including GPIO control, remote access, and programming in C, C++, and Python.

Raspberry Pi is:
- A **complete computing platform**.
- **Affordable** and accessible.
- Integrated with **GPIO for hardware control**.
- Runs **Linux OS**, providing a flexible environment for development.

---

## 📜 History

The Raspberry Pi has evolved over time with many improvements in performance, hardware features, and community support. Learn more about the history on [Raspberry Pi's official page](https://www.raspberrypi.org/).

---

## 🛠️ Versions

Raspberry Pi comes in a variety of models:
- **Raspberry Pi 3**
- **Raspberry Pi 4**
- Raspberry Pi Zero
- Raspberry Pi Pico

---

## ⚡ Quick Setup

### Step 1: Burn OS into the SD Card
1. Download a Raspberry Pi OS image.
2. Use [Raspberry Pi imager](https://www.raspberrypi.com/software/) to flash the OS onto the SD card.

### Step 2: Plug and Play
1. Insert the SD card into your Raspberry Pi.
2. Power it on and connect peripherals (keyboard, mouse, display).

---

## 🌐 Remote Control

### Headless Setup (No Display, Keyboard, or Mouse)
1. Add an `ssh` file to the boot directory on the SD card.
2. Connect Wi-Fi manually by adding `wpa_supplicant.conf` to the boot folder.
3. Use tools like:
   - **Advanced IP Scanner** or **Angry IP Scanner** to find the IP address.
   - **VNC** for remote desktop access.

### Remote File Sharing
- Use **WinSCP** for Windows or terminal-based commands for Linux.

---

## 📂 File Sharing

Share files with:
- **WinSCP** on Windows.
- Terminal commands:
  ```bash
  scp file.txt pi@<IP_ADDRESS>:/home/pi/

## 🔌 GPIO Control


# ⚡ GPIO Control Setup

## GPIO Export
To export a GPIO pin, use the following command:
```
echo <pin_number> > /sys/class/gpio/export 
Example (Export pin 17):
echo 17 > /sys/class/gpio/export
```
## Set Direction
After exporting the pin, set the direction to either `in` or `out`:
```
echo "out" > /sys/class/gpio/gpio<pin_number>/direction
Example (Set pin 17 as output):
echo "out" > /sys/class/gpio/gpio17/direction
```
## Set Value
```
To control the pin value (high or low), use:
echo 1 > /sys/class/gpio/gpio<pin_number>/value
To turn the pin on (high):
echo 1 > /sys/class/gpio/gpio17/value
To turn the pin off (low):
echo 0 > /sys/class/gpio/gpio17/value
```

**Full Example**
Here’s an example of exporting pin 17, setting it as an output, and turning it on (high):

```
echo 17 > /sys/class/gpio/export    # Export pin 17
echo "out" > /sys/class/gpio/gpio17/direction  # Set as output
echo 1 > /sys/class/gpio/gpio17/value  # Turn pin 17 high
```

And turning it off (low):
```
echo 0 > /sys/class/gpio/gpio17/value  # Turn pin 17 low
```
## 🔦 Notes:

1- Replace <pin_number> with the actual GPIO pin number.

2- Ensure the pin is correctly exported before setting direction or value.

3- You may need root privileges to access GPIO files.

# 💻 GPIO Control: Programming Examples

## **C Example (Using WiringPi Library)**

In this C example, we'll use the **WiringPi** library to control a GPIO pin and turn an LED on and off.

```c
#include <wiringPi.h>

int main() {
    wiringPiSetup();           // Initialize WiringPi library
    pinMode(0, OUTPUT);        // Set GPIO pin 0 as output
    digitalWrite(0, HIGH);     // Turn LED on (HIGH)
    delay(1000);               // Wait for 1 second
    digitalWrite(0, LOW);      // Turn LED off (LOW)
    return 0;
}
```
## **Explanation**

-  **`wiringPiSetup()`**: Initializes the WiringPi library to set up the GPIO pin control system.
-  **`pinMode(0, OUTPUT)`**: Configures GPIO pin 0 as an **output** pin, allowing it to send signals.
-  **`digitalWrite(0, HIGH)`**: Turns the **LED on** by sending a **HIGH signal** (3.3V) to GPIO pin 0.
-  **`delay(1000)`**: Pauses the program for **1 second** to keep the LED on.
-  **`digitalWrite(0, LOW)`**: Turns the **LED off** by sending a **LOW signal** (0V) to GPIO pin 0.


## **Python Example:**

```python
import RPi.GPIO as GPIO    #  Import the RPi.GPIO library
import time                #  Import the time library for sleep functionality

GPIO.setmode(GPIO.BCM)     #  Set the GPIO mode to BCM (Broadcom pin numbering)
GPIO.setup(18, GPIO.OUT)   #  Set GPIO pin 18 as an output

GPIO.output(18, GPIO.HIGH)  #  Turn the LED on (send HIGH signal)
time.sleep(1)               #  Wait for 1 second

GPIO.output(18, GPIO.LOW)   #  Turn the LED off (send LOW signal)
GPIO.cleanup()              #  Cleanup GPIO settings
```
## 🔄 **Pulse Width Modulation (PWM)**

PWM allows precise control of devices like servos and motors by varying the duty cycle. Here's an example setup to control an LED's brightness or motor speed:

```python
import RPi.GPIO as GPIO   #  Import the RPi.GPIO library
import time               #  Import the time library for sleep functionality

GPIO.setmode(GPIO.BCM)    #  Set GPIO mode to BCM (Broadcom pin numbering)
GPIO.setup(18, GPIO.OUT)  #  Set GPIO pin 18 as an output

# PWM Setup
pwm = GPIO.PWM(18, 100)   #  Initialize PWM on pin 18 with 100 Hz frequency
pwm.start(50)              #  Start PWM with a 50% duty cycle (50% on, 50% off)

time.sleep(2)              #  Wait for 2 seconds to observe PWM effect
pwm.stop()                 #  stop the PWM signal
GPIO.cleanup()             #  Cleanup GPIO settings
```

## 📘 **Further Resources**

- 📚 **Datasheets and Documentation**: 
  - For a deeper understanding of GPIO functionality and more detailed information, refer to the [**ARM Developer Documentation**](https://developer.arm.com/).

- 🗺️ **GPIO Pinout**: 
  - For a comprehensive and detailed **GPIO pinout guide**, check the official [**Raspberry Pi GPIO Pinout**](https://www.raspberrypi.org/documentation/).






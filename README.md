
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

```
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

```
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


## 🔎 Discovering 

# ⚙️ Technical Concepts

## 1. 🔄 Servo Motor
* **Description:** A type of actuator that rotates to a specific position.
* **Example:** Controlling the position of a robotic arm.
* [Servo Motor](https://www.servocity.com/servos)

## 2. 🖥️ GUI App Startup
* **Description:** The process of launching a graphical user interface (GUI) application.
* **Example:** Opening a web browser or a desktop application.
* [GUI App](https://www.techopedia.com/definition/5437/graphical-user-interface-gui)

## 3. ⚡ Bring Up Device
* **Description:** The process of initializing and starting a device or component.
* **Example:** Powering on a microcontroller and configuring its peripherals.
* [Device Startup](https://www.embedded.com/bringing-up-an-embedded-system/)

## 4. 🔌 UART (Universal Asynchronous Receiver/Transmitter)
* **Description:** A serial communication protocol for transmitting data between devices.
* **Example:** Connecting a microcontroller to a GPS module.
* [UART](https://www.sparkfun.com/pages/serial_communication)

## 5. 🔗 SPI (Serial Peripheral Interface)
* **Description:** A synchronous serial communication protocol for connecting multiple devices on a single bus.
* **Example:** Connecting a microcontroller to a sensor or an external memory chip.
* ![SPI](https://www.analog.com/en/analog-dialogue/articles/introduction-to-spi-interface.html)

## 6. 🔄 I2C (Inter-Integrated Circuit)
* **Description:** A serial communication protocol for connecting multiple devices on a two-wire bus.
* **Example:** Connecting a microcontroller to an accelerometer or a temperature sensor.
* [I2C](https://www.i2c-bus.org/)

## 7. 🗂️ Device Tree
* **Description:** A data structure used in Linux to describe hardware devices and their properties.
* **Example:** Configuring GPIO pins, clocks, and memory addresses for a microcontroller.
* [Device Tree](https://elinux.org/Device_Tree_Reference)

## 📟 Controlling a Servo Motor with PWM on Raspberry Pi

**Overview:**

This document demonstrates how to control a servo motor using Pulse Width Modulation (PWM) on a Raspberry Pi. PWM is a technique where the duty cycle of a digital signal is modulated to control analog devices like servos.

**Hardware Setup:**

1. **Raspberry Pi:** The computing device used to generate the PWM signal.
2. **Servo Motor:** The actuator to be controlled.
3. **Breadboard:** (Optional) For easy connection of components.
4. **Jumper Wires:** To connect the servo motor to the Raspberry Pi's GPIO pins.

**Software Setup:**

1. **Enable PWM:**
    * **Enable the PWM peripheral in the device tree.** (This step may vary depending on your Raspberry Pi model and operating system.)
    * **Create the PWM device node:**

        ```
        sudo mkdir -p /sys/class/pwm/pwmchip0
        sudo ln -s /dev/pwmchip0 /sys/class/pwm/pwmchip0 
        ```

2. **Export the PWM channel:**

    ```
    echo <PWM_channel> > /sys/class/gpio/export 
    ```

    **Example:**

    ```
    echo 0 > /sys/class/gpio/export 
    ```
    (To export PWM channel 0)

3. **Set the PWM period:**

    ```
    echo <period> > /sys/class/pwm/pwmchip0/pwm<PWM_channel>/period 
    ```

    **Example:**

    ```
    echo 20000000 > /sys/class/pwm/pwmchip0/pwm0/period 
    ```
    (Sets the period to 20 milliseconds)

4. **Set the duty cycle:**

    ```
    echo <duty_cycle> > /sys/class/pwm/pwmchip0/pwm<PWM_channel>/duty_cycle 
    ```

    **Example:**

    ```
    echo 1500000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle 
    ```
    (Sets the duty cycle to 1500000 out of 20000000, which corresponds to a mid-point position for many servos)

**Controlling the Servo:**

- To move the servo to different positions, change the `duty_cycle` value. 
- **Note:** The valid duty cycle range and its mapping to servo positions vary depending on the servo motor. Consult the servo motor's datasheet for specific values.

**Example Code Snippet:**

```
import RPi.GPIO as GPIO
import time

# Configure GPIO pin for PWM
GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering
GPIO.setup(18, GPIO.OUT)  # Assuming PWM channel 0 is mapped to GPIO18

# Create PWM object
pwm = GPIO.PWM(18, 50)  # 50Hz frequency
pwm.start(0)  # Start with 0% duty cycle

# Move servo to different positions
try:
    while True:
        # Position 1
        pwm.ChangeDutyCycle(2.5)  # Adjust duty cycle as needed
        time.sleep(1)

        # Position 2
        pwm.ChangeDutyCycle(7.5)
        time.sleep(1)

        # Position 3
        pwm.ChangeDutyCycle(12.5)
        time.sleep(1)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
```

## 🛠️ Raspberry Pi Configuration Tool (`raspi-config`)

**Overview:**

`raspi-config` is a command-line tool on Raspberry Pi systems that allows you to configure various system settings. It provides a menu-driven interface for easy navigation and modification of options.

**Key Configuration Options:**

**1. ⚙️ System Options:**
   - **Configure system settings:** This option allows you to adjust system-level settings such as hostname, locale, and memory splitting.

**2. 🖥️ Display Options:**
   - **Configure display settings:** This option enables you to configure display settings, including resolution, overscan, and desktop composition.

**3. 🔌 Interface Options:**
   - **Configure connections to peripherals:** This option allows you to enable or disable various interfaces like SSH, VNC, SPI, I2C, and serial ports.

**4. ⚡ Performance Options:**
   - **Configure performance settings:** This option allows you to adjust performance-related settings such as overclocking (if supported by your Raspberry Pi model).

**5. 🌐 Localisation Options:**
   - **Configure language and regional settings:** This option allows you to set the system's language, locale, and keyboard layout.

**6. 🛠️ Advanced Options:**
   - **Configure advanced settings:** This option provides access to more advanced settings, such as memory splitting, overclocking, and boot options.

**7. 🔄 Update:**
   - **Update this tool to the latest version:** This option checks for and installs any available updates for the `raspi-config` tool itself.

**8. ℹ️ About raspi-config:**
   - **Information about this configuration tool:** This option displays information about the `raspi-config` tool, such as its version and author.

**Example Usage:**

1. Open a terminal window on your Raspberry Pi.
2. Type `sudo raspi-config` and press Enter.
3. Use the arrow keys to navigate through the menu options.
4. Select the desired option and press Enter to proceed.
5. Follow the on-screen instructions to make the necessary changes.
6. When finished, select "Finish" to exit `raspi-config`.

**Note:**

- It is recommended to use `sudo` to run `raspi-config` to ensure you have the necessary permissions to make system changes.
- Be cautious when modifying system settings, as incorrect configurations can cause instability or prevent your Raspberry Pi from booting.

## 🖥️ GUI APP Startup

**{startup}**

**Description:**

This section outlines the startup process for a Graphical User Interface (GUI) application. 

**Key Stages:**

1. **Initialization:**
    * Loading necessary libraries and modules.
    * Setting up the application environment (e.g., creating windows, initializing graphics).

2. **Resource Loading:**
    * Loading images, sounds, fonts, and other assets required by the application.

3. **Configuration:**
    * Loading and applying application settings (e.g., user preferences, system settings).

4. **Component Creation:**
    * Creating the GUI components (e.g., windows, buttons, menus, text fields) and arranging them on the screen.

5. **Event Handling:**
    * Setting up event listeners to handle user interactions (e.g., mouse clicks, keyboard input).

6. **Data Loading:**
    * Loading initial data from files, databases, or other sources.

7. **Display:**
    * Rendering the initial state of the application's user interface.

**Example:**

```
# Simplified Python example

def start_gui_app():
    """
    Initializes and starts the GUI application.
    """
    # 1. Initialization
    pygame.init()  # Initialize Pygame library
    screen = pygame.display.set_mode((800, 600))  # Create a display window

    # 2. Load resources
    background_image = pygame.image.load("background.png") 

    # 3. Load settings (example)
    try:
        with open("settings.json", "r") as f:
            settings = json.load(f)
    except FileNotFoundError:
        settings = {"theme": "light"}  # Default settings

    # 4. Create GUI components
    start_button = Button(x=300, y=250, text="Start") 

    # 5. Event handling loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if start_button.is_clicked(event.pos):
                    # Handle button click
                    pass

        # 6. Update and render
        screen.blit(background_image, (0, 0)) 
        start_button.draw(screen)
        pygame.display.flip()

    # 7. Cleanup
    pygame.quit()

if __name__ == "__main__":
    start_gui_app()
```

## 🖥️ Systemd Service for a Python GUI Application

**Overview:**

This example demonstrates how to create a systemd service file to automatically start a Python-based GUI application upon system boot. 

**Files:**

* `hello.py`: The Python script containing the GUI application logic.
* `hello_gui.service`: The systemd service file for managing the application.

**hello.py:**

```
import tkinter as tk

wd = tk.Tk()
wd.geometry("200x200+50+50") 
lbl = tk.Label(wd, text="Hello")
lbl.pack()
wd.mainloop()
```

**hello_gui.service:**

```
[Unit]
Description=hello gui app

[Service]
User=pi
Restart=on-failure
ExecStart=/usr/bin/python3.9 /home/pi/apps/gui/hello.py
Environment="DISPLAY=:0"

[Install]
WantedBy=graphical.target
```

**Explanation:**

**hello.py:**

This is a simple Python script that creates a basic Tkinter window with a label displaying "Hello".

**hello_gui.service:**

- **[Unit] section:**
  - **Description:** Provides a description of the service.

- **[Service] section:**
  - **User:** Specifies the user under which the service should run (in this case, "pi").
  - **Restart=on-failure:** Configures the service to automatically restart if it fails.
  - **ExecStart:** Specifies the command to execute to start the service. Here, it's running the hello.py script using Python 3.9.
  - **Environment:** Sets environment variables for the service. `DISPLAY=:0` is necessary for GUI applications to display correctly.

- **[Install] section:**
  - **WantedBy=graphical.target:** Specifies that this service should be started when the graphical desktop is started.

**To enable and start the service:**

1. Save the `hello.py` and `hello_gui.service` files to the appropriate locations.
2. Copy the `hello_gui.service` file to the systemd service directory (e.g., `/etc/systemd/system/`).
3. Enable the service using the command: `sudo systemctl enable hello_gui.service`
4. Start the service using the command: `sudo systemctl start hello_gui.service`

## 🔧 Three Options to Integrate Hardware with an ECU

This diagram outlines three primary approaches to integrating hardware devices with an Electronic Control Unit (ECU), typically found in automotive or embedded systems.

**1. User Space Integration:**

* **Description:** In this approach, user-space applications interact with hardware drivers directly using system calls like `read()`, `write()`, and `ioctl()`. 
* **Pros:** Relatively straightforward to implement for simple devices.
* **Cons:** Can lead to performance bottlenecks and potential security risks due to direct access to hardware.
* **Example:** A user-space application using the GPIO library to control an LED.

**2. Kernel Space Integration (with existing drivers):**

* **Description:** If a suitable driver already exists in the kernel, user-space applications can interact with the hardware through the driver's interface.
* **Pros:** Leverages existing, well-tested drivers, improving performance and reliability.
* **Cons:** Requires knowledge of the driver's API.
* **Example:** A user-space application using the character device interface to control a UART.

**3. Kernel Space Integration (with custom driver):**

* **Description:** For devices without existing drivers, a custom driver needs to be written and integrated into the kernel.
* **Pros:** Provides maximum control and flexibility over hardware interaction.
* **Cons:** Requires in-depth knowledge of kernel programming and device driver development.
* **Example:** Writing a custom driver for a new sensor or actuator.

**Key Considerations:**

* **Device Tree Compatibility:** For modern systems, the device tree (DT) plays a crucial role in configuring and managing hardware. Ensure that your integration method is compatible with the DT.
* **Performance:** The choice of integration method will impact system performance. Kernel-space drivers generally offer better performance due to lower overhead.
* **Security:** Direct user-space access to hardware can pose security risks. Consider using appropriate security measures to protect the system.

## 🔌 UART (Universal Asynchronous Receiver/Transmitter)

**Description:**

UART is a serial communication protocol used to transmit data between devices. It's asynchronous, meaning the sender and receiver don't need to be synchronized by a clock signal. 

**Key Features:**

* **Simple and Versatile:** UART is relatively easy to implement and widely used in various applications.
* **Asynchronous:** Data transmission doesn't rely on a shared clock signal.
* **Full-duplex:** Allows simultaneous data transmission and reception.
* **Low-cost:** UART interfaces are typically inexpensive to implement.

**Data Format:**

UART data is transmitted serially, one bit at a time, over a single wire. The data format typically includes:

* **Start Bit:** A low-level signal indicating the beginning of a data frame.
* **Data Bits:** 7 or 8 bits of data.
* **Parity Bit:** An optional parity bit for error checking.
* **Stop Bit:** A high-level signal indicating the end of a data frame.

**Applications:**

* **Microcontroller Communication:** Connecting microcontrollers to sensors, actuators, and other peripherals.
* **GPS Receivers:** Interfacing with GPS modules to receive location data.
* **Modems:** Communicating with modems for data transmission over phone lines.
* **Serial Consoles:** Connecting to devices for debugging and configuration.

**Example:**

* Connecting an Arduino microcontroller to a GPS module to receive location data.
* Connecting a Raspberry Pi to a Bluetooth module for wireless communication.

## UART Connection on Raspberry Pi

****


**Description:**

This diagram showcases a typical setup for connecting a Raspberry Pi to a PC/Laptop using a UART (Universal Asynchronous Receiver/Transmitter) interface.

**Key Components:**

* **Raspberry Pi:** The single-board computer acting as the host.
* **USB to Serial Converter:** This device converts the USB signal from the PC to the UART signals (TX, RX, GND) required for communication with the Raspberry Pi.
* **USB Cable:** Connects the PC to the USB to Serial Converter.
* **UART Pins:** The Raspberry Pi uses specific GPIO pins for UART communication:
    - **GPIO14 (TXD0):** Transmit data from the Raspberry Pi.
    - **GPIO15 (RXD0):** Receive data to the Raspberry Pi.
    * **GND:** Ground connection.
* **Voltage Levels:** It's crucial to note that the Raspberry Pi's UART pins operate at 3.3V logic levels. Connecting them directly to 5V systems can damage the Pi. The USB to Serial Converter typically handles the voltage level conversion.

**Connection Steps:**

1. **Connect the USB to Serial Converter:** Plug the USB connector into the PC.
2. **Connect the UART Pins:** Connect the TX, RX, and GND pins of the USB to Serial Converter to the corresponding GPIO pins on the Raspberry Pi (GPIO14, GPIO15, and GND).
3. **Configure UART:** Enable the UART interface on the Raspberry Pi and configure the baud rate and other settings as needed.
4. **Establish Communication:** Use a terminal emulator on the PC to establish a serial connection with the Raspberry Pi.

**Applications:**

* **Debugging and Monitoring:** Accessing the Raspberry Pi's console and monitoring system logs.
* **Remote Control:** Controlling the Raspberry Pi remotely using commands sent over the serial connection.
* **Data Transfer:** Sending and receiving data between the Raspberry Pi and the PC.

# 🛠️ Software Development and System Configuration

## 📄 Overview

This guide covers essential aspects of software development and system configuration, focusing on configuring serial port settings on a Linux-based operating system.


### 📂 Sys Folder

The `Sys Folder` section typically lists available serial ports on the system. Common serial ports include:

- `ttyS0`
- `ttyS1`
- `ttyUSB0`
- `ttyUSB1`

These ports are used for serial communication.

### 🖥️ `stty` Command

The `stty` command is a powerful utility in Linux for configuring terminal settings. Below are some examples of how to use the `stty` command to configure serial port settings.

#### Example 1: Setting Baud Rate

```
stty -F /dev/ttyS0 speed 115200
stty -F /dev/ttyS0 speed 115200 line 0 -brkint -imaxbel
```


# Serial Communication on Raspberry Pi

## Context

This guide demonstrates how to configure and test serial communication on a Raspberry Pi using terminal commands. The `stty` command is used to configure the serial port settings, while the `echo` command is used to send data to the serial port.

## Key Elements

### `stty` Command

The `stty` command is used to configure the serial port settings. Below are the key elements of the command:

* **Targeting the Serial Port:**
    ```
    stty -F /dev/ttyS0
    ```
    This command targets the serial port `/dev/ttyS0`.

* **Setting the Baud Rate:**
    ```
    stty -F /dev/ttyS0 speed 9600
    ```
    Sets the baud rate for serial communication to 9600 bits per second.

* **Resetting Line Discipline:**
    ```
    stty -F /dev/ttyS0 line 0
    ```
    Resets the line discipline to the default settings.

* **Disabling Software Flow Control:**
    ```sh
    stty -F /dev/ttyS0 -brkint -imaxbel
    ```
    Disables software flow control mechanisms.

### Sending Data

To send data to the serial port, use the `echo` command:

```
echo "hello" > /dev/ttyS0
```
**Example**

Here is a complete example of configuring and testing serial communication on a Raspberry Pi:

```
# Configure the serial port
stty -F /dev/ttyS0 speed 9600 line 0 -brkint -imaxbel

# Send data to the serial port
echo "hello" > /dev/ttyS0

# Monitor the output of the serial port
tail -f /dev/ttyS0
```




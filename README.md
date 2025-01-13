
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



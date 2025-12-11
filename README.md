# Embedded Smart Temperature Sensor

**EC535: Introduction to Embedded Systems (Fall 2025)**  
**Prof. Ayșe Coskun**  
**Final Project**
---

## Project Overview

This project implements an embedded stove monitoring system for BeagleBone Black (BBB) that monitors temperature using ADC and hardware comparator, displays status via RGB LED, and provides both command-line and GUI interfaces for control and monitoring.

---

## Project Structure

```
EC535-FinalProject/
├── README.md                    # This file - main project documentation
├── Draft1.pdf                   # Project draft document
├── Project Proposal Email Draft.txt
│
├── km/                          # Kernel Modules and Userspace Programs
│   ├── README.md                # Detailed documentation for kernel modules
│   ├── Makefile                 # Build script for kernel modules and userspace programs
│   │
│   ├── Kernel Modules:
│   ├── mySignalLED.c            # RGB LED control module (GPIO-based)
│   ├── myTempSensor.c           # Temperature sensor using ADC + comparator
│   ├── myTempSensor_comp.c      # Temperature sensor using comparator only
│   ├── mytraffic.c              # Traffic light module (Lab 4, not part of final project)
│   │
│   └── Userspace Programs:
│   ├── ul_program.c             # Main userspace program for ADC-based monitoring
│   └── ul_program_comp.c        # Userspace program for comparator-only monitoring
│
└── LCD/                         # Qt-based GUI Application
    ├── README.md                # LCD application documentation
    ├── Makefile                 # Build script for Qt application
    ├── stove.pro                # Qt project file
    │
    ├── Source Files:
    ├── main.cpp                 # Qt application entry point
    ├── stovewidget.h            # StoveWidget class header
    ├── stovewidget.cpp          # StoveWidget class implementation (GUI + kernel interface)
    │
    └── Build Artifacts:
    ├── stove                    # Compiled executable
    ├── *.o                      # Object files
    └── moc_*                    # Qt meta-object compiler files
```

---

## How to Run the System

### Prerequisites

- **BeagleBone Black** with Linux kernel 4.19.82-ti-rt-r33
- **Cross-compilation toolchain**: `arm-linux-gnueabihf-gcc`
- **Kernel source tree** at `$(EC535)/bbb/stock/stock-linux-4.19.82-ti-rt-r33`
- **Qt5** (for LCD application): widgets and network modules
- **Hardware**: RGB LED, temperature sensor (thermistor + comparator), LCD display

### Option 1: Command-Line Interface (ADC-based Temperature Sensor)

1. **Build kernel modules and userspace program:**
   ```bash
   cd km/
   make all
   ```

2. **Load kernel modules:**
   ```bash
   insmod mySignalLED.ko
   insmod myTempSensor.ko
   ```

3. **Create device files:**
   ```bash
   mknod /dev/mysignal c 61 0
   mknod /dev/mytempsensor c 62 0
   chmod 666 /dev/mysignal /dev/mytempsensor
   ```

4. **Set initial states (optional):**
   ```bash
   echo "stove_on" > /dev/mysignal
   echo "temp_below" > /dev/mysignal
   echo "server_connected" > /dev/mysignal
   echo "ref_temp=150.0" > /dev/mytempsensor
   ```

5. **Run userspace program:**
   ```bash
   ./ul_program
   ```

6. **Use commands:**
   - `on` - Turn stove ON
   - `off` - Turn stove OFF
   - `q` - Quit program

### Option 2: Command-Line Interface (Comparator-only Temperature Sensor)

1. **Build kernel modules and userspace program:**
   ```bash
   cd km/
   make all
   ```

2. **Load kernel modules:**
   ```bash
   insmod mySignalLED.ko
   insmod myTempSensor_comp.ko
   ```

3. **Create device files:**
   ```bash
   mknod /dev/mysignal c 61 0
   mknod /dev/mytempsensor_comp c 63 0
   chmod 666 /dev/mysignal /dev/mytempsensor_comp
   ```

4. **Set initial states (optional):**
   ```bash
   echo "stove_on" > /dev/mysignal
   echo "temp_below" > /dev/mysignal
   echo "server_connected" > /dev/mysignal
   echo "period=15000" > /dev/mytempsensor_comp
   ```

5. **Run userspace program:**
   ```bash
   ./ul_program_comp
   ```

6. **Use commands:**
   - `on` - Turn stove ON
   - `off` - Turn stove OFF
   - `period=<ms>` - Set timer period (e.g., `period=2000`)
   - `status` - Read current status
   - `q` - Quit program

### Option 3: GUI Application (LCD Screen)

1. **Build Qt application:**
   ```bash
   cd LCD/
   qmake && make
   ```

2. **Ensure kernel modules are loaded:**
   ```bash
   cd ../km/
   insmod mySignalLED.ko
   insmod myTempSensor_comp.ko
   mknod /dev/mysignal c 61 0
   mknod /dev/mytempsensor_comp c 63 0
   chmod 666 /dev/mysignal /dev/mytempsensor_comp
   ```

3. **Run GUI application:**
   ```bash
   cd ../LCD/
   ./stove
   ```

4. **Use GUI controls:**
   - Power button: Toggle stove on/off
   - Temperature slider: Set target temperature
   - Software limit slider: Set software temperature limit
   - Real-time temperature display and history graph

---

## How the System Works Together

### System Architecture

The system consists of three main layers:

```
┌─────────────────────────────────────────────────────────┐
│  User Interface Layer                                    │
│  ┌──────────────┐  ┌──────────────┐                    │
│  │ LCD GUI App  │  │ Userspace    │                    │
│  │ (Qt Widget)  │  │ Programs     │                    │
│  └──────┬───────┘  └──────┬───────┘                    │
└─────────┼──────────────────┼────────────────────────────┘
          │                  │
          │ /dev/mysignal    │ /dev/mytempsensor
          │ /dev/mytempsensor│ /dev/mysignal
          │ _comp            │
┌─────────┼──────────────────┼────────────────────────────┐
│ Kernel  │                  │                            │
│ Modules │                  │                            │
│ ┌───────▼───────┐  ┌───────▼───────┐  ┌──────────────┐│
│ │ mySignalLED   │  │ myTempSensor_comp │  │ myTempSensor ││
│ │ .ko           │  │ .ko           │  │ _comp.ko     ││
│ └───────┬───────┘  └───────┬───────┘  └──────┬───────┘│
└─────────┼────────────────────┼─────────────────┼────────┘
        │                         │               │
┌───────▼─────────────────────────▼───────────────▼───────┐
│ Hardware Layer                                          │
│ ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐│
│ │ RGB LED  │  │ ADC      │  │ Hardware │  │ LCD     ││
│ │ (GPIO)   │  │ (AIN0)   │  │ Comparator│  │ Display ││
│ └──────────┘  └──────────┘  └──────────┘  └─────────┘│
└─────────────────────────────────────────────────────────┘
```

### Component Interactions

#### 1. **Kernel Modules Layer**

**mySignalLED.ko** (Device: `/dev/mysignal`, Major: 61)
- Controls RGB LED via GPIO pins (Red: GPIO 67, Green: GPIO 44, Blue: GPIO 68)
- Displays three system states in sequence:
  - **Green**: Server connection status
  - **Blue**: Stove on/off status
  - **Red**: Temperature threshold status
- Continuously cycles through states while module is loaded
- Accepts commands via device file writes

**myTempSensor.ko** (Device: `/dev/mytempsensor`, Major: 62)
- Reads temperature from ADC (AIN0) and calculates temperature using thermistor equation
- Monitors hardware comparator via GPIO 26 interrupts
- Provides timer-based periodic measurements
- Supports interrupt-driven notifications via `poll()`/`select()`
- Updates status when temperature threshold is exceeded

**myTempSensor_comp.ko** (Device: `/dev/mytempsensor_comp`, Major: 63)
- Monitors hardware comparator only (no ADC)
- More power-efficient than ADC-based version
- Supports interrupt-driven notifications via `poll()`/`select()` and `fasync()` (SIGIO)
- Periodically polls comparator state

#### 2. **Userspace Programs Layer**

**ul_program** (ADC-based)
- Opens `/dev/mytempsensor` and `/dev/mysignal`
- Continuously reads temperature from ADC-based sensor
- Updates LED state based on temperature threshold
- Handles interrupt events from kernel module
- Provides command-line interface for stove control
- Multi-threaded: separate threads for temperature monitoring and user input

**ul_program_comp** (Comparator-only)
- Opens `/dev/mytempsensor_comp` and `/dev/mysignal`
- Monitors comparator state via interrupts and polling
- Updates LED state based on comparator threshold
- Handles interrupt events from kernel module
- Provides command-line interface for stove control and timer period adjustment
- Multi-threaded: separate threads for comparator monitoring, server updates, and user input

#### 3. **GUI Application Layer**

**LCD/stove** (Qt Widget Application)
- Opens `/dev/mysignal` and `/dev/mytempsensor_comp`
- Provides graphical user interface with:
  - Power button to toggle stove on/off
  - Temperature slider for target temperature
  - Software limit slider for safety
  - Real-time temperature display
  - Temperature history graph
- Synchronizes LED state with GUI state
- Monitors hardware comparator for emergency shutdown
- Updates display based on temperature readings

### Data Flow

1. **Temperature Sensing:**
   - Hardware: Thermistor → ADC (AIN0) or Hardware Comparator → GPIO 26
   - Kernel: `myTempSensor.ko` or `myTempSensor_comp.ko` reads hardware
   - Userspace: Programs read from `/dev/mytempsensor` or `/dev/mytempsensor_comp`
   - GUI: `stovewidget.cpp` reads comparator status and displays temperature

2. **LED Control:**
   - Userspace/GUI: Writes commands to `/dev/mysignal` (e.g., `stove_on`, `temp_above`)
   - Kernel: `mySignalLED.ko` receives commands and updates internal state
   - Hardware: Kernel module controls GPIO pins to drive RGB LED
   - Visual: LED displays system states in sequence

3. **State Synchronization:**
   - Userspace programs and GUI application both interface with same kernel modules
   - State changes in one interface are reflected in LED display
   - Kernel modules maintain consistent state across all userspace interfaces

### Communication Protocol

**Device File Interface:**
- **Reading**: `cat /dev/device` returns current status
- **Writing**: `echo "command" > /dev/device` sends commands
- **Polling**: `poll()`/`select()` for interrupt-driven notifications

**Command Format:**
- LED: `stove_on`, `stove_off`, `temp_above`, `temp_below`, `server_connected`, `server_disconnected`
- Temperature: `ref_temp=150.0`, `period=1000`
- All commands are case-sensitive

---

## Quick Start Guide

### Minimal Setup (Comparator-only with GUI)

```bash
# 1. Build kernel modules
cd km/
make all

# 2. Load modules and create device files
insmod mySignalLED.ko
insmod myTempSensor_comp.ko
mknod /dev/mysignal c 61 0
mknod /dev/mytempsensor_comp c 63 0
chmod 666 /dev/mysignal /dev/mytempsensor_comp

# 3. Build and run GUI
cd ../LCD/
qmake && make
./stove
```

---

## Additional Documentation

- **Kernel Modules**: See `km/README.md` for detailed documentation on kernel modules and userspace programs
- **LCD Application**: See `LCD/README.md` for Qt application details

---

## Troubleshooting

- **Module won't load**: Check `dmesg` for errors, verify GPIO pins are available
- **Device file not found**: Create manually with `mknod` (see run instructions above)
- **LEDs not responding**: Verify GPIO pin assignments match hardware
- **Temperature readings incorrect**: Check ADC channel and thermistor connections
- **GUI won't start**: Ensure Qt5 is installed and kernel modules are loaded

---

## Author

Yash Patel & Leah Jones- EC535 Final Project (Fall 2025)

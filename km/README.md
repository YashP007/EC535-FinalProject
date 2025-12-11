# EC535 Final Project - Kernel Modules and Userspace Programs

## Overview

This directory contains kernel modules and userspace programs for an embedded stove monitoring system running on BeagleBone Black. The system monitors temperature using hardware comparator, displays status via RGB LED, and provides userspace interfaces for control and monitoring.

**Final Implementation (Demo Day):** The system used `myTempSensor_comp.c`, `ul_program_comp.c` (for Adafruit-based control and debugging), and `mySignalLED.c`.

---

## Files Overview

| File | Type | Description |
|------|------|-------------|
| `Makefile` | Build script | Builds kernel modules and userspace programs |
| `mySignalLED.c` | Kernel module | RGB LED control for system status display |
| `myTempSensor_comp.c` | Kernel module | Temperature sensor using comparator only |
| `ul_program_comp.c` | Userspace program | Program for comparator-only monitoring with Adafruit control/debugging |
| `README.md` | Documentation | This file |

---

## Building

### Prerequisites

- BeagleBone Black with Linux kernel 4.19.82-ti-rt-r33
- Cross-compilation toolchain: `arm-linux-gnueabihf-gcc`
- Kernel source tree at `$(EC535)/bbb/stock/stock-linux-4.19.82-ti-rt-r33`

### Build Commands

```bash
# Build all modules and programs
make all

# Build only kernel modules
make

# Build only userspace program
make ul_program_comp

# Clean build artifacts
make clean
```

---

## Final Implementation Setup

### Kernel Modules Used

1. **mySignalLED.ko** - RGB LED control module
2. **myTempSensor_comp.ko** - Comparator-only temperature sensor module

### Userspace Program Used

1. **ul_program_comp** - Comparator monitoring with Adafruit-based control and debugging

### Quick Start

```bash
# Build everything
make all

# Load kernel modules
insmod mySignalLED.ko
insmod myTempSensor_comp.ko

# Create device files
mknod /dev/mysignal c 61 0
mknod /dev/mytempsensor_comp c 63 0
chmod 666 /dev/mysignal /dev/mytempsensor_comp

# Set initial states
echo "stove_on" > /dev/mysignal
echo "temp_below" > /dev/mysignal
echo "server_connected" > /dev/mysignal
echo "period=15000" > /dev/mytempsensor_comp

# Run userspace program
./ul_program_comp
```

---

## Kernel Module: mySignalLED

**Device File:** `/dev/mysignal` (major 61, minor 0)

Controls RGB LED displaying three states:
- **Green**: Server connection status
- **Blue**: Stove on/off status  
- **Red**: Temperature threshold status

### Commands

```bash
# Set states
echo "stove_on" > /dev/mysignal
echo "stove_off" > /dev/mysignal
echo "temp_above" > /dev/mysignal
echo "temp_below" > /dev/mysignal
echo "server_connected" > /dev/mysignal
echo "server_disconnected" > /dev/mysignal

# Configure timing
echo "color_duration=1000" > /dev/mysignal  # 1-10000 ms
echo "sleep_duration=2000" > /dev/mysignal  # 1-60000 ms

# Read status
cat /dev/mysignal
```

### GPIO Pins

- **RED**: GPIO 67 (Temperature threshold)
- **GREEN**: GPIO 44 (Server connection)
- **BLUE**: GPIO 68 (Stove on/off)

### LED Sequence

**Normal (server connected):** Sleep → Green → Blue → Red → repeat  
**Disconnected:** Sleep → All ON → Blue → Red → repeat

---

## Kernel Module: myTempSensor_comp

**Device File:** `/dev/mytempsensor_comp` (major 63, minor 0)

Monitors temperature threshold using hardware comparator only (no ADC). Uses GPIO 26 for comparator input with falling edge interrupts.

### Commands

```bash
# Read status
cat /dev/mytempsensor_comp
# Output: comparator=<0|1> comparator_gpio=<0|1> period=<ms> triggered=<0|1>

# Set timer period
echo "period=15000" > /dev/mytempsensor_comp  # 100-60000 ms
```

### Features

- Hardware comparator interrupts (GPIO 26, falling edge)
- Periodic polling for status updates
- Supports `poll()`/`select()` and `fasync()` (SIGIO) for notifications
- Default check period: 15 seconds

---

## Userspace Program: ul_program_comp

Interfaces with `myTempSensor_comp.ko` and `mySignalLED.ko`. Provides Adafruit-based control and debugging interface.

### Commands

- `on` - Turn stove ON
- `off` - Turn stove OFF
- `period=<ms>` - Set timer period
- `status` - Read current status
- `q` - Quit program

### Features

- Comparator monitoring via interrupts and polling
- LED control based on comparator state
- Timer period control
- Server update simulation
- Multi-threaded architecture

---

## Troubleshooting

- **Module won't load:** Check `dmesg` for errors, verify GPIO pins available
- **Device file not found:** Create with `mknod` and set permissions with `chmod 666`
- **LEDs not responding:** Verify GPIO pin assignments match hardware
- **Comparator not triggering:** Check GPIO 26 wiring and comparator output

---

## Notes

- All state changes are atomic and thread-safe
- GPIO pin numbers may need adjustment based on hardware configuration
- Comparator-only version is more power-efficient (no ADC usage)

---

## Author

Yash Patel - EC535 Final Project

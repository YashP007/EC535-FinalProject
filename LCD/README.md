# LCD Screen Application

This directory contains the Qt-based LCD screen application for the stove control system, designed to run on the BeagleBone Black (BBB).

## Compilation

To compile the code for the LCD screen, run:

```bash
qmake && make
```

This will generate the executable `stove` in the current directory.

## Running on BeagleBone Black

After compilation, run the application on the BBB with:

```bash
./stove
```

The application will display the stove control interface on the connected LCD screen.

## Requirements

- Qt5 (widgets and network modules)
- C++11 compatible compiler
- BeagleBone Black with appropriate display hardware


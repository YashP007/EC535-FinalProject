# Stove Monitoring LED Kernel Module

## Overview
This kernel module (`mySignalLED`) controls an RGB LED for an embedded stove monitoring system. It displays three states through a color sequence:
- **Green**: Server connection status (connected/disconnected)
- **Blue**: Stove on/off status
- **Red**: Temperature threshold status (above/below)

## Building and Loading

### Build the module:
```bash
make
```

### Load the module:
```bash
insmod mySignalLED.ko
```

### Unload the module:
```bash
rmmod mySignalLED
```

### Check module status:
```bash
dmesg -c -s 20
```

## Device File
The module creates a character device at `/dev/mysignal` (major 61, minor 0).

**Note**: You may need to create the device node manually:
```bash
sudo mknod /dev/mysignal c 61 0
sudo chmod 666 /dev/mysignal
```

## Usage

### Setting States

Write commands to `/dev/mysignal` to control the system states:

#### Stove State:
```bash
echo "stove_on" | sudo tee /dev/mysignal
echo "stove_off" | sudo tee /dev/mysignal
```

#### Temperature State:
```bash
echo "temp_above" | sudo tee /dev/mysignal
echo "temp_below" | sudo tee /dev/mysignal
```

#### Server Connection State:
```bash
echo "server_connected" | sudo tee /dev/mysignal
echo "server_disconnected" | sudo tee /dev/mysignal
```

### Configuring Timing

#### Set Color Display Duration (milliseconds):
```bash
echo "color_duration=1000" | sudo tee /dev/mysignal
```
- Valid range: 1-10000 ms
- Default: 1000 ms (1 second)

#### Set Sleep Duration Between Cycles (milliseconds):
```bash
echo "sleep_duration=2000" | sudo tee /dev/mysignal
```
- Valid range: 1-60000 ms
- Default: 2000 ms (2 seconds)

### Reading Status

Read the current status:
```bash
cat /dev/mysignal
```

Output format:
```
stove=<on|off> temp=<above|below> server=<connected|disconnected> color_duration=<ms> sleep_duration=<ms> leds={red:<on|off>,green:<on|off>,blue:<on|off>} state=<state_number>
```

## LED Behavior

### Normal Sequence (when server is connected):
1. **Sleep** (2 seconds default)
2. **Green LED** (1 second) - Shows server connection status
3. **Blue LED** (1 second) - Shows stove on/off (off if stove is off)
4. **Red LED** (1 second) - Shows temperature threshold status
5. Repeat from step 1

### Special Case (when server is disconnected):
1. **Sleep** (2 seconds default)
2. **All LEDs ON** (1 second) - Visual indicator that sequence is starting
3. **Blue LED** (1 second) - Shows stove on/off (off if stove is off)
4. **Red LED** (1 second) - Shows temperature threshold status
5. Repeat from step 1

## Example Usage

### Complete setup example:
```bash
# Load module
sudo insmod mySignalLED.ko

# Set states
echo "stove_on" | sudo tee /dev/mysignal
echo "temp_above" | sudo tee /dev/mysignal
echo "server_connected" | sudo tee /dev/mysignal

# Adjust timing (optional)
echo "color_duration=1500" | sudo tee /dev/mysignal
echo "sleep_duration=3000" | sudo tee /dev/mysignal

# Check status
cat /dev/mysignal
```

## GPIO Pin Assignments
- **RED**: GPIO 67 (Temperature threshold)
- **GREEN**: GPIO 44 (Server connection)
- **BLUE**: GPIO 68 (Stove on/off)

**Note**: Verify these GPIO numbers match your hardware configuration.

## Troubleshooting

- **Module won't load**: Check `dmesg` for error messages. Verify GPIO pins are not in use.
- **Device file not found**: Create it manually with `mknod` (see Device File section).
- **Unknown command warning**: Check command spelling. Valid commands are listed above.
- **LEDs not responding**: Verify GPIO pin assignments match your hardware.

## Notes
- All state changes are atomic and thread-safe
- Timing changes take effect on the next cycle
- The module continuously cycles through the LED sequence while loaded
- Invalid timing values are rejected with a warning message


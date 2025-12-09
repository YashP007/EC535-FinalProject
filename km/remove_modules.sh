#!/bin/bash
# remove_modules.sh - Simple script to remove kernel modules
#
# Edit the MODULES array below to change which modules are removed
# Format: "module_name:major:device_path"

# Configuration - Edit this section to change modules
MODULES=(
    "myTempSensor_comp:63:/dev/mytempsensor_comp"
    "mySignalLED:61:/dev/mysignal"
)

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Removing kernel modules..."

# Remove device files
echo "Removing device files..."
for module_info in "${MODULES[@]}"; do
    IFS=':' read -r module_name major_num device_file <<< "$module_info"
    
    if [ -e "$device_file" ]; then
        rm -f "$device_file"
        echo "  ✓ Removed $device_file"
    fi
done

# Unload modules
echo "Unloading modules..."
for module_info in "${MODULES[@]}"; do
    IFS=':' read -r module_name major_num device_file <<< "$module_info"
    
    if lsmod | grep -q "^${module_name}"; then
        rmmod "${module_name}" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "  ✓ Unloaded ${module_name}"
        else
            echo "  ✗ Failed to unload ${module_name} (may be in use)"
        fi
    fi
done

# Remove .ko and .o files from directory
echo "Cleaning build files..."
rm -f *.ko *.o *.mod.o *.mod.c *.order *.symvers .*.cmd 2>/dev/null || true
echo "  ✓ Removed .ko and .o files"

echo "Removal complete!"

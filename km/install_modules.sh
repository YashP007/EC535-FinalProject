#!/bin/bash
# install_modules.sh - Simple script to install kernel modules
#
# Edit the MODULES array below to change which modules are installed
# Format: "module_name:major:device_path"

# Configuration - Edit this section to change modules
MODULES=(
    "mySignalLED:61:/dev/mysignal"
    "myTempSensor_comp:63:/dev/mytempsensor_comp"
)

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Installing kernel modules..."

# Check if all .ko files exist
echo "Checking for .ko files..."
for module_info in "${MODULES[@]}"; do
    IFS=':' read -r module_name major_num device_file <<< "$module_info"
    ko_file="${module_name}.ko"
    
    if [ ! -f "$ko_file" ]; then
        echo "Error: $ko_file not found!"
        exit 1
    fi
    echo "  ✓ Found $ko_file"
done

# Remove old modules if loaded
echo "Checking for old loaded modules..."
for module_info in "${MODULES[@]}"; do
    IFS=':' read -r module_name major_num device_file <<< "$module_info"
    
    if lsmod | grep -q "^${module_name}"; then
        echo "  Removing old ${module_name}..."
        rmmod "${module_name}" 2>/dev/null || true
    fi
done

# Install modules
echo "Installing modules..."
for module_info in "${MODULES[@]}"; do
    IFS=':' read -r module_name major_num device_file <<< "$module_info"
    ko_file="${module_name}.ko"
    
    # Remove existing device file
    rm -f "$device_file" 2>/dev/null || true
    
    # Create device file
    mknod "$device_file" c "$major_num" 0
    chmod 666 "$device_file"
    echo "  ✓ Created $device_file"
    
    # Load module
    insmod "$ko_file"
    if [ $? -eq 0 ]; then
        echo "  ✓ Loaded ${module_name}"
    else
        echo "  ✗ Failed to load ${module_name}"
        exit 1
    fi
done

echo "Installation complete!"

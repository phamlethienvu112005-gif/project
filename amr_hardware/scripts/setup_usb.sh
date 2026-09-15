#!/bin/bash
# =============================================================================
# AMR USB Setup Script
# Purpose: Configure udev rules and verify ESP32/RPLiDAR symlinks
# Usage: ./setup_usb.sh [--install] [--check] [--help]
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Paths
RULES_SOURCE="/home/thienvu/project_ws/docs/99-amr-usb.rules"
RULES_TARGET="/etc/udev/rules.d/99-amr-usb.rules"

# =============================================================================
# Functions
# =============================================================================

print_banner() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "  AMR USB Setup Script"
    echo "  ESP32 + RPLiDAR Configuration"
    echo "========================================${NC}"
    echo ""
}

print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[i]${NC} $1"
}

# =============================================================================
# Check USB devices and symlinks
# =============================================================================

check_symlinks() {
    echo -e "\n${BLUE}=== Checking USB Symlinks ===${NC}"
    echo ""

    local has_error=0

    # Check ESP32 symlink
    if [ -L "/dev/esp32" ]; then
        local esp32_target=$(readlink -f /dev/esp32)
        echo -e "  /dev/esp32 -> ${GREEN}${esp32_target}${NC}"

        # Get device info
        if [ -e "/dev/esp32" ]; then
            local esp32_dev_info=$(udevadm info -a -n /dev/esp32 2>/dev/null | grep -E "ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{devnum}" | head -3)
            echo "  Device info: $(echo "$esp32_dev_info" | tr '\n' ' ')"
        fi
    else
        echo -e "  /dev/esp32: ${RED}NOT FOUND (symlink missing)${NC}"
        has_error=1
    fi

    echo ""

    # Check RPLiDAR symlink
    if [ -L "/dev/rplidar" ]; then
        local rplidar_target=$(readlink -f /dev/rplidar)
        echo -e "  /dev/rplidar -> ${GREEN}${rplidar_target}${NC}"

        # Get device info
        if [ -e "/dev/rplidar" ]; then
            local rplidar_dev_info=$(udevadm info -a -n /dev/rplidar 2>/dev/null | grep -E "ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{devnum}" | head -3)
            echo "  Device info: $(echo "$rplidar_dev_info" | tr '\n' ' ')"
        fi
    else
        echo -e "  /dev/rplidar: ${YELLOW}NOT FOUND (may not be connected)${NC}"
    fi

    echo ""

    # Check for symlink conflict
    if [ -L "/dev/esp32" ] && [ -L "/dev/rplidar" ]; then
        local esp32_real=$(readlink -f /dev/esp32)
        local rplidar_real=$(readlink -f /dev/rplidar)

        if [ "$esp32_real" == "$rplidar_real" ]; then
            echo -e "${RED}========================================${NC}"
            echo -e "${RED}[CRITICAL] SYMLINK CONFLICT DETECTED!${NC}"
            echo -e "${RED}========================================${NC}"
            echo -e "  /dev/esp32 và /dev/rplidar cùng trỏ đến: $esp32_real"
            echo ""
            echo -e "${YELLOW}Nguyên nhân: Cả 2 thiết bị dùng chip CP210x với VID:PID giống nhau${NC}"
            echo -e "${YELLOW}             usbipd không giữ nguyên USB bus topology${NC}"
            echo ""
            echo "Hành động khắc phục:"
            echo "  1. Gỡ và gắn lại thiết bị USB theo thứ tự đúng"
            echo "  2. Hoặc chạy: sudo $(basename $0) --install"
            echo "  3. Hoặc kiểm tra udev rules: cat $RULES_TARGET"
            echo ""
            has_error=1
        else
            echo -e "${GREEN}[✓] Không có xung đột symlink${NC}"
        fi
    fi

    echo ""

    # List all ttyUSB devices
    echo -e "${BLUE}=== All ttyUSB Devices ===${NC}"
    for dev in /dev/ttyUSB*; do
        if [ -e "$dev" ]; then
            local real_path=$(readlink -f "$dev" 2>/dev/null || echo "N/A")
            echo "  $dev -> $real_path"
        fi
    done

    echo ""
    return $has_error
}

# =============================================================================
# Install udev rules
# =============================================================================

install_rules() {
    echo -e "\n${BLUE}=== Installing Udev Rules ===${NC}"
    echo ""

    # Check if source file exists
    if [ ! -f "$RULES_SOURCE" ]; then
        print_error "Source rules file not found: $RULES_SOURCE"
        echo "Tạo rules file mới..."

        # Create the rules file
        sudo tee "$RULES_SOURCE" > /dev/null << 'RULES_EOF'
# =============================================================================
# AMR USB Device Rules - Symlink Management
# Created: 2026-09-14
# Purpose: Fix /dev/ttyUSBx swapping issue between ESP32 and RPLiDAR
# =============================================================================

# ESP32: First CP210x device
SUBSYSTEM=="tty", SUBSYSTEMS=="usb-serial", \
    ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    ATTRS{devnum}=="7", \
    SYMLINK+="esp32", MODE="0666", GROUP="dialout", \
    OPTIONS="link_priority=200"

# RPLiDAR: Second CP210x device
SUBSYSTEM=="tty", SUBSYSTEMS=="usb-serial", \
    ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    ATTRS{devnum}=="8", \
    SYMLINK+="rplidar", MODE="0666", GROUP="dialout", \
    OPTIONS="link_priority=200"

# Fallback: Direct VID:PID match
SUBSYSTEM=="tty", SUBSYSTEMS=="usb-serial", \
    ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    SYMLINK+="esp32", MODE="0666", GROUP="dialout", \
    OPTIONS="link_priority=-100", \
    GOTO="amr_usb_end"

SUBSYSTEM=="tty", SUBSYSTEMS=="usb-serial", \
    ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    SYMLINK+="rplidar", MODE="0666", GROUP="dialout", \
    OPTIONS="link_priority=-200"

LABEL="amr_usb_end"
RULES_EOF
        print_status "Created rules file: $RULES_SOURCE"
    fi

    # Copy to system
    print_info "Copying rules to /etc/udev/rules.d/..."
    sudo cp "$RULES_SOURCE" "$RULES_TARGET"
    sudo chmod 644 "$RULES_TARGET"
    print_status "Copied to $RULES_TARGET"

    # Reload udev rules
    echo ""
    print_info "Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    print_status "Udev rules reloaded"

    echo ""
    print_info "Waiting for devices to settle..."
    sleep 2

    # Verify
    echo ""
    check_symlinks
}

# =============================================================================
# Show USB device details
# =============================================================================

show_device_details() {
    echo -e "\n${BLUE}=== USB Device Details ===${NC}"
    echo ""

    for dev in /dev/esp32 /dev/rplidar; do
        if [ -e "$dev" ]; then
            echo -e "${GREEN}=== $dev ===${NC}"
            udevadm info -a -n "$dev" 2>/dev/null | grep -E "KERNELS|ATTRS" | grep -v "power" | head -25
            echo ""
        else
            echo -e "${YELLOW}=== $dev === NOT FOUND${NC}"
            echo ""
        fi
    done
}

# =============================================================================
# Help
# =============================================================================

show_help() {
    echo "Usage: $(basename $0) [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --install    Install udev rules and reload"
    echo "  --check     Check symlinks only (default if no option)"
    echo "  --detail    Show detailed USB device info"
    echo "  --help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./setup_usb.sh --check      # Check symlinks"
    echo "  ./setup_usb.sh --install   # Install rules and check"
    echo "  ./setup_usb.sh             # Check symlinks (default)"
}

# =============================================================================
# Main
# =============================================================================

main() {
    print_banner

    case "${1:-}" in
        --install|-i)
            install_rules
            ;;
        --check|-c)
            check_symlinks
            ;;
        --detail|-d)
            show_device_details
            ;;
        --help|-h)
            show_help
            ;;
        "")
            check_symlinks
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
}

main "$@"

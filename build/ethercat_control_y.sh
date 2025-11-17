#!/bin/bash

# EtherCAT Y-Axis Control (Bus Position 3)
# Wrapper script that calls ethercat_control_single_axis.sh with bus_position=3

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Usage function
usage() {
    echo "Usage: $0 [--relative | --absolute] [--velocity=<value>] -- <target_position>"
    echo "  target_position: Target position in micrometers (µm) (must come after --)"
    echo "  --relative:      Move relative to current position"
    echo "  --absolute:      Move to absolute position (default)"
    echo "  --velocity=<val>: Set velocity in µm/s (must be positive integer, default: 1000)"
    echo ""
    echo "This script controls the Y-axis slave at bus position 3"
    echo ""
    echo "Examples:"
    echo "  $0 -- 100000                          # Move to absolute position 100000 µm (10 cm)"
    echo "  $0 --relative -- 50000                # Move 50000 µm (5 cm) relative to current position"
    echo "  $0 --relative --velocity=500 -- -25000 # Move -25000 µm backward at 500 µm/s"
    echo "  $0 --velocity=2000 -- 100000          # Move to 100000 µm at 2000 µm/s (2 mm/s)"
    exit 1
}

# Check if at least one argument is provided
if [ $# -lt 1 ]; then
    echo "Error: Target position is required after --!"
    usage
fi

echo "Y-Axis Control: Calling ethercat_control_single_axis.sh with bus_position=3"
echo ""

# Call the main script with bus_position=3 and all provided arguments
exec "$SCRIPT_DIR/ethercat_control_single_axis.sh" --bus_position=3 "$@"

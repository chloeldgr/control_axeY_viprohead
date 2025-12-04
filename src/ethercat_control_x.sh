#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# EtherCAT Dual Axis/ Dual Slave Position Control 
# Control the bridge axis X of the ProfiloBot (axis X1 and axis X2)
# Controls slaves at bus positions 1 and 2 with position commands

# Usage function
usage() {
    echo "Usage: $0 [--relative | --absolute] [--velocity=<value>] -- <target_position>"
    echo "  target_position: Target position in micrometers (µm) (must come after --)"
    echo "  --relative:      Move relative to current position"
    echo "  --absolute:      Move to absolute position (default)"
    echo "  --velocity=<val>: Set velocity in µm/s (must be positive integer, default: 1000)"
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

# Parse arguments
MODE="absolute"  # Default mode
VELOCITY=1000    # Default velocity in µm/s (1 mm/s)
TARGET_POSITION=""

# Parse optional arguments until -- is found
while [[ $# -gt 0 ]]; do
    case $1 in
        --relative)
            MODE="relative"
            shift
            ;;
        --absolute)
            MODE="absolute"
            shift
            ;;
        --velocity=*)
            VELOCITY_INPUT="${1#*=}"
            # Validate velocity is a positive integer
            if ! [[ "$VELOCITY_INPUT" =~ ^[0-9]+$ ]] || [ "$VELOCITY_INPUT" -le 0 ]; then
                echo "Error: Velocity must be a positive integer (in µm/s)"
                echo "Given: $VELOCITY_INPUT"
                usage
            fi
            VELOCITY="$VELOCITY_INPUT"
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "Error: Invalid argument '$1'. Use -- before target position."
            usage
            ;;
    esac
done

# Check if target position is provided after --
if [ $# -ne 1 ]; then
    echo "Error: Exactly one target position must be provided after --"
    usage
fi

TARGET_POSITION=$1

# Validate target position is a number
if ! [[ "$TARGET_POSITION" =~ ^-?[0-9]+$ ]]; then
    echo "Error: Target position must be a number (in micrometers)"
    usage
fi

echo "=============================================="
echo "EtherCAT Dual Slave Position Control"
echo "=============================================="
echo "Target Position: $TARGET_POSITION µm"
echo "Mode: $MODE"
echo "Velocity: $VELOCITY µm/s"
echo "=============================================="

# Function to display position of a slave
display_position() {
    local bus_position=$1
    local label=$2
    echo -n "  $label position: "
    if position=$(ethercat upload --position $bus_position --type int32 0x6063 0 2>/dev/null); then
        echo "$position µm"
    else
        echo "Failed to read position"
    fi
}

# Function to display positions of both slaves
display_both_positions() {
    local label=$1
    echo "$label positions:"
    display_position 1 "X1"
    display_position 2 "X2"
}

# Function to control a single slave
config_slave() {
    local bus_position=$1
    local target_pos=$2
    local velocity=$3
    
    echo "Configuring slave at bus position $bus_position..."
    
    # 1) Put the slave in OP (if your setup/master allows it)
    echo "  Setting slave $bus_position to OP state..."
    if ! ethercat state --position $bus_position OP; then
        echo "Error: Failed to set slave $bus_position to OP state"
        exit 1
    fi
    
    # 2) Set mode of operation (1 = profile position mode)
    echo "  Setting profile position mode..."
    if ! ethercat download --position $bus_position --type int8 0x6060 0 1; then
        echo "Error: Failed to set profile position mode for slave $bus_position"
        exit 1
    fi
    
    # 3) Set the velocity
    echo "  Setting velocity to $velocity µm/s..."
    if ! ethercat download --position $bus_position --type uint32 0x6081 0 $velocity; then
        echo "Error: Failed to set velocity for slave $bus_position"
        exit 1
    fi
    
    # 4) Set target position
    echo "  Setting target position to $target_pos µm..."
    if ! ethercat download --position $bus_position --type int32 0x607A 0 -- $target_pos; then
        echo "Error: Failed to set target position for slave $bus_position"
        exit 1
    fi
    
    # 5) CiA-402 enable sequence
    echo "  Executing CiA-402 enable sequence..."
    if ! ethercat download --position $bus_position --type uint16 0x6040 0 0x0006; then   # Shutdown
        echo "Error: Failed to execute shutdown command for slave $bus_position"
        exit 1
    fi
    sleep 0.1
    if ! ethercat download --position $bus_position --type uint16 0x6040 0 0x0007; then   # Switch On
        echo "Error: Failed to execute switch on command for slave $bus_position"
        exit 1
    fi
    sleep 0.1
    if ! ethercat download --position $bus_position --type uint16 0x6040 0 0x000F; then   # Enable Operation
        echo "Error: Failed to execute enable operation command for slave $bus_position"
        exit 1
    fi
    sleep 0.1
}


# Function to stop both motors for safety
stop_both_motors() {
    echo "Stopping both motors for safety..."
    local max_attempts=10
    local attempt=1
    local slave1_stopped=false
    local slave2_stopped=false
    
    while [ $attempt -le $max_attempts ] && { [ "$slave1_stopped" = false ] || [ "$slave2_stopped" = false ]; }; do
        echo "  Attempt $attempt to stop motors..."
        
        # Try to stop slave 1 if not already stopped
        if [ "$slave1_stopped" = false ]; then
            if ethercat download --position 1 --type uint16 0x6040 0 0x0006 2>/dev/null; then
                echo "  Slave 1 stopped successfully"
                slave1_stopped=true
            else
                echo "  Failed to stop slave 1"
            fi
        fi
        
        # Try to stop slave 2 if not already stopped
        if [ "$slave2_stopped" = false ]; then
            if ethercat download --position 2 --type uint16 0x6040 0 0x0006 2>/dev/null; then
                echo "  Slave 2 stopped successfully"
                slave2_stopped=true
            else
                echo "  Failed to stop slave 2"
            fi
        fi
        
        # If both are stopped, break out of the loop
        if [ "$slave1_stopped" = true ] && [ "$slave2_stopped" = true ]; then
            echo "Both motors stopped successfully"
            break
        fi
        
        attempt=$((attempt + 1))
        sleep 0.001
    done
    
    # Check if we exceeded max attempts
    if [ $attempt -gt $max_attempts ]; then
        echo "Warning: Could not stop all motors after $max_attempts attempts, push emergency stop!"
        if [ "$slave1_stopped" = false ]; then
            echo "  Slave 1 could not be stopped"
        fi
        if [ "$slave2_stopped" = false ]; then
            echo "  Slave 2 could not be stopped"
        fi
    fi
}

start_absolute_motion_slave() {
    local bus_position=$1
    # Start the motion for absolute move
    if ! ethercat download --position $bus_position --type uint16 0x6040 0 155; then  # Start Motion 
        echo "Error: Failed to start absolute motion for slave $bus_position"
        stop_both_motors
        return 1
    fi
    # 155 : 0b 1001 1011 
    # bit 7 : 1= Fault reset on / bit 6 : 0= absolute / bit 5: 0= no change set immediately / bit 4 : 1= new setpoint
    # bit 3 : 1= Enable operation / bit 2 : 0= no Quick stop / bit 1: 1= Enable Voltage / bit 0= switch on
    return 0
}

start_relative_motion_slave() {
    local bus_position=$1
    # Start the motion for relative move
    if ! ethercat download --position $bus_position --type uint16 0x6040 0 219; then   # Start Motion
        echo "Error: Failed to start relative motion for slave $bus_position"
        stop_both_motors
        return 1
    fi
    # 219 : 0b 1101 1011
    # bit 7 : 1= Fault reset on / bit 6 : 1= relative / bit 5: 0=no change set immediately / bit 4 : 1= new setpoint
    # bit 3 : 1= Enable operation / bit 2 : 0= no Quick stop / bit 1: 1= Enable Voltage / bit 0= switch on
    return 0
}

# Control both slaves
echo ""
echo "=============================================="
display_both_positions "Initial"
echo "=============================================="

echo ""
echo "Configuring Slave 1 (position 1):"
config_slave 1 $TARGET_POSITION $VELOCITY

echo ""
echo "Configuring Slave 2 (position 2):"
config_slave 2 $TARGET_POSITION $VELOCITY

echo ""
echo "=============================================="
display_both_positions "Pre-motion"
echo "=============================================="

# Start motion on both slaves
echo ""
echo "Starting motion..."
motion_success=true
if [ "$MODE" == "absolute" ]; then
    if ! start_absolute_motion_slave 1; then
        motion_success=false
    elif ! start_absolute_motion_slave 2; then
        motion_success=false
    fi
else
    if ! start_relative_motion_slave 1; then
        motion_success=false
    elif ! start_relative_motion_slave 2; then
        motion_success=false
    fi
fi

echo ""
echo "=============================================="
if [ "$motion_success" = true ]; then
    echo "Motion commands sent to both slaves!"
    display_both_positions "Post-motion"
else
    echo "Motion failed!"
    display_both_positions "Final"
fi
echo "=============================================="

# Exit with error code if motion failed
if [ "$motion_success" = false ]; then
    exit 1
fi
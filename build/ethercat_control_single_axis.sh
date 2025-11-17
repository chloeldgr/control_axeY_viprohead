#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# EtherCAT Mono Axis / Slave Position Control 

# Usage function
usage() {
    echo "Usage: $0 [--bus_alias=<alias>] [--bus_position=<pos>] [--relative | --absolute] [--velocity=<value>] [--shutdown_at_the_end] -- <target_position>"
    echo "  target_position:     Target position in micrometers (µm) (must come after --)"
    echo "  --bus_position=<pos>: EtherCAT bus position (mandatory if --bus_alias not provided)"
    echo "  --bus_alias=<alias>: EtherCAT slave bus alias (mandatory if --bus_position not provided)"
    echo "  --relative:          Move relative to current position"
    echo "  --absolute:          Move to absolute position (default)"
    echo "  --velocity=<val>:    Set velocity in µm/s (must be positive integer, default: 1000)"
    echo "  --shutdown_at_the_end: Shutdown the motor after motion completes"
    echo ""
    echo "Examples:"
    echo "  $0 --bus_position=1 -- 100000                                    # Move to absolute position using bus position 1"
    echo "  $0 --bus_alias=2 --relative -- 50000                            # Move 50000 µm relative using bus alias 2"
    echo "  $0 --bus_position=3 --relative --velocity=500 -- -25000         # Move backward at 500 µm/s using bus position 3"
    echo "  $0 --bus_alias=1 --shutdown_at_the_end -- 100000                # Move and shutdown motor after completion"
    exit 1
}

# Check if at least two arguments are provided (bus_position/bus_alias + target_position after --)
if [ $# -lt 2 ]; then
    echo "Error: Either --bus_position or --bus_alias and target position after -- are required!"
    usage
fi

# Parse arguments
MODE="absolute"  # Default mode
VELOCITY=1000    # Default velocity in µm/s (1 mm/s)
BUS_POSITION=""  # Will be set from arguments
BUS_ALIAS=""     # Will be set from arguments
TARGET_POSITION=""
SHUTDOWN_AT_END=false  # Default: don't shutdown at end

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
        --bus_position=*)
            BUS_POSITION_INPUT="${1#*=}"
            # Validate bus_position is a non-negative integer
            if ! [[ "$BUS_POSITION_INPUT" =~ ^[0-9]+$ ]]; then
                echo "Error: Bus position must be a non-negative integer"
                echo "Given: $BUS_POSITION_INPUT"
                usage
            fi
            BUS_POSITION="$BUS_POSITION_INPUT"
            shift
            ;;
        --bus_alias=*)
            BUS_ALIAS_INPUT="${1#*=}"
            # Validate bus_alias is a non-negative integer
            if ! [[ "$BUS_ALIAS_INPUT" =~ ^[0-9]+$ ]]; then
                echo "Error: Bus alias must be a non-negative integer"
                echo "Given: $BUS_ALIAS_INPUT"
                usage
            fi
            BUS_ALIAS="$BUS_ALIAS_INPUT"
            shift
            ;;
        --shutdown_at_the_end)
            SHUTDOWN_AT_END=true
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

# Validate that at least one of bus_position or bus_alias is provided
if [ -z "$BUS_POSITION" ] && [ -z "$BUS_ALIAS" ]; then
    echo "Error: Either --bus_position or --bus_alias (or both) must be provided!"
    usage
fi

echo "=============================================="
echo "EtherCAT Single Axis Position Control"
echo "=============================================="
echo "Target Position: $TARGET_POSITION µm"
echo "Mode: $MODE"
echo "Velocity: $VELOCITY µm/s"
if [ -n "$BUS_POSITION" ]; then
    echo "Bus Position: $BUS_POSITION"
fi
if [ -n "$BUS_ALIAS" ]; then
    echo "Bus Alias: $BUS_ALIAS"
fi
if [ "$SHUTDOWN_AT_END" = true ]; then
    echo "Shutdown at end: Yes"
else
    echo "Shutdown at end: No"
fi
echo "=============================================="

# Function to build ethercat command parameters based on bus_position and bus_alias
build_ethercat_params() {
    local bus_position=$1
    local bus_alias=$2
    local ethercat_params=""
    
    # Check if neither parameter is defined
    if [ -z "$bus_position" ] && [ -z "$bus_alias" ]; then
        echo "Error: Both bus_position and bus_alias are undefined!" >&2
        return 1
    fi
    
    # Build parameters according to the rules
    if [ -n "$bus_alias" ] && [ -n "$bus_position" ]; then
        # Both defined: use both --alias first, then --position
        ethercat_params="--position $bus_position --alias $bus_alias"
    elif [ -n "$bus_position" ] && [ -z "$bus_alias" ]; then
        # Only bus_position defined
        ethercat_params="--position $bus_position"
    elif [ -z "$bus_position" ] && [ -n "$bus_alias" ]; then
        # Only bus_alias defined
        ethercat_params="--alias $bus_alias"
    fi
    
    echo "$ethercat_params"
    return 0
}

# Function to display position of a slave
display_position() {
    local bus_position=$1
    local bus_alias=$2
    local label=$3
    
    # Build ethercat command parameters using the helper function
    local ethercat_params
    ethercat_params=$(build_ethercat_params "$bus_position" "$bus_alias")
    if [ $? -ne 0 ]; then
        echo "  $label position: Failed to build ethercat parameters"
        return 1
    fi
    
    echo -n "  $label position: "
    if position=$(ethercat upload $ethercat_params --type int32 0x6063 0 2>/dev/null); then
        echo "$position µm"
    else
        echo "Failed to read position"
    fi
}

# Function to control a single slave
config_slave() {
    local bus_position=$1
    local bus_alias=$2
    local target_pos=$3
    local velocity=$4
    
    # Build ethercat command parameters using the helper function
    local ethercat_params
    ethercat_params=$(build_ethercat_params "$bus_position" "$bus_alias")
    if [ $? -ne 0 ]; then
        echo "Error in config_slave: Failed to build ethercat parameters"
        return 1
    fi
    
    # Display configuration info
    if [ -n "$bus_position" ] && [ -n "$bus_alias" ]; then
        echo "Configuring slave at bus position $bus_position with bus alias $bus_alias..."
    elif [ -n "$bus_position" ]; then
        echo "Configuring slave at bus position $bus_position..."
    elif [ -n "$bus_alias" ]; then
        echo "Configuring slave with bus alias $bus_alias..."
    fi
    
    # 1) Put the slave in OP (if your setup/master allows it)
    echo "  Setting slave to OP state..."
    if ! ethercat state $ethercat_params OP; then
        echo "Error: Failed to set slave to OP state"
        exit 1
    fi
    
    # 2) Set mode of operation (1 = profile position mode)
    echo "  Setting profile position mode..."
    if ! ethercat download $ethercat_params --type int8 0x6060 0 1; then
        echo "Error: Failed to set profile position mode"
        exit 1
    fi
    
    # 3) Set the velocity
    echo "  Setting velocity to $velocity µm/s..."
    if ! ethercat download $ethercat_params --type uint32 0x6081 0 $velocity; then
        echo "Error: Failed to set velocity"
        exit 1
    fi
    
    # 4) Set target position
    echo "  Setting target position to $target_pos µm..."
    if ! ethercat download $ethercat_params --type int32 0x607A 0 -- $target_pos; then
        echo "Error: Failed to set target position"
        exit 1
    fi
    
    # 5) CiA-402 enable sequence
    echo "  Executing CiA-402 enable sequence..."
    if ! ethercat download $ethercat_params --type uint16 0x6040 0 0x0006; then   # Shutdown
        echo "Error: Failed to execute shutdown command"
        exit 1
    fi
    sleep 0.1
    if ! ethercat download $ethercat_params --type uint16 0x6040 0 0x0007; then   # Switch On
        echo "Error: Failed to execute switch on command"
        exit 1
    fi
    sleep 0.1
    if ! ethercat download $ethercat_params --type uint16 0x6040 0 0x000F; then   # Enable Operation
        echo "Error: Failed to execute enable operation command"
        exit 1
    fi
    sleep 0.1
}

# Function to shutdown a slave
shutdown_slave() {
    local bus_position=$1
    local bus_alias=$2
    
    # Build ethercat command parameters using the helper function
    local ethercat_params
    ethercat_params=$(build_ethercat_params "$bus_position" "$bus_alias")
    if [ $? -ne 0 ]; then
        echo "Error in shutdown_slave: Failed to build ethercat parameters"
        return 1
    fi
    
    # Display shutdown info
    if [ -n "$bus_position" ] && [ -n "$bus_alias" ]; then
        echo "Shutting down slave at bus position $bus_position with bus alias $bus_alias..."
    elif [ -n "$bus_position" ]; then
        echo "Shutting down slave at bus position $bus_position..."
    elif [ -n "$bus_alias" ]; then
        echo "Shutting down slave with bus alias $bus_alias..."
    fi
    
    # Send shutdown command
    echo "  Sending shutdown command..."
    if ! ethercat download $ethercat_params --type uint16 0x6040 0 0x0006; then   # Shutdown
        echo "Error: Failed to shutdown slave"
        exit 1
    fi
    echo "  Slave shutdown completed"
}

start_absolute_motion_slave() {
    local bus_position=$1
    local bus_alias=$2
    
    # Build ethercat command parameters using the helper function
    local ethercat_params
    ethercat_params=$(build_ethercat_params "$bus_position" "$bus_alias")
    if [ $? -ne 0 ]; then
        echo "Error in start_absolute_motion_slave: Failed to build ethercat parameters"
        return 1
    fi
    
    # Start the motion for absolute move
    if ! ethercat download $ethercat_params --type uint16 0x6040 0 155; then  # Start Motion 
        echo "Error: Failed to start absolute motion"
        return 1
    fi
    # 155 : 0b 1001 1011 
    # bit 7 : 1= Fault reset on / bit 6 : 0= absolute / bit 5: 0= no change set immediately / bit 4 : 1= new setpoint
    # bit 3 : 1= Enable operation / bit 2 : 0= no Quick stop / bit 1: 1= Enable Voltage / bit 0= switch on
    return 0
}

start_relative_motion_slave() {
    local bus_position=$1
    local bus_alias=$2
    
    # Build ethercat command parameters using the helper function
    local ethercat_params
    ethercat_params=$(build_ethercat_params "$bus_position" "$bus_alias")
    if [ $? -ne 0 ]; then
        echo "Error in start_relative_motion_slave: Failed to build ethercat parameters"
        return 1
    fi
    
    # Start the motion for relative move
    if ! ethercat download $ethercat_params --type uint16 0x6040 0 219; then   # Start Motion
        echo "Error: Failed to start relative motion"
        return 1
    fi
    # 219 : 0b 1101 1011
    # bit 7 : 1= Fault reset on / bit 6 : 1= relative / bit 5: 0=no change set immediately / bit 4 : 1= new setpoint
    # bit 3 : 1= Enable operation / bit 2 : 0= no Quick stop / bit 1: 1= Enable Voltage / bit 0= switch on
    return 0
}

# Control the specified slave
echo ""
echo "=============================================="
display_position "$BUS_POSITION" "$BUS_ALIAS" "Initial"
echo "=============================================="

echo ""
echo "Configuring the slave:"
config_slave "$BUS_POSITION" "$BUS_ALIAS" $TARGET_POSITION $VELOCITY

echo ""
echo "=============================================="
display_position "$BUS_POSITION" "$BUS_ALIAS" "Pre-motion"
echo "=============================================="

# Start motion on the slave
echo ""
echo "Starting motion..."
motion_success=true
if [ "$MODE" == "absolute" ]; then
    if ! start_absolute_motion_slave "$BUS_POSITION" "$BUS_ALIAS"; then
        motion_success=false
    fi
else
    if ! start_relative_motion_slave "$BUS_POSITION" "$BUS_ALIAS"; then
        motion_success=false
    fi
fi

echo ""
echo "=============================================="
if [ "$motion_success" = true ]; then
    echo "Motion command sent to slave!"
    display_position "$BUS_POSITION" "$BUS_ALIAS" "Post-motion"
else
    echo "Motion failed!"
    display_position "$BUS_POSITION" "$BUS_ALIAS" "Final"
fi
echo "=============================================="

# Shutdown motor if requested
if [ "$SHUTDOWN_AT_END" = true ]; then
    echo ""
    echo "Shutting down motor as requested..."
    shutdown_slave "$BUS_POSITION" "$BUS_ALIAS"
    echo ""
    echo "=============================================="
    echo "Motor shutdown completed!"
    display_position "$BUS_POSITION" "$BUS_ALIAS" "Final"
    echo "=============================================="
fi

# Exit with error code if motion failed
if [ "$motion_success" = false ]; then
    exit 1
fi
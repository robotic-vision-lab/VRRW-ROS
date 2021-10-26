# gACT: Activation status, echo of the rACT bit (activation bit).
uint8 activated

# gGTO: Action status, echo of the rGTO bit (go to bit).
uint8 action_status

# gSTA: Gripper status, returns the current status & motion of the Gripper fingers.
uint8 gripper_status

# gOBJ: Object detection status, is a built-in feature that provides information on possible object pick-up. Ignore if gGTO == 0.
uint8 object_status 

# gFLT: Fault status returns general error messages that are useful for troubleshooting. 
# Fault LED (red) is present on the Gripper chassis, LED can be blue, red or both and be solid or blinking.
uint8 fault_status

# gPR: Echo of the requested position for the Gripper, value between 0x00 and 0xFF.
uint8 position_request

# gPO: Actual position of the Gripper obtained via the encoders, value between 0x00 and 0xFF.
uint8 current_position

# gCU: The current is read instantaneously from the motor drive, value between 0x00 and 0xFF. 
# Approximate current equivalent is 10 * value read in mA.
uint8 motor_current
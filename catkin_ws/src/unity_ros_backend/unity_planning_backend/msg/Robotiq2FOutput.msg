# rACT: First action to be made prior to any other actions, rACT bit will activate the Gripper. 
# Clear rACT to reset the Gripper and clear fault status.
uint8 activate

# rGTO: The "Go To" action moves the Gripper fingers to the requested position using the configuration defined by the other registers,
# rGTO will engage motion while byte 3, 4 and 5 will determine aimed position, force and speed. The only motions performed without the
# rGTO bit are activation and automatic release routines.
uint8 goto

# rATR: Automatic Release routine action slowly opens the Gripper fingers until all motion axes reach their mechanical limits. After all
# motion is completed, the Gripper sends a fault signal and needs to be reactivated before any other motion is performed. The rATR bit
# overrides all other commands excluding the activation bit (rACT).
uint8 trigger_autorelease

# rPR: This register is used to set the target position for the Gripper's fingers. The positions 0x00 and 0xFF correspond respectively to the fully
# opened and fully closed mechanical stops. For detailed finger trajectory, please refer to the Specifications section.
uint8 position

# rSP: This register is used to set the Gripper closing or opening speed in real time, however, setting a speed will not initiate a motion.
uint8 speed

# rFR: The force setting defines the final gripping force for the Gripper. The force will fix the maximum current sent to the motor while in
# motion. If the current limit is exceeded, the fingers stop and trigger an object detection notification. Please refer to the Robot Input
# Registers & Status section for details on force control.
uint8 force
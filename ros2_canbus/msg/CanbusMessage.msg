# A decoded CAN frame

# Times
builtin_interfaces/Time stamp    # default UNIX in nanoseconds (uint64)
builtin_interfaces/Time can_time # default UNIX in nanoseconds (uint64)

# CAN ID and special flags
# When sending, it follows Linux' format for the can_id field, that is:
# - ID: bits 0-11 for standard frame, or 0-28 for extended frame
# - Error: bit 29
# - RTR bit: bit 30
# - Frame format: bit 30, 0 for standard, 1 for extended
#
# On reception, the drivers should report an error through the API's
# error mechanism on error. They return a RTR with the flag set in the
# ID. and do not report anything specific for an extended frame.
#
# Not all drivers support these flags. The only one that is currently
# guaranteed to is the Socket-CAN driver.
uint32 can_id

# Actual data in the frame
# Note: CAN bus protocol is restricted to 8 Bytes
uint8[8] data

# How many valid bytes there is in data
uint8  size

# Canbus Statistics
builtin_interfaces/Time stamp  # default UNIX in nanoseconds (uint64)
# uint32 will hold without overflow for approx. 450 days with a 100 events per second
# in case the CAN will stay longer on should be changed to uint64
uint32 msg_tx # the number of messages sent (total)
uint32 msg_rx # the number of messages received (total)
uint32 tx # the amount of bytes sent (total)
uint32 rx # the amount of bytes received (total)
uint32 error_count # number of errors since start

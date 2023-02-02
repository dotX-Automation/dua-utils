# Generic command result message with header.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# Header
std_msgs/Header header

# Operation result code
uint8 SUCCESS=0 # Task completed successfully
uint8 FAILED=1  # Task failed
uint8 ERROR=2   # Unrecoverable error
uint8 result

# Error description
string error_msg

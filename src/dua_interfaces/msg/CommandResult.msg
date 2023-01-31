# Generic command result message.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

uint8 SUCCESS=0 # Task completed successfully
uint8 FAILED=1  # Task failed
uint8 ERROR=2   # Unrecoverable error

# Operation result core
uint8 result

# Error description
string error_msg

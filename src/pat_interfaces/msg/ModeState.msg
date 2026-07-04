# Current mode of the PAT terminal, published by mode_manager.

uint8 IDLE=0
uint8 ACQUIRE=1
uint8 HANDOFF=2
uint8 LOCK=3
uint8 COAST=4
uint8 SAFE=5

std_msgs/Header header
uint8 mode

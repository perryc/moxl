# Engine subsystem status
# Published by engine_controller_node at 1 Hz

uint8 OFF = 0
uint8 CRANKING = 1
uint8 RUNNING = 2
uint8 ERROR = 3

uint8 state
float32 rpm             # from tach sensor (0 if no sensor)
float32 cht_celsius     # cylinder head temp (0 if no sensor)
bool choke_engaged
string error_message

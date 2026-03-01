# Mission orchestrator status
# Published by mission_node as action feedback and on /moxl/mission/status

uint8 IDLE = 0
uint8 PREFLIGHT = 1
uint8 TRANSITING = 2
uint8 MOWING = 3
uint8 RETURNING = 4
uint8 PARKED = 5
uint8 ERROR = 6
uint8 PAUSED = 7

uint8 state
uint16 current_strip
uint16 total_strips
float32 progress_pct
string status_text

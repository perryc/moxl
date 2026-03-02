# SDR radio traffic detection event
# Published by sdr_detector_node at 1 Hz

builtin_interfaces/Time stamp
float32 frequency_mhz         # Monitored frequency (e.g. 122.8 CTAF)
float32 signal_power_dbm      # Measured signal power level
bool active                    # true = transmission detected right now
float32 quiet_duration_sec     # Seconds since last transmission ended

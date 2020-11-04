# openvr-osc-receiver
A Python class to receive and save VR headset and controller movement data from Brekel OpenVR recorder, via the Open Sound Control (OSC) protocol. 

This was created for a VR user study, specifically to store trial and experimental condition labels alongside movement data while participants were playing an unmodified VR experience. Work in progress! 


## Features
- Records OSC data for all devices supported by OpenVR Recorder (OSC addresses)
- Allows logging of time-stamped messages (e.g. for user study conditions)
- Auto-detection of Quaternion or Euler angle rotation format

## Command Line Usage

```
usage: openvr_osc_receiver.py [-h] [-f log_file] [-a addr [addr ...]]
                              [-d duration] [-i ip_address] [-p port] [-v]
                              [--precision digits]

optional arguments:
  -h, --help          show this help message and exit
  -f log_file         Log file name (default: openvr_data.csv)
  -a addr [addr ...]  One or more OSC addresses to log (e.g., "-a /HMD
                      /Hand_L"). See OpenVR recorder docs for details on
                      supported addresses
  -d duration         Recording duration in seconds (default: until CTRL+C)
  -i ip_address       OSC server IP address to bind to (default: 127.0.0.1)
  -p port             OSC server UPD port to use (default: 7775)
  -v                  Verbose output: Print received OSC data to console
  --precision digits  Decimal precision of output file (default: 10)
```

## Log File Columns
- device: OSC device address, or *LogMessage* for messages
- message: Custom log message
- deviceid: ID value reported by OpenVR Recorder
- time_ovr; time_sys: OSC and system time stamps
- rtime_ovr; rtime_sys: OSC and system time stamps relative to first OSC packet
- posX, posY, posZ: Device position
- rotX, rotY, rotZ, rotW: Device orientation (rotW=NaN if data is in Euler format)
- button1..14: Controller and ViveTracker button state
- thumb0_posX ... - Hand model position and rotation data (see OpenVR recorder documentation for details)

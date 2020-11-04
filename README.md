# openvr-osc-receiver
A Python class to receive and save data from Brekel OpenVR recorder via the Open Sound Control (OSC) protocol. 

This was created for a VR user study, specifically to store trial and experimental condition labels alongside movement data while participants were playing an unmodified VR experience. Work in progress! 


## Features
- Records OSC data for all devices supported by OpenVR Recorder (OSC addresses)
- Allows logging of time-stamped messages (e.g. for user study conditions)
- Auto-detection of Quaternion or Euler angle rotation format


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

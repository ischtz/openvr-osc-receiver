# Example script for the OpenVRReceiver class
#
# This file demonstrates how to use an OpenVRReceiver instance.
# For command line use, you can call the module directly:
# python openvr_osc_receiver.py -h

import time

from openvr_osc_receiver import OpenVRReceiver

ovr = OpenVRReceiver(log_file='example_output.csv', auto_record=False, status=True)
ovr.start_recording()

print('Waiting for OSC data...')
ovr.wait_for_data()

# Record data for 5 seconds
for t in range(1, 6):
    time.sleep(1.0)
    ovr.log_message('Test {:d}'.format(t))

ovr.stop_recording()
ovr.close()

print('Example script finished.')

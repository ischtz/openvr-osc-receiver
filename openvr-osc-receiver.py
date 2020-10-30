# -*- coding: utf-8 -*-
"""
OpenVRReceiver - class to receive OSC stream data from Brekel OpenVR recorder
and save time-stamped motion data and log strings to file
"""

import sys
import time
import math
import argparse

from multiprocessing import Queue
from threading import Thread, Lock
from pythonosc import dispatcher, osc_server


class OpenVRReceiver:
    """ Class to receive and save OSC stream data from Brekel OpenVR recorder 
    
    Args:
        log_file (str): Name of samples log file
        log_sep (str): Separator string to use (default: Tab)
        log_devices (list): List of device addresses to record and log,
            e.g. ['/HMD', 'Controller'] (see Brekel OpenVR Recorder documentation)
        log_precision (int): Decimal precision of recorded data
        udp_ip (string): IP address to listen on (default: 127.0.0.1)
        udp_port (int): UPD port to listen on for OSC data
        auto_record (bool): if True, start recording as soon as data is received
        debug (bool): if True, print received data to stdout
        status (bool): if True, print status messages to stdout (for command line use)
    """

    def __init__(self, log_file='openvr_data.txt', log_sep='\t', log_devices=['/HMD', '/Controller', '/Hand_L', '/Hand_R'],
                 log_precision=10, udp_ip='127.0.0.1', udp_port=7775, auto_record=True, debug=False, status=False):

        self.udp_port = udp_port
        self.udp_ip = udp_ip
        self.log_file = log_file
        self.log_sep = log_sep
        self.log_devices = log_devices
        self.log_precision = log_precision
        self.debug = debug
        self.status = status
        if self.debug:
            self.status = True

        self.samples_received = False
        self._first_sample_timestamp = -1.0
        self._first_sample_time = -1.0
        self._latest_sample_timestamp = -1.0

        # Set up and start recorder thread
        self._recording = False
        self._log_active = True
        self._threadlock = Lock()
        self._rec_queue = Queue()
        self._recorder = Thread(target=self._log_sample, args=[self._rec_queue], name='recorder')
        self._recorder.daemon = True
        self._recorder.start()

        # Set up log file formatting
        self.LOG_HEADER = ['device', 'message', 'deviceid', 'time_ovr', 'time_sys', 'rtime_ovr', 'rtime_sys', 
                           'posX', 'posY', 'posZ', 'rotX', 'rotY', 'rotZ', 'rotW']
        self.LOG_FORMAT = ['{:s}', '{:s}', '{:d}'] + 11 * ['{{:.{prec}f}}'.format(prec=self.log_precision),]
        
        if '/Controller' in self.log_devices or '/GenericTracker' in self.log_devices:
            # Devices with button and axis states (+24 fields)
            self.LOG_HEADER += ['button1', 'button2', 'button3', 'button4', 'button5', 'button6', 'button7',
                                'button8', 'button9', 'button10', 'button11', 'button12', 'button13', 'button14',
                                'axis1X', 'axis1Y',	'axis2X', 'axis2Y',	'axis3X', 'axis3Y',	
                                'axis4X', 'axis4Y',	'axis5X', 'axis5Y']
            self.LOG_FORMAT += 24 * ['{{:.{prec}f}}'.format(prec=self.log_precision),]

        if '/Hand_L' in self.log_devices or '/Hand_R' in self.log_devices:
            # Hand and finger tracking data (+168 fields)
            self.LOG_HEADER += ['thumb0_posX', 'thumb0_posY', 'thumb0_posZ', 'thumb0_rotX', 'thumb0_rotY', 'thumb0_rotZ', 'thumb0_rotW',
                                'thumb1_posX', 'thumb1_posY', 'thumb1_posZ', 'thumb1_rotX', 'thumb1_rotY', 'thumb1_rotZ', 'thumb1_rotW',
                                'thumb2_posX', 'thumb2_posY', 'thumb2_posZ', 'thumb2_rotX', 'thumb2_rotY', 'thumb2_rotZ', 'thumb2_rotW',
                                'thumb3_posX', 'thumb3_posY', 'thumb3_posZ', 'thumb3_rotX', 'thumb3_rotY', 'thumb3_rotZ', 'thumb3_rotW',                              
                                'index0_posX', 'index0_posY', 'index0_posZ', 'index0_rotX', 'index0_rotY', 'index0_rotZ', 'index0_rotW',
                                'index1_posX', 'index1_posY', 'index1_posZ', 'index1_rotX', 'index1_rotY', 'index1_rotZ', 'index1_rotW',
                                'index2_posX', 'index2_posY', 'index2_posZ', 'index2_rotX', 'index2_rotY', 'index2_rotZ', 'index2_rotW',
                                'index3_posX', 'index3_posY', 'index3_posZ', 'index3_rotX', 'index3_rotY', 'index3_rotZ', 'index3_rotW',
                                'index4_posX', 'index4_posY', 'index4_posZ', 'index4_rotX', 'index4_rotY', 'index4_rotZ', 'index4_rotW',
                                'middle0_posX', 'middle0_posY', 'middle0_posZ', 'middle0_rotX', 'middle0_rotY', 'middle0_rotZ', 'middle0_rotW',
                                'middle1_posX', 'middle1_posY', 'middle1_posZ', 'middle1_rotX', 'middle1_rotY', 'middle1_rotZ', 'middle1_rotW',
                                'middle2_posX', 'middle2_posY', 'middle2_posZ', 'middle2_rotX', 'middle2_rotY', 'middle2_rotZ', 'middle2_rotW',
                                'middle3_posX', 'middle3_posY', 'middle3_posZ', 'middle3_rotX', 'middle3_rotY', 'middle3_rotZ', 'middle3_rotW',
                                'middle4_posX', 'middle4_posY', 'middle4_posZ', 'middle4_rotX', 'middle4_rotY', 'middle4_rotZ', 'middle4_rotW',
                                'ring0_posX', 'ring0_posY', 'ring0_posZ', 'ring0_rotX', 'ring0_rotY', 'ring0_rotZ', 'ring0_rotW',
                                'ring1_posX', 'ring1_posY', 'ring1_posZ', 'ring1_rotX', 'ring1_rotY', 'ring1_rotZ', 'ring1_rotW',
                                'ring2_posX', 'ring2_posY', 'ring2_posZ', 'ring2_rotX', 'ring2_rotY', 'ring2_rotZ', 'ring2_rotW',
                                'ring3_posX', 'ring3_posY', 'ring3_posZ', 'ring3_rotX', 'ring3_rotY', 'ring3_rotZ', 'ring3_rotW',
                                'ring4_posX', 'ring4_posY', 'ring4_posZ', 'ring4_rotX', 'ring4_rotY', 'ring4_rotZ', 'ring4_rotW',
                                'pinky0_posX', 'pinky0_posY', 'pinky0_posZ', 'pinky0_rotX', 'pinky0_rotY', 'pinky0_rotZ', 'pinky0_rotW',
                                'pinky1_posX', 'pinky1_posY', 'pinky1_posZ', 'pinky1_rotX', 'pinky1_rotY', 'pinky1_rotZ', 'pinky1_rotW',
                                'pinky2_posX', 'pinky2_posY', 'pinky2_posZ', 'pinky2_rotX', 'pinky2_rotY', 'pinky2_rotZ', 'pinky2_rotW',
                                'pinky3_posX', 'pinky3_posY', 'pinky3_posZ', 'pinky3_rotX', 'pinky3_rotY', 'pinky3_rotZ', 'pinky3_rotW',
                                'pinky4_posX', 'pinky4_posY', 'pinky4_posZ', 'pinky4_rotX', 'pinky4_rotY', 'pinky4_rotZ', 'pinky4_rotW']
            self.LOG_FORMAT += 168 * ['{{:.{prec}f}}'.format(prec=self.log_precision),]

        # Field onsets when using Euler angles
        self._hand_euler_ix = zip(list(range(37, 205, 7)), list(range(7, 151, 6)))

        # Open log file and write header
        self._log = open(self.log_file, 'a')
        self._log_header()
        self._pr('Recording to log file: {:s}'.format(self.log_file))

        # Set up OSC server in its own thread
        self._dispatcher = dispatcher.Dispatcher()
        for addr in self.log_devices:
            self._dispatcher.map(addr, self._osc_msg_handler)
        self._pr('Listening for addresses: {:s}.'.format(str(self.log_devices)))
        self._oscserver = None
        self._oscthread = Thread(target=self._osc_server_thread, name='oscserver')
        self._oscthread.daemon = True
        self._oscthread.start()

        # Start listening for packets
        if auto_record:
            self.start_recording()


    def _pr(self, msg):
        """ Print a time-stamped message to the console """
        if self.status:
            print('[{:s}] '.format(time.strftime('%H:%m:%S')) + msg)


    def _osc_server_thread(self):
        """ Thread for OSC blocking server """
        self._oscserver = osc_server.BlockingOSCUDPServer((self.udp_ip, self.udp_port), self._dispatcher)
        self._pr('Starting OSC server on {:s}:{:d}...'.format(self.udp_ip, self.udp_port))
        self._oscserver.serve_forever()
        self._pr('OSC server shut down.')


    def _osc_msg_handler(self, address, *osc_args):
        """ OSC Message handling callback """
        recv_time = time.perf_counter()
        
        # s: 0:6 - metadata, 7:13 - transform, 14:37 - controller, 38:206 - skeleton
        s = ['', '', -1] + [math.nan,] * (len(self.LOG_FORMAT)-3)

        # Store time stamps of first received packet
        if not self.samples_received:
            self._first_sample_timestamp = osc_args[1]
            self._first_sample_time = recv_time
            self.samples_received = True

        if self._recording:
            if address in ['/HMD', '/TrackingReference', '/DisplayRedirect']:
                self._latest_sample_timestamp = osc_args[1]
                s[0:7] = [address.strip('/'), 
                          '', 
                          osc_args[0],
                          osc_args[1] * 1000.0,
                          recv_time * 1000.0,
                          (osc_args[1] - self._first_sample_timestamp) * 1000.0, 
                          (recv_time - self._first_sample_time) * 1000]

                if len(osc_args) == 8:
                    # Rotation data is in Euler angles
                    s[7:13] = osc_args[2:8]
                elif len(osc_args) == 9:
                    # Rotation data is in Quaternions
                    s[7:14] = osc_args[2:9]

            if address in ['/Controller', '/GenericTracker']:
                self._latest_sample_timestamp = osc_args[1]
                s[0:7] = [address.strip('/'), 
                          '',
                          osc_args[0],
                          osc_args[1] * 1000.0,
                          recv_time * 1000.0,
                          (osc_args[1] - self._first_sample_timestamp) * 1000.0, 
                          (recv_time - self._first_sample_time) * 1000]

                if len(osc_args) == 32:
                    s[7:13] = osc_args[2:8] # Euler
                    s[14:38] = osc_args[8:33]
                elif len(osc_args) == 33:
                    s[7:14] = osc_args[2:9] # Quat
                    s[14:38] = osc_args[9:34]

            if address in ['/Hand_L', '/Hand_R']:
                self._latest_sample_timestamp = osc_args[0]
                s[0:7] = [address.strip('/'), 
                          '',
                          -1, # Hands have no ID field in OSC data
                          osc_args[0] * 1000.0,
                          recv_time * 1000.0,
                          (osc_args[0] - self._first_sample_timestamp) * 1000.0, 
                          (recv_time - self._first_sample_time) * 1000]

                if len(osc_args) == 151:
                    s[7:13] = osc_args[1:7]
                    self._hand_euler_ix = zip(list(range(38, 206, 7)), list(range(7, 151, 6)))
                    # Euler: Skip over rotW fields in output
                    for ix in self._hand_euler_ix:
                        s[ix[0]:ix[0]+6] = osc_args[ix[1]:ix[1]+6]
                elif len(osc_args) == 176: # Quat
                    s[7:14] = osc_args[1:8]
                    s[38:206] = osc_args[8:177]
                
            # Add to recording queue
            with self._threadlock:
                self._rec_queue.put(s)

        if self.debug:
            fmt = '{:.2f}\t{:19s}  X,Y,Z: {: .3f} {: .3f} {: .3f}  rX,rY,rZ,rW: {: 0.3f} {: 0.3f} {: 0.3f} {: 0.3f}'
            self._pr(fmt.format(s[6], s[0], *s[7:14]))


    def _log_header(self):
        """ Write column header to the log file """
        if self._log:
            self._log.write(self.log_sep.join(self.LOG_HEADER) + '\n')


    def _log_sample(self, queue):
        """ Thread to write available sample to log file """
        while self._log_active:
            sample = None

            with self._threadlock:
                if not queue.empty():
                    sample = queue.get()
            
            if sample:
                self._log.write(self.log_sep.join(self.LOG_FORMAT).format(*sample) + '\n')
                self._log.flush()

            time.sleep(0.000001)


    def log_message(self, message, device='LogMessage'):
        """ Write time-stamped message string to log file 
        
        Args:
            device (str): Value to store in "device" field
        """
        recv_time = time.perf_counter()
        if self.samples_received:
            latest_timestamp = self._latest_sample_timestamp * 1000.0
            latest_time_rel = (self._latest_sample_timestamp - self._first_sample_timestamp) * 1000.0
            recv_time_rel = (recv_time - self._first_sample_time) * 1000
        else:
            # might log messages before having seen a sample
            latest_timestamp = math.nan
            latest_time_rel = math.nan
            recv_time_rel = math.nan

        s = [device,
             '"{:s}"'.format(message),
             -1,
             latest_timestamp,
             recv_time * 1000.0,
             latest_time_rel,
             recv_time_rel] + [math.nan,] * (len(self.LOG_FORMAT)-7)

        with self._threadlock:
            self._rec_queue.put(s)

        # Also print to console if status messages are enabled
        self._pr('LogMessage: {:s}'.format(message))


    def start_recording(self):
        """ Start recording received samples to log file """
        self._recording = True
        self._pr('Sample recording started.')


    def stop_recording(self):
        """ Stop sample recording (does not close file) """
        self._recording = False
        self._pr('Sample recording stopped.')


    def wait_for_data(self, timeout=30):
        """ Wait until the first received OSC sample 
        
        Args:
            timeout (int): Timeout in seconds
        """
        t_end = time.perf_counter() + timeout
        while time.perf_counter() < t_end:
            if self.samples_received:
                return True
            time.sleep(0.001)
        self._pr('Timeout while waiting for first OSC sample.')


    def close(self):
        """ Shut down threads and close log file """
        if self._recording:
            self.stop_recording()
        self._oscserver.shutdown()

        # Wait until recording queue is empty
        if self._rec_queue.qsize() > 100:
            self._pr('Saving remaining samples to log file...')
        while not self._rec_queue.empty():
            pass
        self._log_active = False # stop log thread
        self._log.close()
        self._pr('Log file closed.')


if __name__ == '__main__':

    # Create command line arguments for standalone operation
    ap = argparse.ArgumentParser()
    ap.add_argument('-f', dest='log_file', metavar='log_file', default='openvr_data.csv', type=str,
                    help='Log file name (default: openvr_data.csv)')
    a_help = 'One or more OSC addresses to log (e.g., "-a /HMD /Hand_L"). '
    a_help += 'See OpenVR recorder docs for details on supported addresses'
    ap.add_argument('-a', dest='address', metavar='addr', nargs='+',
                    help=a_help)
    ap.add_argument('-d', dest='duration', metavar='duration', default=None, type=int,
                    help='Recording duration in seconds (default: until CTRL+C)')
    ap.add_argument('-i', dest='udp_ip', metavar='ip_address', default='127.0.0.1', type=str,
                    help='OSC server IP address to bind to (default: 127.0.0.1)')
    ap.add_argument('-p', dest='udp_port', metavar='port', default=7775, type=int,
                    help='OSC server UPD port to use (default: 7775)')
    ap.add_argument('-v', dest='verbose', action='store_true',
                    help='Verbose output: Print received OSC data to console')
    ap.add_argument('--precision', metavar='digits', dest='precision', action='store', default=10,
                    type=int, help='Decimal precision of output file (default: 10)')
    args = ap.parse_args()

    # Verify that all addresses are known
    ADDRS = ['/HMD', '/TrackingReference', '/DisplayRedirect',
             '/Controller', '/GenericTracker','/Hand_L', '/Hand_R']
    if args.address is not None:
        for a in args.address:
            if a not in ADDRS:
                print('Error: Unsupported address type specified: {:s}. Forgot a "/"?'.format(a))
                sys.exit(-1)
        log_devices = args.address
    else:
        log_devices = ['/HMD', '/Controller', '/Hand_L', '/Hand_R']

    # Initialize receiver instance and start recording
    ovr = OpenVRReceiver(log_file=args.log_file,
                         log_precision=args.precision,
                         log_devices=log_devices,
                         udp_ip=args.udp_ip,
                         udp_port=args.udp_port,
                         debug=args.verbose,
                         auto_record=True,
                         status=True)
    
    if args.duration is not None:
        time.sleep(args.duration)
    else:
        try:
            while True:
                time.sleep(0.00001)
        except KeyboardInterrupt:
            ovr._pr('Received KeyboardInterrupt, exiting.')
    
    ovr.close()

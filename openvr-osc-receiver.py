# -*- coding: utf-8 -*-
"""
OpenVRReceiver - class to receive OSC stream data from Brekel OpenVR recorder
and save time-stamped motion data and log strings to file
"""

import time
import math

from multiprocessing import Queue
from threading import Thread, Lock
from pythonosc import dispatcher, osc_server


class OpenVRReceiver:
    """ Class to receive and save OSC stream data from Brekel OpenVR recorder """

    def __init__(self, log_file='openvr_data.txt', log_sep='\t', udp_ip='127.0.0.1', udp_port=7775, auto_record=True, debug=False):

        self.udp_port = udp_port
        self.udp_ip = udp_ip
        self.log_file = log_file
        self.log_sep = log_sep
        self.debug = debug

        self.samples_received = False
        self._first_sample_timestamp = None
        self._first_sample_time = None
        
        # Set up and start recorder thread
        self._recording = False
        self._log_active = True
        self._threadlock = Lock()
        self._rec_queue = Queue()
        self._recorder = Thread(target=self._log_sample, args=[self._rec_queue], name='recorder')
        self._recorder.daemon = True
        self._recorder.start()

        # Set up log file and write file header
        self.LOG_HEADER = ['address', 'deviceid', 'time_ovr', 'time_sys', 'rtime_ovr', 'rtime_sys', 
                           'posX', 'posY', 'posZ', 'rotX', 'rotY', 'rotZ', 'rotW']
        self.LOG_FORMAT = self.log_sep.join(['{:s}', '{:d}\t{:.2f}\t{:.5f}\t{:.2f}'] + 8 * ['{:.5f}',])
        self.LOG_FORMAT += '\n'
        self._log = open(self.log_file, 'a')
        self._log_header()

        # Set up OSC server in its own thread
        ADDR_TO_MAP = ['/HMD']#, '/Controller', '/Hand_L', '/Hand_R']
        self._dispatcher = dispatcher.Dispatcher()
        for addr in ADDR_TO_MAP:
            self._dispatcher.map(addr, self._osc_msg_handler)
        self._pr('Listening for addresses: {:s}.'.format(str(ADDR_TO_MAP)))
        self._oscserver = None
        self._oscthread = Thread(target=self._osc_server_thread, name='oscserver')
        self._oscthread.daemon = True
        self._oscthread.start()

        # Start listening for packets
        if auto_record:
            self.start_recording()


    def _pr(self, msg):
        """ Print a time-stamped message """
        print('[{:s}] '.format(time.strftime('%H:%m:%S')) + msg)


    def _osc_server_thread(self):
        """ OSC server running in separate thread """
        self._oscserver = osc_server.ThreadingOSCUDPServer((self.udp_ip, self.udp_port), self._dispatcher)
        self._pr('Starting OSC server on {:s}:{:d}...'.format(self.udp_ip, self.udp_port))
        self._oscserver.serve_forever()
        self._pr('OSC server shut down.')


    def _osc_msg_handler(self, address, *osc_args):
        """ OSC Message handling callback """
        recv_time = time.time()
        
        # Store time stamps of first received packet
        if not self.samples_received:
            self._first_sample_timestamp = osc_args[1]
            self._first_sample_time = recv_time
            self.samples_received = True

        if self._recording:
            s = [address.strip('/'),
                 osc_args[0], 
                 osc_args[1] * 1000.0, 
                 recv_time, 
                 (osc_args[1] - self._first_sample_timestamp) * 1000.0, 
                 (recv_time - self._first_sample_time) * 1000]
            
            if len(osc_args) < 9:
                # Rotation data is in Euler angles
                s.extend(osc_args[2:])
                s.append(math.nan) # fill rotW field
            else:
                # Rotation data is in Quaternions
                s.extend(osc_args[2:])

            # Add to recording queue
            with self._threadlock:
                self._rec_queue.put(s)

        if self.debug:
            print("[{:s}]\t{:d} {:.5f} {:.2f} {:.5f}".format(address, osc_args[0], osc_args[1] * 1000.0, (osc_args[1]-self._first_sample_timestamp) * 1000.0, recv_time))

        # len(args) == 8 -> rotation in euler, 9 -> quaternions
        #print(len(osc_args))


    def _log_header(self):
        """ Write the header to log file """
        if self._log:
            self._log.write(self.log_sep.join(self.LOG_HEADER) + '\n')


    def _log_sample(self, queue):
        """ Thread function: Write available sample to log file """
        while self._log_active:
            sample = None

            with self._threadlock:
                if not queue.empty():
                    sample = queue.get()
            
            if sample:
                self._log.write(self.LOG_FORMAT.format(*sample))
                self._log.flush()

            time.sleep(0.0001)


    def start_recording(self):
        """ Start recording received samples to log file """
        self._recording = True
        self._pr('Sample recording started.')


    def stop_recording(self):
        """ Stop sample recording, but do not close log file """
        self._recording = False
        self._pr('Sample recording stopped.')


    def close(self):
        """ Shut down and close log file """
        self.stop_recording()
        self._oscserver.shutdown()

        # Wait until recording queue is empty
        while not self._rec_queue.empty():
            time.sleep(0.001)
        self._log_active = False # stop log thread
        self._log.close()
        self._pr('Log file closed.')


if __name__ == '__main__':
    ovr = OpenVRReceiver(log_file='openvr_testing.csv')
    time.sleep(10)
    ovr.close()

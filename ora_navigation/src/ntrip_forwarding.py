#! /usr/bin/env python3
import base64
import socket
import errno
import time
import serial
import threading
import sys
import select
import os
import queue as Queue

# NTRIP parameters
server_ip = '148.149.0.87'
server_port = 10001
#mountpoint = 'NETWORK_SOLUTION_RTCM3-GG'
mountpoint = 'METR_RTCM3-GG'
username = 'dmocnik'
password = os.getenv('MDOT_PW')

# Serial port relay parameters
serial_device = '/dev/ttyACM0'
baud_rate = 155200


# Testing
use_serial = True
dummy_gga = '$GPGGA,000000,4243.425,N,08320.251,W,1,4,1.5,00.0,M,0,M,,*73'

# Global variables
serial_port = serial.Serial()
ntrip_tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
rtcm_queue = Queue.Queue()
gga_queue = Queue.Queue()
needs_reset = False


def init_serial():
    global serial_port
    # Wait for serial port to be ready and keep trying to connect until it is
    print(f'Connecting to serial device [{serial_device}] at {baud_rate}')
    while True:
        try:
            serial_port = serial.Serial(serial_device, baud_rate, rtscts=True, dsrdtr=True)
            print('Connected!')
            break
        except (serial.SerialException, OSError) as ex:
            print(ex)

        # Wait before trying again and stop program on Ctrl-C
        try:
            time.sleep(2)
        except KeyboardInterrupt:
            sys.exit(0)


def connect_to_ntrip_server():
    global ntrip_tcp_sock
    print('Connecting to NTRIP server...')
    while True:
        try:
            ntrip_tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ntrip_tcp_sock.connect((server_ip, server_port))
            break
        except Exception as ex:
            print(f'Error connecting socket: {ex}')

        try:
            time.sleep(2)
        except KeyboardInterrupt:
            sys.exit(0)

    encoded_credentials = (username + ':' + password).encode('utf-8')
    base64_credentials = base64.b64encode(encoded_credentials).decode('utf-8')
    server_request = 'GET /%s HTTP/1.0\r\nUser-Agent: NTRIP ABC/1.2.3\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic %s\r\n\r\n' % (
        mountpoint, base64_credentials)
    ntrip_tcp_sock.send(server_request.encode('utf-8'))

    while True:
        try:
            response = ntrip_tcp_sock.recv(10000)
        except socket.error as e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                continue
            else:
                # a "real" error occurred
                print(e)
        else:
            if 'ICY 200 OK'.encode() in response:
                print('Success')
                return True
            else:
                print(f'Received unexpected response from server:\n{response}')
                return False

        # After a "real" error, wait a while before trying to get a response again
        time.sleep(2)


class NtripSocketThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()
        self.no_rtcm_data_count = 0
        self.sent_gga = False

    def run(self):
        global needs_reset
        print('Starting TCP socket thread')
        while not self.stop_event.is_set():
            # Receive RTCM messages from NTRIP server and put in queue for
            # serial port thread to send to GPS receiver
            try:
                ready_to_read, ready_to_write, in_error = select.select([ntrip_tcp_sock, ], [ntrip_tcp_sock, ], [], 5)
            except select.error:
                needs_reset = True
                continue

            if len(ready_to_read) > 0:
                rtcm_msg = ntrip_tcp_sock.recv(100000)
                if len(rtcm_msg) > 0:
                    print(f'Received RTCM message of length {len(rtcm_msg)}')
                    rtcm_queue.put(rtcm_msg)
                    self.no_rtcm_data_count = 0

            # Get GPGGA messages from serial receive queue and send
            # to NTRIP server to keep connection alive
            if len(ready_to_write) > 0:
                try:
                    gga_msg = gga_queue.get_nowait()
                    print(f'Sending new GGA message to NTRIP server {gga_msg}')
                    ntrip_tcp_sock.send(gga_msg.encode())
                    self.sent_gga = True
                except Queue.Empty:
                    pass

            if self.no_rtcm_data_count > 200:
                print('No RTCM messages for 10 seconds; resetting')
                needs_reset = True
                self.no_rtcm_data_count = 0

            if self.sent_gga:
                self.no_rtcm_data_count += 1

            time.sleep(0.05)

        print('Stopping TCP socket thread')
        ntrip_tcp_sock.close()

    def stop(self):
        self.stop_event.set()


class SerialThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.stale_gga_count = 0
        self.stop_event = threading.Event()

    def run(self):
        global needs_reset
        print('Starting serial port thread')
        while not self.stop_event.is_set():
            # Get RTCM messages from NTRIP TCP socket queue and send
            # to GPS receiver to enable RTK tracking
            try:
                rtcm_msg = rtcm_queue.get_nowait()

                if use_serial:
                    try:
                        print('sending')
                        serial_port.write(rtcm_msg)
                    except serial.SerialException as ex:
                        print(ex)
                        needs_reset = True
                        time.sleep(1)
                else:
                    print(f'Would send data to GPS now ({len(rtcm_msg)} bytes)')

            except Queue.Empty:
                # Nothing in the RTCM message queue this time
                pass
            except serial.portNotOpenError as ex:
                print(f'Error writing serial data: {ex}')
            except TypeError as ex:
                print(f'Error writing serial data: {ex}')

            # Read GPGGA messages from GPS receiver and put in queue for
            # TCP socket thread to send to NTRIP server to keep its connection alive
            if self.stale_gga_count > 400:
                print('WARNING -- More than 20 seconds since last received GPGGA message from receiver')
                self.stale_gga_count = 0

            if use_serial:
                try:
                    if serial_port.inWaiting() > 0:
                        nmea_str = serial_port.readline()
                        if 'GPGGA' in nmea_str.decode():
                            gga_queue.put(nmea_str)
                            self.stale_gga_count = 0
                except IOError as ex:
                    print(f'Serial port error: {ex}')
                    needs_reset = True
                    time.sleep(1)

            if self.stale_gga_count > 300:
                self.stale_gga_count = 0
                gga_queue.put(dummy_gga)

            self.stale_gga_count += 1
            time.sleep(0.05)

        print('Stopping serial port thread')
        serial_port.close()

    def stop(self):
        self.stop_event.set()


def stop_threads(workers):
    for worker in workers:
        worker.stop()
        worker.join()


def start_threads():
    workers = [NtripSocketThread(), SerialThread()]
    for worker in workers:
        worker.start()
    return workers


if __name__ == '__main__':

    STARTUP = 0
    RUNNING = 1
    RESETTING = 2
    SHUTDOWN = 3

    state = STARTUP
    workers = []

    while True:
        try:
            if state == STARTUP:
                if use_serial:
                    init_serial()

                connect_to_ntrip_server()
                workers = start_threads()
                state = RUNNING
            elif state == RUNNING:
                if needs_reset:
                    state = RESETTING
                    needs_reset = False
            elif state == RESETTING:
                stop_threads(workers)
                state = STARTUP
            elif state == SHUTDOWN:
                print('Shutting down')
                stop_threads(workers)
                break

            time.sleep(0.05)
        except (KeyboardInterrupt, SystemExit):
            state = SHUTDOWN
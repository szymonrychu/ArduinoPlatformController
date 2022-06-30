#!/usr/bin/env python3
import serial
import traceback
from threading import Thread
import time
import zmq
import pty
from threading import Thread
import os

import logging
from os import environ
_env2log_name = 'REMOTE_SERIAL_LOG_LEVEL'
_env2log_default = 'INFO'
_env2log = {
    'DEBUG': logging.DEBUG,
    'INFO':  logging.INFO,
    'WARN':  logging.WARNING,
    'ERROR': logging.ERROR,
    'FATAL': logging.FATAL
}
_envlog = environ.get(_env2log_name, _env2log_default)
if not _envlog in _env2log.keys():
    _envlog = _env2log_default
logging.basicConfig(format='%(asctime)s [%(levelname)s] %(message)s', level=_env2log[_envlog])
        
class SerialWrapper():

    def __init__(self, fpath, baudrate=115200):
        self._fpath = fpath
        self._baudrate = baudrate
        self._serial = None
        self.repair_serial()

    def data_available(self):
        return self._serial.inWaiting()

    def read_data(self):
        raw_data = None
        try:
            if self.data_available():
                raw_line = self._serial.readline()
                if raw_line is not None:
                    raw_data = raw_line.decode('ascii')
                    if raw_data != '':
                        if raw_data[-1] == '\n':
                            raw_data = raw_data[:-1]
        except serial.SerialTimeoutException:
            pass
        except UnicodeDecodeError:
            logging.warn(f"cannot parse '{raw_data}'")
            self.repair_serial()
        return raw_data

    def write_data(self, raw_data):
        try:
            self._serial.write(raw_data.encode())
            return True
        except TypeError:
            tb = traceback.format_exc()
            logging.warn(str(tb))
            self._repair_serial()

    def repair_serial(self):
        if self._serial != None:
            try:
                self._serial.close()
                self._serial = None
            except Exception:
                tb = traceback.format_exc()
                logging.warn(str(tb))
        while self._serial == None:
            try:
                self._serial = serial.Serial(self._fpath, self._baudrate, timeout=0.1)
            except Exception:
                tb = traceback.format_exc()
                logging.warn(str(tb))
                time.sleep(1)

class ZeroMQThread(Thread):
    def __init__(self, zeromq_socket_type, zeromq_socket_addr, incoming_message_handler=None, bind=True):
        Thread.__init__(self, target=self.receive_ZMQ)
        self._socket = zmq.Context().socket(zeromq_socket_type)
        if bind:
            self._socket.bind(zeromq_socket_addr)
            logging.debug(f"Binded to {zeromq_socket_addr}")
        else:
            logging.debug(f"Connected to {zeromq_socket_addr}")
            self._socket.connect(zeromq_socket_addr)
        self._process_messages = False
        self.__message_processor = incoming_message_handler or self.__default_message_processor

    def start(self):
        self._process_messages = True
        Thread.start(self)

    def join(self, timeout=0.0):
        self._process_messages = True
        Thread.join(timeout)

    def __default_message_processor(self, msg):
        logging.debug(f"incoming_message = '{msg}'")

    def receive_ZMQ(self):
        while self._process_messages:
            raw_msg = self._socket.recv()
            msg = raw_msg.decode('ascii')
            while msg[-1] == '\n':
                msg = msg[:-1]
            self.__message_processor(msg)

    def _zmq_transmit(self, raw_data):
        self._socket.send(raw_data.encode())

    def transmit(self, msg):
        self._zmq_transmit(msg)

class ZeroMQPAIR(ZeroMQThread):

    def __init__(self, zeromq_server_host='*', zeromq_port=8888, incoming_message_handler=None, bind=True):
        ZeroMQThread.__init__(self, zmq.PAIR, f"tcp://{zeromq_server_host}:{zeromq_port}", incoming_message_handler, bind)

class ZeroMQSerialServer(ZeroMQPAIR, SerialWrapper):

    def __init__(self, fpath, baudrate=115200, zeromq_server_host='*', zeromq_port=8888):
        ZeroMQPAIR.__init__(self, zeromq_server_host, zeromq_port, self._inc_message_handler)
        SerialWrapper.__init__(self, fpath, baudrate)

    def start(self):
        ZeroMQPAIR.start(self)
        while self._process_messages:
            msg = self.read_data()
            if msg:
                logging.info(f"Serial -> ZMQ '{msg}'")
                self.transmit(msg)

    def _inc_message_handler(self, message):
        logging.info(f"ZMQ <- Serial '{message}'")
        self.write_data(message)


class ZeroMQSerialClient(ZeroMQPAIR, SerialWrapper):

    def __init__(self, zeromq_server_host='localhost', zeromq_port=8888):
        ZeroMQPAIR.__init__(self, zeromq_server_host, zeromq_port, self._inc_message_handler, bind=False)
        self._master, self._slave = pty.openpty()
        self._slave_fd = os.ttyname(self._slave)
        SerialWrapper.__init__(self, self._slave_fd)

    def get_tty_path(self):
        return str(self._slave_fd)

    def start(self):
        ZeroMQPAIR.start(self)
        while self._process_messages:
            res = b""

            while self._process_messages and not res.endswith(b'\n'):
                res += os.read(self._master, 1)

            msg = res.decode('ascii')
            logging.info(f"Serial -> ZMQ '{msg}'")
            self.transmit(msg)

    def _inc_message_handler(self, msg):
        logging.info(f"Serial <- ZMQ '{msg}'")
        os.write(self._master, f"{msg}\n".encode())

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='ZeroMQ Serial Proxy')
    parser.add_argument('-H', '--server-host', type=str, default='*', help='hostname/ip of the server to connect to')
    parser.add_argument('-p', '--server-port', type=int, default=8888, help='port of the server to connect to')

    sub_parsers = parser.add_subparsers(help='sub-command help', dest='type')

    client_parser = sub_parsers.add_parser('client', help='Client subcommand')

    server_parser = sub_parsers.add_parser('server', help='Server subcommand')
    server_parser.add_argument('-f', '--serial-fpath', type=str, help='path to serial tty to talk to', required=True)
    server_parser.add_argument('-b', '--serial-baudrate', type=int, default=115200, help='baudrate of serial tty')

    args = parser.parse_args()
    if args.type == 'client':
        client = ZeroMQSerialClient(args.server_host, args.server_port)
        print(f"Serial path: '{client.get_tty_path()}'")
        client.start()
    elif args.type == 'server':
        server = ZeroMQSerialServer(args.serial_fpath, args.serial_baudrate, args.server_host, args.server_port)
        server.start()
    else:
        parser.print_help()
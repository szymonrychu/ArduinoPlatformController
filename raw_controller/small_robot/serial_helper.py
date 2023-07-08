import serial
import queue
import traceback
from threading import Thread, Lock
import json
import time
from log import log
        
class SerialWrapper():

    def __init__(self, fpath, baudrate=115200):
        self._fpath = fpath
        self.__baudrate = baudrate
        self.serial = serial.Serial(fpath, baudrate)


    def read_data(self):
        raw_data = None
        try:
            if self.serial.inWaiting():
                raw_line = self.serial.readline()
                if raw_line is not None:
                    raw_data = raw_line.decode('ascii')
                    if raw_data != '':
                        if raw_data[-1] == '\n':
                            raw_data = raw_data[:-1]
        except UnicodeDecodeError:
            log.error('cannot parse "{}"'.format(raw_data))
            self.repair_serial()
        return raw_data

    def write_data(self, raw_data):
        try:
            self.serial.write('{}\n'.format(raw_data).encode())
            return True
        except TypeError:
            tb = traceback.format_exc()
            log.error(str(tb))
            self.repair_serial()

    def repair_serial(self):
        try:
            self.serial.close()
            self.serial = None
        except Exception:
            tb = traceback.format_exc()
            log.error(str(tb))
        SerialWrapper.__init__(self, self._fpath, self.__baudrate)

class ThreadedSerialWrapper(Thread, SerialWrapper):

    def __init__(self, fpath:str, baudrate:int=115200, new_data_handler=None, sleep_time:float=0.001):
        Thread.__init__(self, target=self.__handle_input_data)
        SerialWrapper.__init__(self, fpath, baudrate)
        self.__data_lock = Lock()
        self.__data = {}
        self._running = False
        self._sleep_time = sleep_time
        self.__data_handler = new_data_handler

    def start(self) -> None:
        self._running = True
        Thread.start(self)

    def running(self) -> bool:
        return self._running

    def stop(self) -> None:
        self._running = False
        Thread.join(self)

    def get_last_data(self) -> dict:
        with self.__data_lock:
            return self.__data

    def __handle_input_data(self):
        while self._running:
            data = self.read_data()
            if not data:
                continue
            _data = None
            try:
                _data = json.loads(data)
            except json.decoder.JSONDecodeError:
                pass
            if not _data:
                continue
            with self.__data_lock:
                self.__data = _data
            self.__data_handler()
            time.sleep(self._sleep_time)

    def __default_input_handler(self, *args, **kwargs):
        pass

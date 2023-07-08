#!/usr/bin/env python3

import tkinter
import time
from serial_helper import ThreadedSerialWrapper
from threading import Thread


class SerialController(ThreadedSerialWrapper):

    def __init__(self, fpath:str, baudrate:int=115200):
        ThreadedSerialWrapper.__init__(self, fpath, baudrate, new_data_handler=self.__handle_data)
        self._pressed_keys = []
        self._handled_keys = []
        # self._key_press_thread = Thread(target=self.__handle_key_press)

    def start(self) -> None:
        ThreadedSerialWrapper.start(self)
        # self._key_press_thread.start()
        self._root = tkinter.Tk()
        self._root.wm_title("Robot Controller")
        self._root.protocol("WM_DELETE_WINDOW", self.stop)
        self._root.bind("<Key>", self.__key_press_event)
        self._root.bind("<KeyRelease>", self.__key_release_event)
        self._root.mainloop()

    def stop(self) -> None:
        # self._key_press_thread.join()
        ThreadedSerialWrapper.stop(self)
        self._root.destroy()

    def __key_press_event(self, event):
        if event.char not in self._handled_keys:
            if 'w' == event.char:
                self.write_data('{"motor1":{"distance":0.2,"velocity":0.5},"motor2":{"distance":0.2,"velocity":0.5},"motor3":{"distance":0.2,"velocity":0.5},"motor4":{"distance":0.2,"velocity":0.5}}')
            elif 's' == event.char:
                self.write_data('{"motor1":{"distance":-0.2,"velocity":0.5},"motor2":{"distance":-0.2,"velocity":0.5},"motor3":{"distance":-0.2,"velocity":0.5},"motor4":{"distance":-0.2,"velocity":0.5}}')
            elif 'a' == event.char:
                self.write_data('{"turn": {"angle": -0.39265, "velocity": 0.5, "x": -0.4}}')
            elif 'd' == event.char:
                self.write_data('{"turn": {"angle":0.39265, "velocity": 0.5, "x": 0.4}}')
            elif 'q' == event.char:
                self.write_data('{"turn": {"angle": -0.39265, "velocity": 0.5}}')
            elif 'e' == event.char:
                self.write_data('{"turn": {"angle": 0.39265, "velocity": 0.5}}')
            self._handled_keys.append()
    
    def __key_release_event(self, event):
        self._handled_keys.remove(event.char)

    def __handle_data(self):
        pass
        # print(self.get_last_data())

    # def __handle_key_press(self):
    #     while self.running():
    #         print(self._pressed_keys)
    #         # if len(self._pressed_keys) < 1:
    #         #     self.write_data('{"reset":{"queue":true}}')

    #         time.sleep(1.0)


def main():
    sc = SerialController("/dev/cu.usbmodem136716701")
    sc.start()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3


import serial
import traceback
import threading
import time
import queue
import os, struct, array, math, traceback, threading
from fcntl import ioctl

class Loggable():

    def _log_debug(self, data):
        print("D:{}".format(data))

    def _log_info(self, data):
        print("I:{}".format(data))

    def _log_warning(self, data):
        print("W:{}".format(data))

    def _log_error(self, data):
        print("E:{}".format(data))

    def _log_exception(self, data):
        print("!:{}".format(data))

class SerialWrapper(Loggable):

    def __init__(self, fpath, sbaudrate=115200):
        self.serial = serial.Serial(fpath, sbaudrate)
        self._log_debug('SerialWrapper:__init__: Initialized serial {} with baudrate {}'.format(fpath, sbaudrate))

    def _read_data(self):
        raw_data = None
        try:
            if self.serial.inWaiting():
                raw_data = self.serial.readline().decode('ascii')
                if raw_data != '':
                    if raw_data[-1] == '\n':
                        raw_data = raw_data[:-1]
        except UnicodeDecodeError:
            self._log_error('SerialWrapper:_read_data: cannot parse "{}"'.format(raw_data))

        # self._log_info('SerialWrapper:_read_data: readed {}'.format(raw_data))
        return raw_data

    def _write_data(self, raw_data):
        try:
            self.serial.write('{}\n'.format(raw_data).encode())
            # self._log_info('SerialWrapper:_write_data: written {}'.format(raw_data))
        except TypeError:
            tb = traceback.format_exc()
            self._log_error('SerialWrapper:_write_data: can\'t write "{}"'.format(raw_data))
            self._log_error('SerialWrapper:_write_data: traceback: "{}"'.format(tb))

class ThreadWrapper(threading.Thread, Loggable):

    def __init__(self, loop_handler=None, sleep_time=0.1):
        threading.Thread.__init__(self, target=self.__loop)
        self.process_tasks = False
        self.__loop_handler = loop_handler or self.__default_loop_handler
        self.__sleep_time = sleep_time

    def __loop(self):
        self._log_debug('ThreadWrapper:__loop: starting thread with sleep time {}'.format(self.__sleep_time))
        while self.process_tasks:
            self.__loop_handler()
            time.sleep(self.__sleep_time)

    def __default_loop_handler(self):
        self._log_warning('ThreadWrapper:loop_handler: undefined!')
        pass

    def start(self):
        self.process_tasks = True
        threading.Thread.start(self)

    def join(self, timeout=None):
        self.process_tasks = False
        threading.Thread.join(self, timeout=timeout or self.__sleep_time*10)

class DS4(threading.Thread):
    axis_names = {
        0x00 : 'LJOY_X',
        0x01 : 'LJOY_Y',
        0x02 : 'L2(analog)',
        0x03 : 'RJOY_X',
        0x04 : 'RJOY_Y',
        0x05 : 'R2(analog)',
        0x10 : 'left/right',
        0x11 : 'up/down',
    }
    button_names = {
        0x130 : 'X',
        0x131 : 'Crcl',
        0x133 : 'Tr',
        0x134 : 'Sq',
        0x136 : 'L1',
        0x137 : 'R1',
        0x138 : 'L2(digital)',
        0x139 : 'R2(digital)',
        0x13a : 'select',
        0x13b : 'start',
        0x13c : 'mode',
        0x13d : 'LJOY',
        0x13e : 'RJOY',
    }

    def __init__(self, sbaudrate=115200, js0path='/dev/input/js0'):
        threading.Thread.__init__(self, target=self.main_loop)
        self.jsdev = open(js0path, 'rb')
        self.button_map = []
        self.axis_map = []
        self.axis_states = {}
        self.button_states = {}
        self.__handle_controller = True

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = DS4.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = DS4.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

    def start(self):
        self.__handle_controller = True
        threading.Thread.start(self)

    def join(self, *args, **kwargs):
        self.__handle_controller = False
        threading.Thread.join(self, *args, **kwargs)
        
    def get_device_name(self):
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
        print('Device name: %s' % js_name)

    def main_loop(self):
        while self.__handle_controller:
            evbuf = self.jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)
                initial = False
                if type & 0x80:
                    initial = True
                    # print("(initial)", end="")

                if type & 0x01:
                    button = self.button_map[number]
                    if button:
                        self.button_states[button] = value
                        if not initial:
                            if value:
                                self.event_callback(button, 1)
                            else:
                                self.event_callback(button, 0)

                if type & 0x02 and not initial:
                    axis = self.axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        self.axis_states[axis] = fvalue
                        if not initial:
                            self.event_callback(axis, fvalue)

    def event_callback(self, event_name, event_value):
        pass

class SerialThread(ThreadWrapper, SerialWrapper):

    def __init__(self, fpath, sbaudrate=115200, output_handler=None):
        ThreadWrapper.__init__(self, loop_handler=self.handler)
        SerialWrapper.__init__(self, fpath, sbaudrate)
        self.__input_queue = queue.Queue()
        self.__output_handler = output_handler
        self.send_receive_count = 0
        self.angle = 0
        self.steer = 0

    def parse_input(self, raw_request):
        # self._log_warning('SerialThread:_parse_input: unhandled!')
        return raw_request

    def parse_response(self, raw_response):
        # self._log_warning('SerialThread:_parse_response: unhandled!')
        return raw_response

    def put(self, raw_request):
        # self._log_debug('SerialThread:put: raw_request {}'.format(raw_request))
        parsed_input = self.parse_input(raw_request)
        if parsed_input:
            self.__input_queue.put(parsed_input)

    def handler(self):
        try:
            parsed_input = self.__input_queue.get(block=False)
            self._write_data(parsed_input)
        except queue.Empty:
            pass
        raw_response = None
        raw_response = self._read_data()

        if raw_response and self.__output_handler:
            response = self.parse_response(raw_response)
            self.__output_handler(response)
        
        time.sleep(0.01)





class DS4PlatformController(DS4):

    def __init__(self, serial1, serial2, serial3, serial4):
        self.handler_thread = threading.Thread(target=self.handler)
        DS4.__init__(self)
        serials = [
            serial1, serial2, serial3, serial4
        ]
        self.serial_threads = []
        for serial in serials:
            self.serial_threads.append(SerialThread(serial, output_handler=self.serial_output_handler))
        self.last_states = {}
        self.__operate = True

        self.angle = 0
        self.power = 0
        self.steer = 0
        self.distance = 10000.0

    def start(self):
        self.__operate = True
        threading.Thread.start(self)
        for serial in self.serial_threads:
            serial.start()
        self.handler_thread.start()

    def join(self, *args, **kwargs):
        self.handler_thread.join(*args, **kwargs)
        for serial in self.serial_threads:
            serial.join(*args, **kwargs)
        self.__operate = False
        threading.Thread.join(self, *args, **kwargs)

    def serial_output_handler(self, data):
        print(data)

    def event_callback(self, event_name, event_value):
        if event_name == 'L1' and event_value == 1:
            self.angle = 0
        if event_name == 'R1' and event_value == 1:
            self.steer = 0
        if event_name == 'start' and event_value == 1:
            self.serial_threads[0].put('G99')
            self.serial_threads[1].put('G99')
            self.serial_threads[2].put('G99')
            self.serial_threads[3].put('G99')

    def handler(self):
        while self.__operate:
            angle = -self.axis_states['LJOY_X']
            angle_tmp = (self.angle + angle*0.1)
            if angle_tmp > 1:
                angle_tmp = 1
            if angle_tmp < -1:
                angle_tmp = -1
            self.angle = angle_tmp
            self.power = -self.axis_states['LJOY_Y']

            steer = -self.axis_states['RJOY_X']
            steer_tmp = (self.steer + steer*0.1)
            if steer_tmp > 1:
                steer_tmp = 1
            if steer_tmp < -1:
                steer_tmp = -1
            self.steer = steer_tmp

            # print("angle:{0:.2f} steer:{1:.2f} power:{2:.2f}".format(self.angle, self.steer, self.power))
            
            power = self.handle_power()
            self.serial_threads[0].put(self.handle_angle1(angle, -steer))
            self.serial_threads[1].put(self.handle_angle2(angle, -steer))
            self.serial_threads[2].put(self.handle_angle3(angle, -steer))
            self.serial_threads[3].put(self.handle_angle4(angle, -steer))
            if power:
                self.serial_threads[0].put(power)
                self.serial_threads[1].put(power)
                self.serial_threads[2].put(power)
                self.serial_threads[3].put(power)
            time.sleep(0.1)

    def handle_angle1(self, ANG, STE):
        inner_angle = 0
        outer_angle = 0
        r = 0
        try:
            l = 50.0
            b = 33.0
            r = l/(2*math.tan(math.pi*STE/5.0))
            if abs(r) < b/2.0:
                r = b/2.0
            s = math.sqrt(r*r-(l/2.0)*(l/2.0))-b/2.0

            inner_angle = max(math.tan(l/s), 0)
            outer_angle = max(math.tan(l/(b+s)), 0)
        except ValueError:
            pass
        except ZeroDivisionError:
            pass
        power = 0.8
        if r > 0:
            angle = (180 / math.pi)*(ANG-outer_angle)
        else:
            angle = (180 / math.pi)*(ANG+inner_angle)
            if inner_angle < outer_angle:
                power = power * (inner_angle/outer_angle)
            elif outer_angle < inner_angle:
                power = power * (outer_angle/inner_angle)

        data = "G01 {0:.5f} {1:.5f}".format(angle, power)
        if 'handle_angle1' not in self.last_states.keys() or data != self.last_states['handle_angle1']:
            self.last_states['handle_angle1'] = data
            print('handle_angle1', data)
            return data
        else:
            return None

    def handle_angle2(self, ANG, STE):
        inner_angle = 0
        outer_angle = 0
        r = 0
        try:
            l = 50.0
            b = 33.0
            r = l/(2*math.tan(math.pi*STE/5.0))
            if abs(r) < b/2.0:
                r = b/2.0
            s = math.sqrt(r*r-(l/2.0)*(l/2.0))-b/2.0

            inner_angle = max(math.tan(l/s), 0)
            outer_angle = max(math.tan(l/(b+s)), 0)
        except ValueError:
            pass
        except ZeroDivisionError:
            pass
        power = 0.8
        if r > 0:
            angle = (180 / math.pi)*(ANG+outer_angle)
        else:
            angle = (180 / math.pi)*(ANG-inner_angle)
            if inner_angle < outer_angle:
                power = power * (inner_angle/outer_angle)
            elif outer_angle < inner_angle:
                power = power * (outer_angle/inner_angle)

        data = "G01 {0:.5f} {1:.5f}".format(angle, power)
        if 'handle_angle2' not in self.last_states.keys() or data != self.last_states['handle_angle2']:
            self.last_states['handle_angle2'] = data
            print('handle_angle2', data)
            return data
        else:
            return None

    def handle_angle3(self, ANG, STE):
        inner_angle = 0
        outer_angle = 0
        r = 0
        try:
            l = 50.0
            b = 33.0
            r = l/(2*math.tan(math.pi*STE/5.0))
            if abs(r) < b/2.0:
                r = b/2.0
            s = math.sqrt(r*r-(l/2.0)*(l/2.0))-b/2.0

            inner_angle = max(math.tan(l/s), 0)
            outer_angle = max(math.tan(l/(b+s)), 0)
        except ValueError:
            pass
        except ZeroDivisionError:
            pass
        power = 0.8
        if r > 0:
            angle = (180 / math.pi)*(ANG+inner_angle)
        else:
            angle = (180 / math.pi)*(ANG-outer_angle)
            if inner_angle < outer_angle:
                power = power * (inner_angle/outer_angle)
            elif outer_angle < inner_angle:
                power = power * (outer_angle/inner_angle)

        data = "G01 {0:.5f} {1:.5f}".format(angle, power)
        if 'handle_angle3' not in self.last_states.keys() or data != self.last_states['handle_angle3']:
            self.last_states['handle_angle3'] = data
            print('handle_angle3', data)
            return data
        else:
            return None

    def handle_angle4(self, ANG, STE):
        inner_angle = 0
        outer_angle = 0
        r = 0
        try:
            l = 50.0
            b = 33.0
            r = l/(2*math.tan(math.pi*STE/5.0))
            if abs(r) < b/2.0:
                r = b/2.0
            s = math.sqrt(r*r-(l/2.0)*(l/2.0))-b/2.0

            inner_angle = max(math.tan(l/s), 0)
            outer_angle = max(math.tan(l/(b+s)), 0)
        except ValueError:
            pass
        except ZeroDivisionError:
            pass
        power = 0.8
        if r > 0:
            angle = (180 / math.pi)*(ANG-inner_angle)
        else:
            angle = (180 / math.pi)*(ANG+outer_angle)
            if inner_angle < outer_angle:
                power = power * (inner_angle/outer_angle)
            elif outer_angle < inner_angle:
                power = power * (outer_angle/inner_angle)

        data = "G01 {0:.5f} {1:.5f}".format(angle, power)
        if 'handle_angle4' not in self.last_states.keys() or data != self.last_states['handle_angle4']:
            self.last_states['handle_angle4'] = data
            print('handle_angle4', data)
            return data
        else:
            return None

    def handle_power(self):
        if self.power >= 0:
            data = "G02 {0:.5f} {1:.5f}".format(self.distance, 2.0*round(abs(self.power)/4.0, 1))
        else:
            data = "G02 {0:.5f} {1:.5f}".format(-self.distance, 2.0*round(abs(self.power)/4.0, 1))
        
        if 'handle_power' not in self.last_states.keys() or data != self.last_states['handle_power']:
            self.last_states['handle_power'] = data
            print('handle_power', data)
            return data
        else:
            return None



if __name__ == "__main__":
    ds4 = DS4PlatformController(
        '/dev/teensy1', '/dev/teensy2', '/dev/teensy3', '/dev/teensy4', 
    )
    ds4.start()
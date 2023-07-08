#!/usr/bin/env python3

import tkinter
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.backend_bases import key_press_handler
from matplotlib import pyplot as plt, animation
import numpy as np
from serial_helper import ThreadedSerialWrapper
import json

def _min(lists:list):
    _tmp = []
    for list in lists:
        _tmp.extend(list)
    return min(_tmp)

def _max(lists:list):
    _tmp = []
    for list in lists:
        _tmp.extend(list)
    return max(_tmp)

class DynamicPlotUpdate():

    def __init__(self, x:list=[], y:list=[]) -> None:
        self._x = x
        self._y = y
    
    @property
    def x(self):
        return self._x
    
    @property
    def y(self):
        return self._y


class DynamicPlot():

    def __init__(self, fig: plt.Figure, updates:list, position=111):
        self._ax = fig.add_subplot(position)
        self._lines = []
        for update in updates:
            line, = self._ax.plot(np.array(update.x), np.array(update.y))
            self._lines.append(line)
    
    def init(self):
        for line in self._lines:
            line.set_data([], [])
    
    def animate(self, updates:list):
        _x = []
        _y = []
        for line, update in zip(self._lines, updates):
            line.set_data(np.array(update.x), np.array(update.y))
            _y.extend(update.y)
            _x.extend(update.x)
        
        self._ax.set_xlim(_min(_x), _max(_x))
        self._ax.set_ylim(_min(_y), _max(_y))

    @property
    def lines(self):
        return set(self._lines)


class MotorData():

    def __init__(self) -> None:
        self._micros = []
        self._motor_data = {}
    
    def update(self, data_json:dict, micros:float, max_items = 500):
        self._micros.append(micros)
        while len(self._micros) > max_items:
            self._micros.pop(0)
        
        for k, v in data_json.items():
            if k not in self._motor_data.keys():
                self._motor_data[k] = []
            self._motor_data[k].append(v)

            while len(self._motor_data[k]) > max_items:
                self._motor_data[k].pop(0)

    @property
    def velocity(self) -> DynamicPlotUpdate:
        return DynamicPlotUpdate(self._micros, self._motor_data.get('velocity', []))
        
    @property
    def distance(self) -> DynamicPlotUpdate:
        return DynamicPlotUpdate(self._micros, self._motor_data.get('distance', []))
    
    @property
    def steering(self) -> DynamicPlotUpdate:
        return DynamicPlotUpdate(self._micros, self._motor_data.get('steering', []))



class TKinterRobotController(ThreadedSerialWrapper):

    def __init__(self, fpath, baudrate=115200):
        ThreadedSerialWrapper.__init__(self, fpath, baudrate, input_handler=self.__handle_plot)
        self._current_micros = []
        self._motors = []
        for _ in range(4):
            self._motors.append(MotorData())

        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True

        self._root = tkinter.Tk()
        self._root.wm_title("Robot Controller")
        self._root.protocol("WM_DELETE_WINDOW", self.stop)
        
        self._fig = plt.Figure(dpi=100)

        self._distance_plot = DynamicPlot(self._fig, [m.distance for m in self._motors], 311)
        self._velocity_plot = DynamicPlot(self._fig, [m.velocity for m in self._motors], 312)
        self._steering_plot = DynamicPlot(self._fig, [m.steering for m in self._motors], 312)

        self._canvas = FigureCanvasTkAgg(self._fig, master=self._root)
        self._canvas.draw()

        self._toolbar = NavigationToolbar2Tk(self._canvas, self._root, pack_toolbar=False)
        self._toolbar.update()

        self._canvas.mpl_connect("key_press_event", self.__handle_keypress)
        self._canvas.mpl_connect("key_press_event", key_press_handler)

        self._button = tkinter.Button(master=self._root, text="Quit", command=self.stop)
        self._button.pack(side=tkinter.BOTTOM)

        self._toolbar.pack(side=tkinter.BOTTOM, fill=tkinter.X)
        self._canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)
        
        self._anim = None
        self.__data_lock = Lock()

    def stop(self) -> None:
        ThreadedSerialWrapper.stop(self)
        self._root.quit()
    
    def __init(self):
        self._velocity_plot.init()
        self._distance_plot.init()
        self._steering_plot.init()
        return self._velocity_plot.lines, self._distance_plot.lines, self._steering_plot.lines
    
    def __animate(self, i):
        with self.__data_lock:
            self._distance_plot.animate([m.distance for m in self._motors])
            self._velocity_plot.animate([m.velocity for m in self._motors])
            self._steering_plot.animate([m.steering for m in self._motors])
        return self._velocity_plot.lines, self._distance_plot.lines, self._steering_plot.lines

    def __handle_keypress(self, event):
        print(f"you pressed {event.key}")

    def __handle_plot(self, raw_data):
        try:
            data = json.loads(raw_data)
            with self.__data_lock:
                for i in range(4):
                    self._motors[i].update(data[f'motor{i+1}'], data['micros']/1000000.0)


        except json.decoder.JSONDecodeError:
            print(raw_data)

    def start(self):
        self._canvas.draw()
        self._toolbar.update()
        self._anim = animation.FuncAnimation(self._fig, self.__animate, init_func=self.__init, frames=1000, interval=33)
        ThreadedSerialWrapper.start(self)
        tkinter.mainloop()



controller = TKinterRobotController("/dev/cu.usbmodem136716701")
controller.start()


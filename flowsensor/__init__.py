"""

For using Hall sensor type flowmeter with RPi GPIO pins

As of DEc 2020 kind of abandonned.
"""


import RPi.GPIO as GPIO
import time
import abc
import threading
import types
from typing import List, Union


class Measurement:
    def __init__(self):
        self.value: float = 0
        self.time: float = 0

    def __radd__(self, other):
        """
        So if you call sum(<List(Measurement)>
        Or m1 + m2 I suppose
        """
        return other.value + self.value

    def __mul__(self, other):
        return other.value * self.value

    def __rmul__(self, other):
        return other.other * self.value


class OnFlowSensorChangeListener(abc.ABC):

        @abc.abstractmethod
        def on_flow_changed(self, q: Measurement, v: Measurement):
            pass

        @abc.abstractmethod
        def on_flowsensor_reset(self):
            pass

        @abc.abstractmethod
        def onStart(self):
            pass

        @abc.abstractmethod
        def onStop(self):
            pass


class HallFlowSensor:
    """
    Hall flow sensor.

    """

    DEFAULT_SAMPLE_INTERVAL = 0.5  # secs
    unit = "L/min"
    unit_correction = 1

    def __init__(
            self,
            gpio_pin,
            factor,
            offset,
            interval: float = DEFAULT_SAMPLE_INTERVAL,
            store_in_memory : bool = False
    ):

        self._lock = threading.Lock()
        self.last_t: float = -1
        self._start_time = -1
        self._interval = interval

        self._current_q = Measurement()
        self._current_v = Measurement()

        self.pin = gpio_pin
        self._hz_count = 0

        self.thread = None
        self._running = False

        self.factor = factor
        self.offset = offset

        self._on_flow_change_callbacks: List[Union[types.FunctionType, OnFlowSensorChangeListener]] = []


     
        if store_in_memory:
            ram = MemoryStorage()
            self._on_flow_change_callbacks.append(ram)


    def init_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def count_pulse(self, channel):
        self._hz_count += 1

    def update_flowrate(self):
        t = time.time()
        dt = t - self.last_t

        self._current_q.value = (1 / dt) * ((self._hz_count + self.offset) / self.factor) * HallFlowSensor.unit_correction
        
        self._hz_count = 0
        self.last_t = t

        self._current_q.time = t

        self._current_v.value = self._current_v.value + self._current_q.value * dt
        self._current_v.time = t

        for callback in self._on_flow_change_callbacks:
            if type(callback) == types.FunctionType:
                callback()
            elif isinstance(callback, OnFlowSensorChangeListener):
                callback.on_flow_changed(self._current_q, self._current_v)


    def start(self, reset=True):
        self._start_time = time.time()
        print ("Starting flow meter")
        for callback in self._on_flow_change_callbacks:
            if isinstance(callback, OnFlowSensorChangeListener):
                callback.onStart()

        if reset:
            self.reset()

        def thread_function(name):
            GPIO.add_event_detect(self.pin, GPIO.FALLING, callback=self.count_pulse)
            self._running = True
            while self._running:
                time.sleep(self._interval)
                self.update_flowrate()


            print('thread stopped')

        self.thread = threading.Thread(target=thread_function, args=(1,),daemon=True)
        self.thread.start()

    def stop(self):
        self._running= False
        GPIO.remove_event_detect(self.pin)

        for callback in self._on_flow_change_callbacks:
            if isinstance(callback, OnFlowSensorChangeListener):
                callback.onStop()

    def reset(self):
        self.last_t: float = -1
        self._hz_count = 0

        for callback in self._on_flow_change_callbacks:
            if isinstance(callback, OnFlowSensorChangeListener):
                callback.on_flowsensor_reset()

    def add_on_change_listener(self, callback: Union[OnFlowSensorChangeListener, types.FunctionType]):
        if callback not in self._on_flow_change_callbacks:
            self._on_flow_change_callbacks.append(callback)

    def clear_on_flowsensor_change_listeners(self):
        self._on_flow_change_callbacks = []

    def run_calibration(self, test_volume: float, auto_stop=True):
        """
        TODO

        Args:
            test_volume: Amount of fluid you are using to calibrate, in litres
            auto_stop: If true, calibration will stop automatically when no flow detected.

        Returns:
    """

    @property
    def current_volume(self):
        return self._current_v

    @property
    def current_flowrate(self):
        return self._current_q

    @property
    def current_runtime(self):
        return time.time() - self._start_time



class MemoryStorage(OnFlowSensorChangeListener):
    """
    Stores data in RAM
    """

    def __init__(self):
        pass

    def on_flow_changed(self, q: Measurement, v: Measurement):
        pass

    def on_flowsensor_reset(self):
        pass

    def onStart(self):
        pass

    def onStop(self):
        pass

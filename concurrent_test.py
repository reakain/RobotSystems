import concurrent.futures
import copy
import time
from readerwriterlock import rwlock

class dumb_bus(object):
    def __init__(self):
        self.lock = rwlock.RWLockWriteD()
        self.message = None

    def write(self,msg):
        # Why the deep copy? 
        # To be certain that we are passing by value, and not reference, 
        # no matter the data type of msg
        with self.lock.gen_wlock():
            self.message = copy.deepcopy(msg)

    def read(self):
        # Why the deep copy? 
        # To be certain that we are passing by value, and not reference, 
        # no matter the data type of msg
        with self.lock.gen_rlock():
            return copy.deepcopy(self.message)

def sensor_function(sensor_values_bus, sensor_delay):
    while True:
        # read/write/act
        # Sleep the delay time
        time.sleep(sensor_delay)

def interpreter_function(sensor_values_bus, interpreter_bus, interpreter_delay):
    while True:
        # read/write/act
        # Sleep the delay time
        time.sleep(interpreter_delay)



if __name__ == "__main__":
    sensor_values_bus = dumb_bus()
    interpreter_bus = dumb_bus()
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        eSensor = executor.submit(sensor_function,
                    sensor_values_bus, sensor_delay)
        eInterpreter = executor.submit(interpreter_function,
                    sensor_values_bus, interpreter_bus, interpreter_delay)
    eSensor.result()
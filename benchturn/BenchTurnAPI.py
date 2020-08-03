import threading
import xmlrpclib
from threading import Lock
from time import sleep

# use BenchTurn server functions as API
# to use this code initiate new Client instance
# Based on Scorpy API


lock = Lock()


# decorator to use lock on each API call
def with_lock(func):
    def func_with_args(*args, **kwargs):
        lock.acquire()
        ret_val = func(*args, **kwargs)
        lock.release()
        return ret_val

    return func_with_args


class Client:
    def __init__(self):
        self.server = xmlrpclib.ServerProxy('http://132.66.51.252:8002')
        self.status = {}
        self.get_benchturn_info()
        print "methods: {}".format(str(self.server.system.listMethods()))

    @with_lock
    def get_status(self):
        return self.server.get_status()

    @with_lock
    def toggle_door(self):
        self.server.toggle_door()
        return True

    @with_lock
    def toggle_chuck(self):
        self.server.toggle_chuck()
        return True

    @with_lock
    def change_tool(self, tool=1):
        self.server.change_tool(tool)
        return True

    def move_step(self, axis="z", distance=-1, speed=300, wait=True):
        self._move_step(axis, distance, speed)
        if wait:
            while self.status.get("is_moving"):
                sleep(0.1)
            sleep(1)
        return True

    @with_lock
    def _move_step(self, axis="z", distance=-1, speed=300):
        self.server.move_step(axis, distance, speed)
        return True

    @with_lock
    def test(self):
        self.server.test()
        return True

    def move_absolute(self, x="None", z="None", speed=300, wait=True, short=True):
        # pass "None" because the server will not
        # allow to pass None values
        self._move_absolute(x, z, speed, short)
        if wait:
            self.wait_for_move_finish()
        return True

    @with_lock
    def _move_absolute(self, x="None", z="None", speed=300, short=True):
        self.server.move_absolute(x, z, speed, short)
        return True

    @with_lock
    def change_spindle_speed(self, speed=0):
        self.server.change_spindle_speed(speed)
        return True

    @with_lock
    def start_spinning(self, is_cw=True, speed=0):
        self.server.start_spinning(is_cw, speed)
        return True

    @with_lock
    def stop_spinning(self):
        self.server.stop_spinning()
        return True

    @with_lock
    def clear_status_msg(self):
        self.server.clear_status_msg()
        return True

    def home(self):
        self._home()
        self.wait_for_homing()
        return True

    @with_lock
    def _home(self):
        self.server.home()
        return True

    @with_lock
    def move_continuous(self, axis, speed=300, direction=True):
        self.server.move_continuous(axis, speed, direction)
        return True

    # wait for movement to finish (without lock)
    def wait_for_move_finish(self):
        sleep(1)
        while self.status.get("is_moving"):
            sleep(0.1)

    def wait_for_homing(self):
        sleep(1)
        while self.status.get("is_homing_in_progress"):
            sleep(1)
        sleep(1)
        return True

    @with_lock
    def go_to_point(self, point):
        self.server.go_to_point(point)
        return True

    @with_lock
    def draw_from_coords(self, x_arr, z_arr):
        self.server.draw_from_coords(x_arr, z_arr)
        return True

    # get information async
    def get_benchturn_info(self):
        def callback():
            while True:
                self.status = self.get_status()
                sleep(0.3)

        t = threading.Thread(target=callback)
        t.start()
        return True

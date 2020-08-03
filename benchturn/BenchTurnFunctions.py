import struct
import threading
import time
import numpy as np

import BenchTurnUtils as utils


# do not send control messages
# in a too short interval
def wait_between_messages():
    time.sleep(0.01)


# all functions here will be registered to the server,
# and can be called using the API
# This is also the object containing the robot's information
class BenchTurnFunctions(object):
    def __init__(self):
        self.kill = False
        self.is_homing_in_progress = False
        self.x_pos = self.z_pos = self.door_closed = self.chuck_closed = \
            self.is_spindle_spinning = self.spindle_speed = \
            self.z_move = self.x_move = self.is_moving = None
        self.should_close_door = False
        self.should_close_chuck = False
        self.status = {}

        # use this socket to periodically get the robot's state
        # port 5000
        self._info_sock = utils.get_socket(False)

        # control socket for sending commands, such as move, etc.
        # port 2500
        self._ctrl_sock = utils.get_socket(True)

        self._moving_on_X = False
        self._moving_on_Z = False

        self.tool_index = 0
        self._desired_tool_index = 0
        self.change_tool_counter = 0
        # self.should_lock_tool = False
        self.is_tool_change_is_progress = False
        self.should_lock_tool = False

        self.status_msg = ""

        self.emergency_stop = False

        self.safe_step = 0.5
        self.workpiece = {}

        self.send_init_messages()
        self.get_info()

    def send_init_messages(self):
        # when the server is first loading send initial messages to robot
        # (captured using WireShark)
        try:
            messages = utils.json_to_hex_array(filename="init_msgs")
            for msg in messages:
                self._ctrl_sock.send_msg(msg, decode=False, debug=False)
        except Exception as e:
            print e
            raise e

    def toggle_door(self):
        # debounce requests:
        # do not change anything if the desired
        # state is different than the current state
        # (meaning let the previous change end)
        if self.should_close_door is not self.door_closed:
            return True
        self.should_close_door = not self.should_close_door
        return True

    def toggle_chuck(self):
        # debounce requests:
        # do not change anything if the desired
        # state is different than the current state
        # (meaning let the previous change end)
        if self.should_close_chuck is not self.chuck_closed:
            return True
        else:
            self.should_close_chuck = not self.should_close_chuck
        return True

    def change_tool(self, tool_req):
        # change desired tool index according to change tool request
        # the tool is replaced by messages from info socket
        def _change_tool(tool):
            self._desired_tool_index = tool
            self.is_tool_change_is_progress = True
            while self.status.get("tool_index") != tool:
                time.sleep(0.5)
            time.sleep(1)
            self.should_lock_tool = True
            time.sleep(3)
            self.is_tool_change_is_progress = False
            self.should_lock_tool = False
            return True

        if int(tool_req) not in range(1, 5):
            self.set_status_msg("invalid tool index", True)
            return False
        if tool_req == self.status.get("tool_index"):
            return True
        t = threading.Thread(target=_change_tool, args=(tool_req,))
        t.start()
        return True

    def change_tool_request_to_send(self):
        # calculate first byte of fet info message
        # it should be \x02 if the tool turret needs to move, else \x00
        # locking the turret is done by sending "\x0f"
        should_change_tool = self._desired_tool_index != 0 and \
                             self._desired_tool_index != self.tool_index

        # tool will be changed only if door is closed!
        if should_change_tool and self.door_closed:
            ret = "\x03"
        else:
            ret = "\x00"
            if self.should_lock_tool:
                ret = "\x0f"
        return ret

    def reset_registers(self):
        # used before executing G code in CNC-base, and before homing
        # function based on BenchMill's reset_registers
        self.reset_program()
        for msg in utils.reset_registers_messages:
            self._ctrl_sock.send_msg(msg, decode=True)
            wait_between_messages()

    def reset_program(self):
        # send reset messages before start program
        # function based on BenchMill's reset_program
        for msg_to_send in utils.reset_program_messages:
            self._ctrl_sock.send_msg(msg_to_send, decode=False)
            wait_between_messages()

    def move_step(self, axis="z", distance=-1, speed=300, wait=False, short=True):
        # move distance by *single* axis
        # speed units are mm / min
        # distance units are mm
        if abs(distance) < 0.0002:
            return True
        if self.emergency_stop:
            raise Exception("emergency stop is on")
        if self.kill:
            raise Exception("killed")
        self.reset_program()
        self.set_speed_step(axis, speed)

        # print "move step: {} on {}".format(distance, axis)

        # choose the right direction
        if axis == "x":
            prefix = utils.Move.move_x_prefix
            estimated_pos = (self.x_pos + distance, self.z_pos)
            if estimated_pos[0] > utils.max_x or estimated_pos[0] < utils.min_x:
                self.set_status_msg("ERROR on move_step: trying to move to x= {}, "
                                    "which is not in the robot's workspace".format(estimated_pos[0]), True)
                return False
        else:
            prefix = utils.Move.move_z_prefix
            estimated_pos = (self.x_pos, self.z_pos + distance)
        # translate distance integer to hex sting
        distance_str = struct.pack('i', int(10000 * distance))
        msg = "{}{}".format(prefix, distance_str)
        self._ctrl_sock.send_msg(msg, decode=False)
        if wait:
            # print "waiting ..."
            if short:
                utils.wait_for_short_move_finish(self)
            else:
                utils.wait_for_move_finish(self)
            # print "move is over"
            final_pos = (self.x_pos, self.z_pos)
            # if final_pos != estimated_pos:
            #     self.set_error_message("Error on move_step: expected position {}, got {}".format(
            #         estimated_pos, final_pos)
            #     )
        return True

    # for move step: set the speed per axis
    def set_speed_step(self, axis, speed=None):
        messages = None
        if axis is None or speed is None:
            return
        speed_str = struct.pack('i', int(speed * 10000))
        if axis == "x":
            messages = utils.SetSpeed.set_speed_x
        if axis == "z":
            messages = utils.SetSpeed.set_speed_z

        for msg in messages:
            self._ctrl_sock.send_msg(msg.format(speed_str), decode=False)

    # move single axis to coord value
    def move_absolute(self, x="None", z="None", speed=300, wait=False, short=True):
        if self.emergency_stop:
            raise Exception("emergency stop is on")
        if self.kill:
            raise Exception("killed")
        if z != "None":
            # distance = round(z - self.z_pos, 4)
            distance = z - self.z_pos
            if abs(distance) < 0.0001:
                distance = 0.0001 * utils.sign(distance)
            print "moving from {} to {}: {} mm".format(self.z_pos, z, distance)
            self.move_step("z", distance, speed=speed, wait=wait, short=short)
        elif x != "None":
            # clip x to make sure it is in the robot's valid range
            x = max(utils.min_x, min(x, utils.max_x))
            # distance = round(z - self.z_pos, 4)
            distance = x - self.x_pos
            print "moving from {} to {}: {} mm".format(self.x_pos, x, distance)
            self.move_step("x", distance, speed=speed, wait=wait, short=short)
        return True

    # start spinning the spindle at a given speed
    # function based on BenchMill's start_spinning
    # cw not working
    def start_spinning(self, cw, speed=0):
        first_message = utils.Spindle.start_spinning_first_message_cw if cw else utils.Spindle.start_spinning_first_message_ccw
        self._ctrl_sock.send_msg(first_message, decode=False)
        self.change_spindle_speed(speed=speed)

        for msg in utils.Spindle.start_spinning_last_messages:
            self._ctrl_sock.send_msg(msg, decode=False)
        return True

    # stop spinning the spindle
    # function based on BenchMill's stop_spinning
    def stop_spinning(self):
        self._ctrl_sock.send_msg(utils.Spindle.stop_spinning_message, decode=False)
        return True

    # change spindle speed
    # function based on BenchMill's change_spindle_speed
    # speed units: RPM
    def change_spindle_speed(self, speed=0):
        speed_str = utils.spindle_speed_to_send(speed)
        if speed != 0:
            self._ctrl_sock.send_msg("{}{}".format(utils.Spindle.change_speed_prefix, speed_str), decode=False)
        else:
            self._ctrl_sock.send_msg(utils.Spindle.speed_0_message, decode=False)
        return True

    # home robot asynchronously
    def home(self):
        t = threading.Thread(target=self._home)
        t.start()
        return True

    # home the robot
    # flow: reset, and home per axis
    # move the robot near the origin and set the final position as 0
    def _home(self):
        self.is_homing_in_progress = True
        self.reset_registers()

        print("homing x ...")
        for msg in utils.homing_x:
            self._ctrl_sock.send_msg(msg, decode=False)
            wait_between_messages()
        utils.wait_for_move_finish(self)

        print("homing z ...")
        for msg in utils.homing_z:
            self._ctrl_sock.send_msg(msg, decode=False)
            wait_between_messages()
        utils.wait_for_move_finish(self)
        print("Done Homing")
        time.sleep(1)
        # resend init messages to fix bug
        # other functions will no work without this line
        self.send_init_messages()
        self.is_homing_in_progress = False
        if self.x_pos != 0 or self.z_pos != 0:
            self.set_status_msg(
                "Homing failed, probably coordinates are out of range. expected position (0,0), got {}"
                    .format((self.x_pos, self.z_pos)
                            ), True)
            return False
        self.set_status_msg("Homing completed", False)
        return True

    def stop_all(self, raise_ex=False):
        self.stop_spinning()
        self.set_speed_step(0)
        if raise_ex:
            self.kill = True
        return True

    ###############################
    # high level functions
    ###############################

    # go in a strait line from current position to point = (x, z)
    # noinspection PyTypeChecker
    def go_to_point(self, point):
        if self.emergency_stop:
            raise Exception("emergency stop is on")
        if self.kill:
            raise Exception("killed")
        x, z = round(point[0], 4), round(point[1], 4)

        distance = utils.dist(self.x_pos, self.z_pos, x, z)
        step_x, step_z = utils.get_steps(self.x_pos, self.z_pos, x, z)
        if distance == 0:
            return True
        print "moving from {} {} to {} {}".format(self.x_pos, self.z_pos, x, z)
        print "steps: {} {}".format(step_x, step_z)
        if step_z == 0:
            # move directly on x
            print "move directly on x"
            self.move_absolute(x=x, speed=50, wait=True, short=False)
        elif step_x == 0:
            # move directly on z
            print "move directly on z"
            self.move_absolute(z=z, speed=50, wait=True, short=False)
        else:
            while abs(self.x_pos - x) > utils.minimal_step and abs(self.z_pos - z) > utils.minimal_step:
                self.move_step("x", step_x, speed=50, wait=True, short=True)
                time.sleep(0.2)
                self.move_step("z", step_z, speed=50, wait=True, short=True)
                time.sleep(0.2)
                new_distance = utils.dist(self.x_pos, self.z_pos, x, z)
                if new_distance >= distance:
                    self.set_status_msg("Error at go_to_point: tool is not moving towards the desired point", True)
                    return False

            # the tool is now close enough to the target and can move there directly
            time.sleep(1)
            # self.move_absolute(x=x, speed=50, wait=True, short=True)
            self.move_step("z", z - self.z_pos, speed=50, wait=True, short=True)
            time.sleep(0.2)
            # print "now at {}".format((self.x_pos, self.z_pos))
            # self.move_absolute(z=z, speed=50, wait=True, short=True)
            self.move_step("x", x - self.x_pos, speed=50, wait=True, short=True)
        time.sleep(0.2)

        if not self.check_position_after_movement(x, z):
            return False

        print "final position: {}".format((self.x_pos, self.z_pos))
        return True

    # after go_to_point: check if desired position is different then the final position
    def check_position_after_movement(self, x, z):
        if (self.x_pos, self.z_pos) != (x, z):
            # ignore small errors
            error_value = round(abs(self.x_pos - x), 4), round(abs(self.z_pos - z), 4)
            if error_value[0] > 0.0001 or error_value[1] > 0.0001:
                self.set_status_msg("Error at go_to_point: Final position is not as expected."
                                    "expected: {}, got: {}".format((x, z), (self.x_pos, self.z_pos)), True)
                if error_value[0] > 0.1 or error_value[1] > 0.1:
                    # the position difference is too big and the process should stop
                    # something went wrong
                    self.stop_all()
                    raise Exception("Position difference is too big. tool is stuck")
                return False
        return True

    # cut the workpiece according to the given drawing
    def draw_from_coords(self, x_arr, z_arr):
        def draw_from_coords_async():
            # offset between absolute and machine coordinate system
            self.coords_offset = (-53.052, -192.921)
            # WARNING: do not change workpiece size without checking carefully it works
            self.WORKPIECE_SIZE = (9.5, 38)
            safety_buffer = 1

            if not self.door_closed or not self.chuck_closed:
                self.set_status_msg("Please verify BenchTurn is ready", True)
                return False

            # create workpiece model
            self.workpiece = self.create_workpiece(z_arr)

            # first positioning
            # element-wise addition of self.coords_offset and self.WORKPIECE_SIZE tuples,
            # with safety buffer
            first_pos = [sum(el) + safety_buffer for el in zip(self.coords_offset, self.WORKPIECE_SIZE)]
            self.move_absolute(z=first_pos[1], speed=700, wait=True, short=False)
            self.move_absolute(x=first_pos[0], speed=600, wait=True, short=False)
            print "after moving to first_pos: {}".format((self.x_pos, self.z_pos))
            time.sleep(1)

            self.start_spinning(True, 1300)

            ##########################################################################
            # first, cut strait lines along z axis to the highest x
            z_start, z_end = min(z_arr), max(z_arr)
            safe_cut_x = max(x_arr)
            self.safe_cut_along_z(safe_cut_x, z_start, z_end, first_pos[0] - 1)

            # update all z values where x should be safe_cut_x as finished
            print("safe_cut_x = {}, z where x=safe_cut_x = {}".format(
                safe_cut_x, np.array(z_arr)[np.array(x_arr) == safe_cut_x]))
            for z in np.array(z_arr)[np.array(x_arr) == safe_cut_x]:
                self.update_workpiece(safe_cut_x, z, finished_arg=True)

            finished = False
            cycle_index = 0
            while not finished:
                cycle_index += 1
                self.set_status_msg("starting rough cycle {}".format(cycle_index), False)
                finished = self._cut_shape(x_arr, z_arr)

            self.stop_spinning()
            self.set_status_msg("Finished drawing", False)
            return True

        t = threading.Thread(target=draw_from_coords_async)
        t.start()
        return True

    # cut along z axis, taking safe steps on each rough cycle
    # x: the desired depth of line in the material
    # z1, z2: the line start / end points
    # initial_x: the current depth of workpiece in z axis
    def safe_cut_along_z(self, x, z1, z2, initial_x):
        z_end, z_start = sorted([z1, z2])
        current_x = initial_x
        print "current_x = {}, current_x - x = {}".format(current_x, current_x - x)
        while current_x - x > self.safe_step:  # x is smaller then current_x
            current_x -= self.safe_step
            print "start cutting on x = {}".format(current_x)
            self._cut_along_z(current_x, z_start, z_end, initial_x)
            self.update_workpiece(current_x, z_end, z_start, finished_arg=False)
        print "start cutting on x = {}".format(x)
        self._cut_along_z(x, z_start, z_end, initial_x)
        self.update_workpiece(x, z_end, z_start, finished_arg=False)
        self.move_absolute(z=z_start, speed=600, wait=True, short=False)
        time.sleep(0.5)
        self.move_absolute(x=initial_x, speed=600, wait=True, short=False)

    # WARNING: Do not use this function without using safe_cut_along_z
    def _cut_along_z(self, x, z_start, z_end, initial_x):
        print "cut along z, on x = {}".format(x)
        self.move_absolute(z=z_start, speed=600, wait=True, short=False)
        time.sleep(0.3)
        self.move_absolute(x=x, speed=50, wait=True, short=False)
        time.sleep(0.3)
        self.move_absolute(z=z_end, speed=70, wait=True, short=False)
        time.sleep(0.3)
        self.move_absolute(x=initial_x + 3, speed=600, wait=True, short=False)
        time.sleep(0.3)

    # WARNING: this function assumes valid input and that it is used
    # in context (draw_from_coords). any other usage for this function
    # may be unsafe
    def _cut_shape(self, x_arr, z_arr):

        # reverse arrays if needed,
        # for tool to cut to the right direction (right to left)
        if z_arr[0] < z_arr[-1]:
            z_arr = z_arr[::-1]
            x_arr = x_arr[::-1]
        finished = True
        # the initial edge coordinates
        init_edge = self.coords_offset[0] + self.WORKPIECE_SIZE[0]
        self.move_absolute(z=z_arr[0], speed=600, wait=True, short=False)
        time.sleep(0.01)
        self.move_absolute(x=init_edge, speed=600, wait=True, short=False)
        for point_index, cur_z in enumerate(z_arr):
            print "finished all: {}".format(finished)
            # cur_x is where the workpiece edge currently stands (at z = cur_z)
            cur_x = self.workpiece[cur_z]["x"]
            delta_x = cur_x - x_arr[point_index]
            print "delta_x: {}".format(delta_x)
            if delta_x < 0.001:
                self.update_workpiece(x_arr[point_index], cur_z, finished_arg=True)

            if self.can_skip(z_arr, point_index):
                print "skipping ..."
                self.move_absolute(x=init_edge, speed=600, wait=True, short=True)
                time.sleep(0.01)
                self.move_absolute(z=cur_z, speed=100, wait=True, short=True)
                # time.sleep(0.01)
                # self.move_absolute(x=cur_x, speed=100, wait=True, short=True)
                continue

            # if skipped, return in a strait line to the real edge of the workpiece
            if point_index > 0:
                prev_x = self.workpiece[z_arr[point_index-1]]["x"]
                if self.x_pos != prev_x:
                    self.move_absolute(x=prev_x, speed=70, wait=True, short=False)

            if delta_x > self.safe_step:
                # the cut is too big,
                # meaning another cycle should be executed
                finished = False
                x = cur_x - self.safe_step
                self.go_to_point((x, cur_z))
                self.update_workpiece(x, cur_z)
            else:
                # go to the desired x value
                x = x_arr[point_index]
                self.go_to_point((x, cur_z))
                self.update_workpiece(x, cur_z, finished_arg=True)
        self.move_absolute(x=init_edge, speed=600, wait=True, short=False)
        return finished

    # during turning: if the current and next point are finished,
    # the tool can move faster using move_absolute
    def can_skip(self, z_arr, point_index):
        if point_index == len(z_arr) - 1:
            return self.workpiece[z_arr[point_index]]["finished"]
            # return False

        cur_point, next_point = self.workpiece[z_arr[point_index]], \
                                self.workpiece[z_arr[point_index + 1]]
        print "cur_point: {}, next_point: {}".format(cur_point["finished"], next_point["finished"])
        return cur_point["finished"] and next_point["finished"]

    # workpiece model saves x value for each z in the shape's z_arr
    def create_workpiece(self, z_arr):
        # init x is the initial edge position
        init_values = [{"x": self.WORKPIECE_SIZE[0] + self.coords_offset[0], "finished": False} for i in z_arr]
        return dict(zip(z_arr, init_values))

    # after cutting, update the change in x value at the workpiece model
    # use z_max to update any z value in range(z, z_end)
    def update_workpiece(self, x, z, z_max=None, finished_arg=None):
        finished = finished_arg if finished_arg else self.workpiece[z]["finished"]
        if z_max is None:
            if z in self.workpiece:
                self.workpiece.update({z: {"x": x, "finished": finished}})
                print "updated workpiece: {}: {}".format(z, {"x": x, "finished": finished})
            else:
                print "Error: trying to update z value not in workpiece"
                return
        else:
            print "updated workpiece all keys with value {}".format({"x": x, "finished": finished})
            relevant_keys = dict(filter(lambda elem: z <= elem[0] <= z_max, self.workpiece.items())).keys()
            self.workpiece.update(dict(zip(relevant_keys, [{"x": x, "finished": finished} for i in relevant_keys])))

    ###############################
    # info functions
    ###############################

    # on a separate thread, update the robot's state
    def get_info(self):
        def callback():
            while True:
                msg_to_send = utils.get_periodic_info_msg(self)
                # send and receive message from info socket
                info = self._info_sock.send_msg(msg_to_send, debug=False, decode=False)
                self.parse_message(info)
                self.check_position()
                self.check_emergency()
                # self.check_tool_change()

        t = threading.Thread(target=callback)
        t.start()
        return True

    def get_status(self):
        if self.status is None:
            return {}
        else:
            return self.status

    def set_status_msg(self, msg, is_err):
        self.status_msg = [msg, is_err]
        print msg

    def clear_status_msg(self):
        self.status_msg = ["", True]
        return True

    # parse the info message received from robot periodically
    def parse_message(self, msg):
        self.x_pos, self.z_pos = utils.parse_position(msg)
        self.door_closed, self.chuck_closed = utils.parse_door_and_chuck_status_from_msg(msg)
        self.tool_index = utils.get_tool_index(msg)
        self.is_spindle_spinning = utils.is_spindle_spinning(msg)
        self.spindle_speed = utils.get_spindle_speed(msg)

        self.is_moving = msg[27] != "\x00"
        self.emergency_stop = utils.get_emergency_status(msg)

        def get_workpiece_to_send():
            z = sorted(self.workpiece)
            return {
                "z": z,
                "x": [self.workpiece[zi]["x"] for zi in z],
                "finished": [self.workpiece[zi]["finished"] for zi in z],
            }

        self.status.update(
            {
                "tool_index": self.tool_index,
                "spindle_speed": self.spindle_speed,
                "is_spindle_spinning": self.is_spindle_spinning,
                "x_pos": self.x_pos,
                "z_pos": self.z_pos,
                "door_closed": self.door_closed,
                "chuck_closed": self.chuck_closed,
                "is_moving": self.is_moving,
                "is_homing_in_progress": self.is_homing_in_progress,
                "workpiece": get_workpiece_to_send(),
                "emergency_stop": self.emergency_stop,

                "status_msg": self.status_msg,
            })

        return True

    # validate that the tool's current position is in it's valid range
    def check_position(self):
        x_pos = self.status.get("pos_x")
        if not x_pos:
            return

        if x_pos > utils.max_x:
            self.set_status_msg("ERROR: x is too high!", True)
        if x_pos < utils.min_x:
            self.set_status_msg("ERROR: x is too low!", True)

    def check_tool_change(self):
        # change tool counter:
        # try spinning the tool turret for short pulses, to get accuracy
        # when the counter is zero the turret will spin (if needed) for a single pulse
        # see change_tool_request_to_send in utils file
        if self.change_tool_counter == 0:
            self.change_tool_counter = 0
        else:
            if self._desired_tool_index != self.tool_index:
                self.change_tool_counter -= 1

    def check_emergency(self):
        if self.emergency_stop:
            self.stop_all()


if __name__ == "__main__":
    benchturn_client = BenchTurnFunctions()
    time.sleep(5)
    benchturn_client.toggle_door()
    time.sleep(1)
    benchturn_client.toggle_chuck()

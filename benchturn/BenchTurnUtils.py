import math
import struct
import socket
import json

from time import sleep

# constants

data_offset = 2 * 0x2a  # 2 * 42, for debugging (header length)

tool_index_offset = 31  # the index in which spindle index described in msg


# last digit on byte 31 of message is:
# 1 for tool 1,
# 2 for tool 2,
# 4 for tool 3,
# 8 for tool 4,
def get_tool_index(msg):
    i = struct.unpack('b', msg[tool_index_offset])[0] % 16  # last digit
    if i == 0:
        return 0
    else:
        return int(math.log(i, 2)) + 1


max_x = 9
min_x = -60

# call before moving
reset_program_messages = [
    "\x03\x00\x03\x00\x15\x00\x64\x00\x00\x00",
    "\x03\x00\x03\x00\x2a\x00\x64\x00\x00\x00",
    "\x03\x00\x03\x00\x16\x00\x64\x00\x00\x00",
]

reset_registers_messages = [
    '03000000320003000000',
    '03000300190000000000',
    '030003001f0000000000',
    '030001000f0000000000',
    '030001000e0000000000',
    '00a555aa55aa55aa55aa',
    '012b55aa55aa55aa55aa',
    '020000003c0055aa55aa'
]

robot_ip = "192.168.0.10"
robot_control_port = 2500
robot_port = 5000


class Move:
    move_x_prefix = "\x03\x00\x01\x00\x00\x00"
    # move_x_2_prefix = "\x03\x00\x01\x00\x08\x00"
    move_z_prefix = "\x03\x00\x01\x00\x02\x00"
    # move_z_2_prefix = "\x03\x00\x01\x00\x0a\x00"
    move_y_prefix = "\x03\x00\x01\x00\x01\x00"


# prefix for changing the tool's feed speed
class SetSpeed:
    set_speed_x = ["\x03\x00\x08\x00\x6d\x00{}", "\x03\x00\x03\x00\x65\x00{}"]
    set_speed_y = ["\x03\x00\x08\x00\x81\x00{}", "\x03\x00\x03\x00\xc9\x00{}"]
    set_speed_z = ["\x03\x00\x08\x00\x95\x00{}", "\x03\x00\x03\x00\x2d\x01{}"]


class Spindle:
    speed_0_message = "\x03\x00\x03\x00\x1c\x00\x64\x00\x00\x00"
    change_speed_prefix = "\x03\x00\x03\x00\x1c\x00"
    start_spinning_first_message_cw = "\x03\x00\x01\x00\x0d\x00\x01\x00\x00\x00"
    start_spinning_first_message_ccw = "\x03\x7c\x01\x00\x0d\x00\xff\xff\xff\xff"
    start_spinning_last_messages = [
        "\x03\x00\x03\x00\x1f\x00\x00\x00\x00\x00",
        "\x03\x00\x01\x00\x0f\x00\x00\x00\x00\x00",
        "\x03\x00\x01\x00\x0e\x00\x00\x00\x00\x00"
    ]
    stop_spinning_message = "\x03\x46\x01\x00\x0d\x00\x00\x00\x00\x00"

    replace_spindle_messages = [
        "\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        "\x03\x00\x08\x00\x0f\x00\x00\x00\x00\x00",
        "\x03\x00\x00\x00\x33\x00\x00\x00\x00\x00",
        "\x03\x00\x00\x00\x32\x00\x01\x00\x00\x00",
    ]


homing_x = [
    "\x03\x2d\x01\x00\x0d\x00\x00\x00\x00\x00",
    "\x03\x2e\x03\x00\x15\x00\x64\x00\x00\x00",
    "\x02\x36\x00\x00\x0e\x00\x64\x00\x00\x00",
    "\x03\x37\x00\x00\x0b\x00\x01\x00\x00\x00",
]

homing_z = [
    "\x02\xf2\x00\x00\x10\x00\x01\x00\x00\x00",
    "\x03\xf3\x00\x00\x0d\x00\x01\x00\x00\x00",
]


class DoorStatus:
    OPEN = '\x00'
    CLOSE = '\x40'


class ChuckStatus:
    OPEN = '\x00'
    CLOSE = '\x40'


# utility functions

def get_socket(is_control):
    try:
        if is_control:
            # ctrl
            s = Sock(2500, (robot_ip, robot_control_port))
        else:
            # info
            s = Sock(5000, (robot_ip, robot_port))
    except Exception as e:
        print("Unable to open {} socket: {}".format("info" if is_control else "control", e))
        print("HINT: is CNCBase open?")
        print("Terminating...")
        raise e
    return s


# return the periodic "get info" message (as hex string)
def get_periodic_info_msg(obj):
    return "{}{}{}{}".format(obj.change_tool_request_to_send(), '\x02', '\x02', door_and_chuck_requests_to_send(obj))


# return the hex sting for the desired door and chuck states
def door_and_chuck_requests_to_send(obj):
    if obj.should_close_door and obj.should_close_chuck:
        return '\xc0'
    elif not obj.should_close_door and not obj.should_close_chuck:
        return '\x00'
    if obj.should_close_door:
        return '\x40'
    if obj.should_close_chuck:
        return '\x80'


# parse door and chuck status from info message
def parse_door_and_chuck_status_from_msg(msg):
    door_closed = msg[34] == '\x40' or msg[34] == '\xc0'
    chuck_closed = msg[34] == '\x80' or msg[34] == '\xc0'
    return door_closed, chuck_closed

# emergency button is on  if msg[28] is odd
def get_emergency_status(msg):
    bit = struct.unpack("b", msg[28])[0]
    return bit % 2 != 0
# wait for movement to finish
def wait_for_move_finish(client):
    sleep(1)
    while client.status.get("is_moving"):
        sleep(0.1)
    sleep(1)

def wait_for_short_move_finish(client):
    while client.status.get("is_moving"):
        sleep(0.1)


# convert hexadecimal 4 byte raw representation of integer
# to the (normalized) numeric value
def hex_to_float(hex_str, fmt='i'):
    return struct.unpack(fmt, hex_str)[0] / 10000.0


def parse_position(msg):
    x_pos = hex_to_float(msg[4:8])
    z_pos = hex_to_float(msg[12:16])
    return x_pos, z_pos


def is_spindle_spinning(msg):
    # return msg[32] != '\x00'
    return msg[64] != '\x0f'


# not working
def get_spindle_speed(msg):
    return struct.unpack('!i', msg[84:88])[0] / 10000.0


# sockets
# architecture is based on CNC_Driver
class Sock:
    def __init__(self, port, serv):
        # open UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.server_address = serv

    def close(self):
        self.sock.close()

    # send message via the socket and get reply
    def send_msg(self, msg, debug=False, decode=True):
        if decode:
            msg = msg.decode("hex")
        if debug:
            print "sending message {}".format([b for b in msg])
        self.sock.sendto(msg, self.server_address)
        data, server = self.sock.recvfrom(4096)
        return data


def spindle_speed_to_send(speed):
    return struct.pack('i', int(abs(speed)))


minimal_step = 0.01


def get_steps(x1, z1, x2, z2):
    delta_x, delta_z = (x2 - x1), (z2 - z1)
    if abs(delta_z) < 0.0002:
        return sign(delta_x) * minimal_step, 0
    if abs(delta_x) < 0.0002:
        return 0, sign(delta_z) * minimal_step
    ratio = abs(delta_x / delta_z)
    if ratio > 1:
        # step_x > step_z
        step_x = sign(delta_x) * minimal_step * ratio
        step_z = sign(delta_z) * minimal_step
    else:
        # step_z > step_x
        step_x = sign(delta_x) * minimal_step
        step_z = sign(delta_z) * minimal_step / ratio
    return round(step_x, 4), round(step_z, 4)


def dist(x1, z1, x2, z2):
    return math.sqrt((x1 - x2) ** 2 + (z1 - z2) ** 2)


def sign(x):
    if x == 0:
        return 0
    elif x > 0:
        return 1
    else:
        return -1


#######################
# convert wireshark JSON to hex data
# usage: send init messages
def json_to_hex_array(filename):
    obj = json.load(open('./res/{}.json'.format(filename)))
    res = []
    for package in obj:
        data = package["_source"]["layers"]["data"]["data.data"]
        data = data.encode("utf-8")
        data = data.replace(":", "")
        data = data.decode("hex")
        res.append(data)

    return res

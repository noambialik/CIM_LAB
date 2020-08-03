from time import sleep
import threading

from BenchTurnAPI import *


def get_benchturn_info(client):
    def callback():
        while True:
            if client.status:
                # print("tool_index: {}, z: {}, x = {}, is_moving: {}, is_spindle_spinning: {}\r".format
                #       (client.status.get("tool_index"), client.status.get("z_pos"),
                #        client.status.get("x_pos"),
                #        client.status.get("is_moving"),
                #        client.status.get(
                #            "is_spindle_spinning"))),
                # print client.status.get("workpiece")
                pass
            else:
                print("{}\r".format("status is None")),
            sleep(0.3)

    t = threading.Thread(target=callback)
    t.start()
    return True


def wait_for_client_status(client):
    if client.status is not None:
        return

    print("waiting for status to become not None ...")
    while client.status is None:
        sleep(0.1)


if __name__ == "__main__":
    client = Client()

    get_benchturn_info(client)

    wait_for_client_status(client)
    if not client.status.get("door_closed"):
        client.toggle_door()
        sleep(4)

    if not client.status.get("chuck_closed"):
        client.toggle_chuck()
        sleep(4)

    # client.move_absolute(x=0, speed=100)
    # sleep(3)
    # client.move_step(axis="z", distance=-10)

    # client.home()
    # sleep(3)
    z = -188.421 + 38
    client.move_absolute(z=z, speed=500)

    # client.start_spinning(1300)
    # sleep(5)
    # client.stop_spinning()

    # client.change_tool(1)
    # client.change_tool(3)
    # client.change_tool(4)
    # client.change_tool(2)
    # client.change_tool(4)

    def go(pos):
        client.go_to_point(pos)
        sleep(4)
        print client.status.get("z_pos"), client.status.get("x_pos")

    # go((-0.5, -0.25))
    # go((-1, -1))
    # go((0, 0))
    x_arr = [-44.0, -45.5806, -45.8675, -45.1527, -45.3808, -45.7255, -45.1773, -45.3248, -45.3307, -45.316,
             -45.3195, -45.3213, -45.3156, -45.3249,
             -45.3319, -45.2211, -45.0345, -45.8294, -45.6068, -45.3836, -45.2095, -45.1218, -45.1113,
             -45.1792]
    z_arr = [-177.0122, -176.0122, -175.0122, -174.0122, -173.0122, -172.0122, -171.0122, -170.0122, -169.0122,
             -168.0122, -167.0122, -166.0122, -165.0122, -164.0122,
             -163.0122, -162.0122, -161.0122, -160.0122, -159.0122, -158.0122, -157.0122, -156.0122, -155.0122,
             -154.0122]
    # client.draw_from_coords(x_arr, z_arr)

    # client.move_step(axis="z", distance=-1, speed=300, wait=True)
    # client.move_absolute(z=-100, speed=300, wait=True, short=False)

    print "Done"

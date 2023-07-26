from turtle import width
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import mpl_toolkits.mplot3d.art3d as art3d

import cv2
import numpy as np
#import tello
import time
import math
from djitellopy import Tello
# from pyimagesearch.pid import PID
import torch
import sys

tello = None
Go_=False
No_flying=False
is_flying = False
start_fly=False

def keyboard(self, key):
    global is_flying
    global Go_
    global start_fly
    print("key:", key)
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'):
        self.takeoff()
        is_flying = True
    if key == ord('2'):
        self.land()
        is_flying = False
    if key == ord('3'):
        self.send_rc_control(0, 0, 0, 0)
        is_flying = False
        start_fly=False
        print("stop!!!!")
    if key == ord('w'):
        Go_=True
        self.send_rc_control(0, fb_speed, 0, 0)
        print("forward!!!!")
    if key == ord('s'):
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        print("backward!!!!")
    if key == ord('a'):
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        print("left!!!!")
    if key == ord('d'):
        self.send_rc_control(lf_speed, 0, 0, 0)
        print("right!!!!")
    if key == ord('z'):
        self.send_rc_control(0, 0,(-1)* ud_speed, 0)
        print("down!!!!")
    if key == ord('x'):
        self.send_rc_control(0, 0, ud_speed, 0)
        print("up!!!!")
    if key == ord('c'):
        self.send_rc_control(0, 0, 0, degree)
        print("rotate!!!!")
    if key == ord('v'):
        self.send_rc_control(0, 0, 0, (-1) *degree)
        print("counter rotate!!!!")
    if key == ord('5'):
        height = self.get_height()
        if height>5:
            is_flying=True
        print("height: ",height)
    if key == ord('6'):
        battery = self.get_battery()
        print ("battery: ",battery)

class T(tuple):
    def __add__(self, other):
        return T(x + y for x, y in zip(self, other))
def main():
    # Tello
    drone = Tello()
    drone.connect()
    #time.sleep(10)
    drone.streamon()
    vid = cv2.VideoWriter('record/drone_{}.MP4'.format(time.strftime("%m_%d_%H_%M_%S",time.localtime())), cv2.VideoWriter_fourcc(*'mp4v'),24, (960, 720))
    vid_L = cv2.VideoWriter('record/drone_{}L.MP4'.format(time.strftime("%m_%d_%H_%M_%S",time.localtime())), cv2.VideoWriter_fourcc(*'mp4v'), 24, (960, 720))
    while True:
        frame = drone.get_frame_read().frame
        width=frame.shape[1]
        height=frame.shape[0]
        vid.write(frame)
        result_frame=frame.copy()
        w=width
        h=height
        offset = T((0, 0))
        # 22.5 degree
        # Ly = int(round(np.tan(22.5 * np.pi / 180.0) * w / 2))
        # cv2.line(result_frame, T((0, h // 2 - Ly - h // 8)) + offset,
        #             T((w - 1, h // 2 + Ly - 1 - h // 8)) + offset, (128, 128, 128), 2)
        # cv2.line(result_frame, T((0, h // 2 - Ly)) + offset,
        #             T((w - 1, h // 2 + Ly - 1)) + offset, (128, 128, 128), 2)
        # cv2.line(result_frame, T((0, h // 2 - Ly + h // 8)) + offset,
        #             T((w - 1, h // 2 + Ly - 1 + h // 8)) + offset, (128, 128, 128), 2)

        # 157.5 degree
        # Ly = int(round(np.tan(22.5 * np.pi / 180.0) * w / 2))
        # cv2.line(result_frame, T((0, h // 2 + Ly - h // 8)) + offset,
        #             T((w - 1, h // 2 - Ly - 1 - h // 8)) + offset, (128, 128, 128), 2)
        # cv2.line(result_frame, T((0, h // 2 + Ly)) + offset,
        #             T((w - 1, h // 2 - Ly - 1)) + offset, (128, 128, 128), 2)
        # cv2.line(result_frame, T((0, h // 2 + Ly + h // 8)) + offset,
        #             T((w - 1, h // 2 - Ly - 1 + h // 8)) + offset, (128, 128, 128), 2)

        # 67.5 degree
        # Lx = int(round(np.tan(22.5 * np.pi / 180.0) * h / 2))
        # cv2.line(result_frame, T((w // 2 - Lx - w // 12, 0)) + offset,
        #             T((w // 2 + Lx - w // 12, h - 1)) + offset, (230, 224, 176), 2)
        # cv2.line(result_frame, T((w // 2 - Lx, 0)) + offset,
        #             T((w // 2 + Lx, h - 1)) + offset, (230, 224, 176), 2)
        # cv2.line(result_frame, T((w // 2 - Lx + w // 12, 0)) + offset,
        #             T((w // 2 + Lx + w // 12, h - 1)) + offset, (230, 224, 176), 2)

        # 112.5 degree
        # Lx = int(round(np.tan(22.5 * np.pi / 180.0) * h / 2))
        # cv2.line(result_frame, T((w // 2 + Lx - w // 12, 0)) + offset,
        #             T((w // 2 - Lx - w // 12, h - 1)) + offset, (230, 224, 176), 2)
        # cv2.line(result_frame, T((w // 2 + Lx, 0)) + offset,
        #             T((w // 2 - Lx, h - 1)) + offset, (230, 224, 176), 2)
        # cv2.line(result_frame, T((w // 2 + Lx + w // 12, 0)) + offset,
        #             T((w // 2 - Lx + w // 12, h - 1)) + offset, (230, 224, 176), 2)

        # 45 degree
        # cv2.line(result_frame, T((w // 2 - h // 2 - w // 12, 0)) + offset,
        #             T((w // 2 - 1 + h // 2 - w // 12, h - 1)) + offset, (0, 0, 255), 2)
        # cv2.line(result_frame, T((w // 2 - h // 2, 0)) + offset, T((w // 2 - 1 + h // 2, h - 1)) + offset, (0, 0, 255), 2)
        # cv2.line(result_frame, T((w // 2 - h // 2 + w // 12, 0)) + offset,
        #             T((w // 2 - 1 + h // 2 + w // 12, h - 1)) + offset, (0, 0, 255), 2)

        # # 45 degree
        # cv2.line(result_frame, T((w // 2 - h // 2 - w // 12, 0)),
        #          T((w // 2 - 1 + h // 2 - w // 12, h - 1)), (0, 255, 0), 2)
        # cv2.line(result_frame, T((w // 2 - h // 2, 0)), T((w // 2 - 1 + h // 2, h - 1)), (0, 255, 0), 2)
        # cv2.line(result_frame, T((w // 2 - h // 2 + w // 12, 0)),
        #          T((w // 2 - 1 + h // 2 + w // 12, h - 1)), (0, 255, 0), 2)

        # 135 degree
        # cv2.line(result_frame, T((w // 2 + h // 2 - 2 - w // 12, 0)) + offset,
        #             T((w // 2 - 1 - h // 2 - w // 12, h - 1)) + offset, (0, 0, 255), 2)
        # cv2.line(result_frame, T((w // 2 + h // 2 - 2, 0)) + offset, T((w // 2 - 1 - h // 2, h - 1)) + offset, (0, 0, 255),
        #             2)
        # cv2.line(result_frame, T((w // 2 + h // 2 - 2 + w // 12, 0)) + offset,
        #             T((w // 2 - 1 - h // 2 + w // 12, h - 1)) + offset, (0, 0, 255), 2)
        # # # 135 degree
        # cv2.line(result_frame, T((w // 2 + h // 2 - 2 - w // 12, 0)),
        #          T((w // 2 - 1 - h // 2 - w // 12, h - 1)), (0, 255, 0), 2)
        # cv2.line(result_frame, T((w // 2 + h // 2 - 2, 0)), T((w // 2 - 1 - h // 2, h - 1)), (0, 255, 0),
        #          2)
        # cv2.line(result_frame, T((w // 2 + h // 2 - 2 + w // 12, 0)),
        #          T((w // 2 - 1 - h // 2 + w // 12, h - 1)), (0, 255, 0), 2)

        # 180 degree
        # cv2.line(result_frame, T((0, h // 2 - h // 8 - 1)) + offset, T((w - 1, h // 2 - h // 8 - 1)) + offset, (0, 0, 255),
        #             2)
        # cv2.line(result_frame, T((0, h // 2 - 1)) + offset, T((w - 1, h // 2 - 1)) + offset, (0, 0, 255), 2)
        # cv2.line(result_frame, T((0, h // 2 + h // 8 - 1)) + offset, T((w - 1, h // 2 + h // 8 - 1)) + offset, (0, 0, 255),
        #             2)
        # # 180 degree
        # cv2.line(result_frame, T((0, h // 2 - h // 8 - 1)), T((w - 1, h // 2 - h // 8 - 1)), (0, 255, 0),
        #          2)
        # cv2.line(result_frame, T((0, h // 2 - 1)), T((w - 1, h // 2 - 1)), (0, 255, 0), 2)
        # cv2.line(result_frame, T((0, h // 2 + h // 8 - 1)), T((w - 1, h // 2 + h // 8 - 1)), (0, 255, 0),
        #          2)

        # 90 degree
        # cv2.line(result_frame, T((w // 2 - w // 12 - 1, 0)) + offset, T((w // 2 - w // 12 - 1, h - 1)) + offset,
        #             (0, 0, 255), 2)
        # cv2.line(result_frame, T((w // 2 - 1, 0)) + offset, T((w // 2 - 1, h - 1)) + offset, (0, 0, 255), 2)
        # cv2.line(result_frame, T((w // 2 + w // 12 - 1, 0)) + offset, T((w // 2 + w // 12 - 1, h - 1)) + offset,
        #             (0, 0, 255), 2)
        # # # 90 degree
        # cv2.line(result_frame, T((w // 2 - w // 12 - 1, 0)), T((w // 2 - w // 12 - 1, h - 1)),
        #          (0, 255, 0), 2)
        # cv2.line(result_frame, T((w // 2 - 1, 0)), T((w // 2 - 1, h - 1)), (0, 255, 0), 2)
        # cv2.line(result_frame, T((w // 2 + w // 12 - 1, 0)), T((w // 2 + w // 12 - 1, h - 1)),
        #          (0, 255, 0), 2)
        cv2.imshow('framecopy' , result_frame)
        vid_L.write(result_frame)

        key = cv2.waitKey(1)
        if key != -1:
            keyboard(drone, key)
            if key==ord('3') or key==ord('2'):
                draw_path(np.array(coord_array))
                vid.release()
                vid_L.release()
                cv2.destroyAllWindows()
                return

if __name__ == '__main__':
    main()    
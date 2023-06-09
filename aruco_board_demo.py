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

def getCameraPitch(R):
    vector_x = np.mat([[1],[0],[0]])
    vector_z = np.mat([[0],[0],[1]])
    #print(vector_z.shape)
    new_z = -np.dot(R[0][0:3 , 0:3] , vector_z)
    #print(new_z)
    pitch = math.atan2(np.dot(new_z.T , vector_x) , np.dot(new_z.T , vector_z))
    pitch = pitch * 180 / np.pi
    return pitch

def set_distance(distance):
    if(distance > 0.1):
        new_distance = 0.1
        return new_distance
    else:
        return distance
def control_UVA(drone,df,width,height,coord=[0,0,-50]) :
    global is_flying
    max_speed_threshold = 40
    max_lf_threshold=20
    yaw_update=0
    y_update=0
    z_update=0
    x_update=0
    if len(df.index)>0:
        Go_ = True
        #print(df)

        if len(df[df['name']=='aruco_board'].index)>0:
            try:
                marker_center=((df[df['name']=='aruco_board']['xmin']+df[df['name']=='aruco_board']['xmax'])/2,\
                            (df[df['name']=='aruco_board']['ymin']+df[df['name']=='aruco_board']['ymax'])/2)
                boxheight=df[df['name']=='aruco_board']['ymax']-df[df['name']=='aruco_board']['ymin']
                if(type(boxheight) is not float):
                    boxheight=boxheight.values[0]
                    marker_center=(marker_center[0].values[0],marker_center[1].values[0])
            #print("marker_center",marker_center)
            #print("boxheight",boxheight)
            except:
                print("marker_center error\n",[df['name']=='aruco_board'])
                print("boxheight",boxheight)
                print(df)
                return 
            #print("marker_center:",marker_center)
        else:
            marker_center=((df.at[0,'xmax']+df.at[0,'xmin'])/2,\
                        (df.at[0,'ymax']+df.at[0,'ymin'])/2)
            boxheight=df.at[0,'ymax']-df.at[0,'ymin']
        if abs(coord[2])>4:
            if abs(marker_center[0] - width/2) > 20:                #旋轉
                # print(marker_center[0] - width/2)
                yaw_update = (marker_center[0] - width/2)//7
                # print("org_yaw: " + str(yaw_update))
                # print("new_yaw: " + str(yaw_update))
                # print("Turn left")
        else:
            x_update =coord[0]
            if x_update>0.6:
                x_update=-max_lf_threshold
            elif x_update<0:
                x_update=max_lf_threshold
            else:
                x_update=0
        if abs(marker_center[1] - height/2) > 30 and (boxheight>300 or abs(coord[2])<4):       #上下
            # print(marker_center[1] - height/2)
            y_update = (marker_center[1] - height/2)//-7
            # print(marker_center[1] - height/2 +100)
            # print("org_y: " + str(y_update))
            # print("new_y: " + str(y_update))
            # print("Turn left")
        elif x_update==0:
            z_update = 22-coord[2]
        # print("org_z: " + str(z_update))
            # z_update = z_pid.update(z_update, sleep=0)
        # print("pid_z: " + str(z_update))
        if z_update > max_speed_threshold:
            z_update = max_speed_threshold
        elif z_update < -max_speed_threshold:
            z_update = 0

        if is_flying:
            drone.send_rc_control(int(x_update), int(z_update)  ,int(y_update) ,int(yaw_update) )
            # print("1,rollLeft,0,0.5,0,0")
                # return "1,rollLeft,0,0.5,0,0"
    else:
        if is_flying:
            drone.send_rc_control(0, 0, 0, 0) 
    return x_update,z_update,y_update,yaw_update
def hovering_control(coord_array,scale):
    avg_dir=np.array([0,0,0],dtype=np.float64)
    for before,after in zip(coord_array[:-2],coord_array[1:]):
        avg_dir+=np.array(after)-np.array(before)
    avg_dir/=len(coord_array)-1
    x_update=-avg_dir[0]/abs(avg_dir[0])*scale/2
    y_update=avg_dir[1]/abs(avg_dir[1])*scale/2
    z_update=-avg_dir[2]/abs(avg_dir[2])*scale
    return x_update,y_update,z_update

def del_difftag(raw_corners,raw_ids):
    def removearray(L,arr):
        ind = 0
        size = len(L)
        while ind != size and not np.array_equal(L[ind],arr):
            ind += 1
        if ind != size:
            L.pop(ind)
        else:
            raise ValueError('array not found in list.')
    ids = []
    corners = []

    for c, i in zip(raw_corners, raw_ids):
        if (c[0][0] < c[0][2]).all() and i<9:
            ids.append(i)
            corners.append(c)
    if len(corners)>2:
        for c, i in zip(corners.copy(), ids.copy()):
            avg_corner=np.array(corners)[:,:,0,:].mean(axis=0)
            if (np.absolute(c[0][0]-avg_corner)>4*np.absolute(c[0][0]-c[0][2])).any():
                removearray(corners,c)
                ids.remove(i)
    return corners,ids
def pasteImg(frame,img,left,top):
    h=img.shape[0]+top
    for i in range(img.shape[1]):
        frame[top:h,left+i,0] = frame[top:h,left+i,0]*(1-img[:,i,3]/255) + img[:,i,0]*(img[:,i,3]/255)
        frame[top:h,left+i,1] = frame[top:h,left+i,1]*(1-img[:,i,3]/255) + img[:,i,1]*(img[:,i,3]/255)
        frame[top:h,left+i,2] = frame[top:h,left+i,2]*(1-img[:,i,3]/255) + img[:,i,2]*(img[:,i,3]/255)
    return frame
def draw_path(coords_3d):

    average_height = np.mean(coords_3d, axis = 0)[1]
    rec_w = 0.17
    fig = plt.figure(figsize=(30,8), dpi=200)
    
    # plot 3d (tag=9)
    ax3d_area = fig.add_subplot(1, 3, 1, projection='3d')
    ax3d_area.scatter(-coords_3d[:,0], -coords_3d[:,2], coords_3d[:,1], c="firebrick", marker='+', depthshade=0)
    rec = Rectangle((-rec_w/2, -rec_w/2), rec_w, rec_w, color="black")
    ax3d_area.add_patch(rec)
    art3d.pathpatch_2d_to_3d(rec, z=0, zdir="y")
    ax3d_area.tick_params(axis='x', labelsize=20)
    ax3d_area.tick_params(axis='y', labelsize=20)
    ax3d_area.tick_params(axis='z', labelsize=20)
    ax3d_area.set_xlim(-7, 7)
    ax3d_area.set_ylim(-3, 11)
    ax3d_area.set_zlim(-7, 7)
    
    # plot 3d plat view (tag=9)
    ax3df_area = fig.add_subplot(1, 3, 2, projection='3d')
    ax3df_area.scatter(-coords_3d[:,0], -coords_3d[:,2], coords_3d[:,1], c="firebrick", marker='+', depthshade=0)
    rec = Rectangle((-rec_w/2, -rec_w/2), rec_w, rec_w, color="black")
    ax3df_area.add_patch(rec)
    art3d.pathpatch_2d_to_3d(rec, z=0, zdir="y")
    ax3df_area.tick_params(axis='x', labelsize=20)
    ax3df_area.tick_params(axis='y', labelsize=20)
    ax3df_area.tick_params(axis='z', labelsize=20)
    ax3df_area.view_init(elev=0, azim=-90)
    ax3df_area.set_xlim(-7, 7)
    ax3df_area.set_zlim(-7, 7)
    ax3df_area.dist = 7
    
    # plot 2d (tag=9)
    ax2d_area = fig.add_subplot(1, 3, 3)
    axtemp_area = ax2d_area.scatter(-coords_3d[:,0], -coords_3d[:,2], c="firebrick", marker='+')
    x, y = [-rec_w/2, rec_w/2], [0, 0]
    ax2d_area.plot(x, y, color="black", linewidth=5)
    ax2d_area.tick_params(axis='x', labelsize=20)
    ax2d_area.tick_params(axis='y', labelsize=20)
    ax2d_area.axis('equal')
    
   
    
    fig.tight_layout()
    plt.savefig('run/figures/drone_{}.png'.format(time.strftime("%m_%d_%H_%M_%S",time.localtime())))
    plt.close()

def main():
    # Tello
    model = torch.hub.load('yolov5', 'custom', path='./best/best.pt',source='local')
    model.iou = 0.3# 設定 IoU 門檻值
    model.conf = 0.7# 設定信心門檻值
    drone = Tello()
    drone.connect()
    #time.sleep(10)
    global No_flying
    global Go_
    global is_flying
    global start_fly
    x_update=y_update=z_update=yaw_update=0
    # # Get the parameters of camera calibration
    # fs = cv2.FileStorage("calibrateCamera.xml", cv2.FILE_STORAGE_READ)
    # intrinsic = fs.getNode("intrinsic").mat()
    # distortion = fs.getNode('distortion').mat()
    intrinsic = np.load("run/numpy/opencv_mtx.npy")
    distortion = np.load("run/numpy/opencv_dist.npy")


    # z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    # y_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    # yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.1)

    # yaw_pid.initialize()
    # z_pid.initialize()
    # y_pid.initialize()

    max_speed_threshold = 40
    max_lf_threshold=30
    # print(intrinsic)
    # print(distortion)
    # Aruco parameters
    refine_method='NONE'
    CornerRefineMethod = {'NONE': cv2.aruco.CORNER_REFINE_NONE,
                          'SUBPIX': cv2.aruco.CORNER_REFINE_SUBPIX,
                          'CONTOUR': cv2.aruco.CORNER_REFINE_CONTOUR,
                          'APRILTAG': cv2.aruco.CORNER_REFINE_APRILTAG}
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_params.cornerRefinementMethod = CornerRefineMethod[refine_method]
    aruco_board = cv2.aruco.GridBoard_create(3, 3, 0.16, 0.027, aruco_dict)
    img_up = cv2.imread("img/上.png",cv2.IMREAD_UNCHANGED)
    img_down = cv2.imread("img/下.png",cv2.IMREAD_UNCHANGED)
    img_right = cv2.imread("img/右.png",cv2.IMREAD_UNCHANGED)
    img_left = cv2.imread("img/左.png",cv2.IMREAD_UNCHANGED)
    img_go = cv2.imread("img/前.png",cv2.IMREAD_UNCHANGED)
    img_clockwise = cv2.imread("img/順.png",cv2.IMREAD_UNCHANGED)
    img_counterclock = cv2.imread("img/逆.png",cv2.IMREAD_UNCHANGED)
    img_up = cv2.resize(img_up, (30, 30))
    img_down = cv2.resize(img_down, (30, 30))
    img_right = cv2.resize(img_right, (30, 30))
    img_left = cv2.resize(img_left, (30, 30))
    img_go = cv2.resize(img_go, (30, 30))
    img_clockwise = cv2.resize(img_clockwise, (30, 30))
    img_counterclock = cv2.resize(img_counterclock, (30, 30))
    drone.streamon()
    vid = cv2.VideoWriter('record/drone_{}.MP4'.format(time.strftime("%m_%d_%H_%M_%S",time.localtime())), cv2.VideoWriter_fourcc(*'mp4v'),24, (960, 720))
    vid_L = cv2.VideoWriter('record/drone_{}L.MP4'.format(time.strftime("%m_%d_%H_%M_%S",time.localtime())), cv2.VideoWriter_fourcc(*'mp4v'), 24, (960, 720))
    count_frame=0
    start_fly=False
    coord_array = []
    scale=40
    while True:
        x_update=y_update=z_update=0
        xyz_text=""
        frame = drone.get_frame_read().frame
        if is_flying:
            start_fly=True

        if (not No_flying) and start_fly:
            count_frame+=1
            if count_frame%2!=0:
                is_flying=False
            else:
                is_flying=True
        Go_ = False
        # 影像來源
        # frame = cv2.imread('./beacon.mp4_20221006_211200.325.png')
        # print(frame.shape)
        width=frame.shape[1]
        height=frame.shape[0]
        im=frame[:,:,[2,0,1]]
        # brightness
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray_frame)    
        # if brightness<60:
        #     frame = cv2.bilateralFilter(frame, d = -1, sigmaColor = 15, sigmaSpace = 5) ## add 20221229 justin #加上這一行 Justin 20221229
        vid.write(frame)

        # 進行物件偵測
        results = model(im)
        # 顯示結果摘要
        #results.print()
        df=results.pandas().xyxy[0]

        result_frame=frame.copy()
        # IMU=drone.query_attitude()
        # print(IMU)
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        if len(markerCorners):
            corners,ids=del_difftag(markerCorners, markerIds)
            np_ids = np.array(ids)
            _, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, np_ids, aruco_board, intrinsic, distortion, None, None, False)
            #rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 18, intrinsic, distortion) 
            if rvec is not None:
                z_update = 0
                y_update = 0
                yaw_update = 0
                x_update=0 

                RotationMatrix = cv2.Rodrigues(rvec)
                rmat, _ = cv2.Rodrigues(rvec)
                coord = (rmat.T@-tvec).T
                #print("coord",coord)
                #tvec=tvec.reshape(-1,3)
                #print("tvec",tvec)
                yaw = getCameraPitch(RotationMatrix)
                if yaw<0:
                    yaw+=180
                else:
                    yaw-=180
                tan=(coord[0,0]-0.3)/coord[0,2]
                world_degree=math.degrees(math.atan(tan))
                yaw+=math.degrees(math.atan(tan))

                
                for (mark_corner, mark_id) in zip(corners, ids):
                    # always return in this order: topleft, topright, bottomright, bottomleft
                    aruco_corners = mark_corner[0].astype(np.int32)

                    # draw label outlines (and their id)
                    cv2.polylines(result_frame, [aruco_corners], isClosed=True, color=(0, 140, 255), thickness=7)
                    #label = "{}".format(mark_id[0])
                    #text_pos = np.mean(aruco_corners, axis=0).astype(np.int_)
                    #cv2.putText(frame, label, text_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, color=(0, 140, 255), thickness=3)
                
                # display distance and axis
                distance = np.sum(np.sqrt(tvec**2))

                cv2.putText(result_frame, "Distance: {:.2f}m".format(distance), np.array([20, height-20]), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                cv2.drawFrameAxes(result_frame, intrinsic, distortion, rvec, tvec, 0.2)
                coord_array.append([coord[0,0],coord[0,1],coord[0,2]])
                xyz_text = "x: " + str(round(coord[0,0],3)) + "  y: " + str(round(coord[0,1], 3)) + "  z: " + str(round(coord[0,2],3))+" world degree: "+str(round(world_degree,3))+" yaw: "+str(round(yaw,3))
                cv2.putText(result_frame , xyz_text, np.array([20, height-50]) , cv2.FONT_HERSHEY_SIMPLEX , 0.8 , (0,255,255) , 2 , cv2.LINE_AA)
                if abs(coord[0,0]-0.3)<0.3 and abs(coord[0,1]-0.3)<0.5 and coord[0,2]>-2:
                    print("...............hovering...............")
                    cv2.putText(result_frame, "hovering", np.array([20, height-180]), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                    x_update,y_update,z_update=hovering_control(coord_array[-5:],scale)
                    if scale>20:
                        scale-=0.2
                elif(abs(coord[0,2]) > 20):
                    #drone.send_rc_control(0, 0, 0, 0)
                    Go_ = True
                    z_update = -coord[0,2]+20 
                    #print("org_z: " + str(z_update))
                    # z_update = z_pid.update(z_update, sleep=0)
                    #print("pid_z: " + str(z_update))
                    if z_update > max_speed_threshold:
                        z_update = max_speed_threshold
                    elif z_update < -max_speed_threshold:
                        z_update = 0
                    # if is_flying:
                    #     drone.send_rc_control(0, int(z_update)  ,0 ,0 )
                #drone.send_rc_control(0, 0, 0, 0) 
                else:
                    rotate_delay=0
                    if(abs(coord[0,2]) > 4 and rotate_delay<=0):
                        rotate_delay-=1
                        #drone.send_rc_control(0, 0, 0, 0)
                        Go_ = True
                        z_update = 30-coord[0,2]
                        #print("org_z: " + str(z_update))
                        #z_update = z_pid.update(z_update, sleep=0)
                        #print("pid_z: " + str(z_update))
                        if z_update > max_speed_threshold:
                            z_update = max_speed_threshold
                        elif z_update < 0:
                            z_update = 0
                        #elif z_update < -max_speed_threshold:
                        #    z_update = -max_speed_threshold
                        #drone.send_rc_control(0, int(z_update)  ,0 ,0 )
                        if (abs(yaw) > 5) :
                            max_rotate_threshold=10
                            Go_ = True
                            if abs(world_degree)>30:
                                x_update=world_degree
                            else:
                                yaw_update = yaw
                                # print("org_yaw: " + str(yaw_update))
                                #yaw_update = yaw_pid.update(yaw, sleep=0)
                                if yaw_update > max_rotate_threshold:
                                    yaw_update = max_rotate_threshold
                                elif yaw_update < -max_rotate_threshold:
                                    yaw_update = -max_rotate_threshold
                            # print("new_yaw: " + str(yaw_update))
                            # print("Turn left")
                            #drone.send_rc_control(0, 0, 0, int(yaw_update))
                    else:
                        if abs(coord[0,2]) < 4:
                            rotate_delay-=1
                        else:
                            rotate_delay=300
                        Go_ = True
                        if (abs(yaw-world_degree) > 8) and abs(world_degree)<20 :
                            max_rotate_threshold=10
                            Go_ = True
                            yaw_update = yaw-world_degree
                            # print("org_yaw: " + str(yaw_update))
                            #yaw_update = yaw_pid.update(yaw, sleep=0)
                            if yaw_update > max_rotate_threshold:
                                yaw_update = max_rotate_threshold
                            elif yaw_update < -max_rotate_threshold:
                                yaw_update = -max_rotate_threshold
                        else:
                            x_update =coord[0,0]
                            if x_update>0.6:
                                x_update=-max_lf_threshold#*(abs(coord[0,2])/4)
                            elif x_update<0:
                                x_update=max_lf_threshold#*(abs(coord[0,2])/4)
                            else:
                                x_update=0
                                z_update =30-coord[0,2]
                                if z_update > max_speed_threshold:
                                    z_update = max_speed_threshold
                                elif z_update < 0:
                                    z_update = 0
                        
                    height_delay=0
                    if (abs(coord[0,1]-0.3)>0.3) and height_delay<=0:
                        height_delay-=1
                        Go_ = True
                        max_y_threshold=40
                        y_update = (coord[0,1]-0.3)*50
                        #print("org_y: " + str(y_update))
                        # y_update = y_pid.update(y_update, sleep=0)
                        if y_update > max_y_threshold:
                            y_update = max_y_threshold
                        elif y_update < -max_y_threshold:
                            y_update = -max_y_threshold
                        #print("pid_y: " + str(y_update))
                        #drone.send_rc_control(0, 0 ,int((y_update//2) * (-3))  ,0 )
                    elif abs(coord[0,1]-0.3)<0.3:
                        height_delay=150
                    else:
                        height_delay-=1
                try:
                    if is_flying:
                        drone.send_rc_control(int(x_update), int(z_update)  , int(y_update ) , int(yaw_update) )
                except:
                    print("ValueError:",x_update,z_update,y_update,yaw_update)
                #print("fb_speed: " + str( z_update ))
                #print("ud_speed: " + str( y_update ))
                #print("yaw_speed: " + str( yaw_update ))
        else:
            if(len(df.index)):
                Go_=True      
                if len(coord_array):
                    x_update,z_update,y_update,yaw_update=control_UVA(drone,df,width,height,coord_array[-1])
                else:
                    x_update,z_update,y_update,yaw_update=control_UVA(drone,df,width,height)

        for i in df.index:
            cv2.rectangle(result_frame, (int(df.at[i,'xmin']),int(df.at[i,'ymin'])), (int(df.at[i,'xmax']), int(df.at[i,'ymax'])), (0, 0 , 255), 3, cv2.LINE_AA)
        # try:
        #     IMU_text="pitch:"+str(IMU['pitch'])+" roll:"+str(IMU['roll'])+" yaw:"+str(IMU['yaw'])
        #     cv2.putText(result_frame , IMU_text, np.array([20, height-130]) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0,255,255) , 2 , cv2.LINE_AA)
        # except:
        #     print("IMU error")
        if z_update>0:
            result_frame=pasteImg(result_frame,img_go,20,height-150)
        if x_update<0:
            result_frame=pasteImg(result_frame,img_left,55,height-150)
        elif x_update>0:
            result_frame=pasteImg(result_frame,img_right,55,height-150)
        if yaw_update<0:
            result_frame=pasteImg(result_frame,img_counterclock,90,height-150)
        elif yaw_update>0:
            result_frame=pasteImg(result_frame,img_clockwise,90,height-150)
        if y_update>0:
            result_frame=pasteImg(result_frame,img_up,125,height-150)
        elif y_update<0:
            result_frame=pasteImg(result_frame,img_down,125,height-150)
        #cv2.putText(result_frame , str(time.time()), np.array([20, height-150]) , cv2.FONT_HERSHEY_SIMPLEX , 0.7 , (0,255,255) , 2 , cv2.LINE_AA)
        controll_text="brightness:"+str(round(brightness,2))+" lf_speed:" + str( round(x_update,2) )+" fb_speed:" + str(round(z_update,2)  )+" ud_speed:" + str( round(y_update,2) )+" yaw_speed:" + str( round(yaw_update,2) )
        cv2.putText(result_frame , controll_text, np.array([20, height-90]) , cv2.FONT_HERSHEY_SIMPLEX , 0.7 , (0,255,255) , 2 , cv2.LINE_AA)
        vid_L.write(result_frame)
        
        cv2.imshow('framecopy' , result_frame)
        flying_info="fb_speed:" + str(round(z_update,2)  )+" ud_speed:" + str( round(y_update,2) )+" yaw_speed:" + str( round(yaw_update,2) )+"\n"+xyz_text+"\n"+df.to_string()
        #sys.stdout.write("\x1b[2K\x1b[1A\x1b[2K"+"\x1b[1A\x1b[2K"+"\x1b[1A\x1b[2K"+"\r"+flying_info)
        # sys.stdout.write(flying_info)
        # sys.stdout.flush()
        print(flying_info)
        key = cv2.waitKey(1)

        if key != -1:
            keyboard(drone, key)
            if key==ord('3') or key==ord('2'):
                draw_path(np.array(coord_array))
                vid.release()
                vid_L.release()
                cv2.destroyAllWindows()
                return
            
        elif Go_ == False or not is_flying:
            pass
            #drone.send_rc_control(0, 0, 0, 0)  


if __name__ == '__main__':
    main()    
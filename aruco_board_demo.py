from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import mpl_toolkits.mplot3d.art3d as art3d
import math
import cv2
import numpy as np
import time
import math
from djitellopy import Tello
import torch
import sys
import os
os.environ["KMP_DUPLICATE_LIB_OK"]="True"#for intel bug
MARKERS_X=3   #x方向marker數量
MARKERS_Y=3   #y方向marker數量
MRRKER_SIZE=0.16#0.06#0.16 #marker尺寸(單位：公尺)
MARKER_SEPERATION=0.027#0.01#0.027 #marker間隔(單位：公尺)
ARUCO_MODE_MARKER_TH=3#ArUco定位MARKER數量門檻值
X_DRIFT_TH = 0.3 # 水平方向懸停飄移門檻值(單位：公尺)
Y_DRIFT_TH = 0.3 # 垂直方向懸停飄移門檻值 (單位：公尺)   
COORD_RECORD_TH=3  # 座標紀錄偵測最小Marker數
HOVERING_DISTANCE_TH = 2 #接近ArUco board 懸停門檻值(單位：公尺)
HALF_BOARD_SIZE = 0.3 #ArUco board 大小的一半
ONLY_FORWARD_DISTANCE_TH = 20 #接近至board前才調整姿態，否則僅前進
MAX_FB_SPEED = 40  #無人機前後飛的最高速度
MAX_IF_SPEED=20    #無人機左右飛的最高速度
MAX_UD_SPEED=40    #無人機上下飛的最高速度
MAX_ROTATE_SPEED=10 #無人機旋轉飛的最高速度
MAX_X_TH=10 #無人機飛行靠近座標太大門檻值(單位:公尺)
YAW_TH=5 #ArUco旋轉修正角度門檻值
WORLD_DEGREE_TH=20 #相對角度過大平飛門檻值
hovering_velocity_scale=20 #懸停時，為了調整速度的慣性，煞停的緩衝調整值  
HOVERING_VELOCITY_SCALE_TH = 10 #煞停的緩衝最小值
PIXEL_DISTANCE_TO_VELOCITY_SCALE=7
ROTATE_TH=20 #單位pixel
ROTATE_DEGREE_TH=8 #看向ArUco的角度門檻值(單位度)
FPS=30
YOLO_RESTORE_TH=10 #YOLO恢復位置紀錄座標數最小門檻值

UP_DOWN_TH=30 #單位pixel
BOXHIGHT_CONTROL_UPDOWN_TH=300#單位pixel
Z_SPEED_INITIAL=30
SIDE_FLIGHT_TH=4 #單位:公尺
tello = None
No_flying=True  #T:不飛行的模式
is_flying = False
start_fly=False
ARUCO_MAX_ID=9 #以ID判斷錯誤偵測
OUTLIER_MIN_NUM=3 #至少要有3個才有所謂離群值
CornerRefineMethod = {'NONE': cv2.aruco.CORNER_REFINE_NONE,
                        'SUBPIX': cv2.aruco.CORNER_REFINE_SUBPIX,
                        'CONTOUR': cv2.aruco.CORNER_REFINE_CONTOUR,
                        'APRILTAG': cv2.aruco.CORNER_REFINE_APRILTAG}
refine_method='NONE'  #角點細化方法
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params.cornerRefinementMethod = CornerRefineMethod[refine_method]
aruco_board = cv2.aruco.GridBoard_create(MARKERS_X, MARKERS_Y, MRRKER_SIZE, MARKER_SEPERATION, aruco_dict)

intrinsic = np.load("run/numpy/avg_mtx.npy")
distortion = np.load("run/numpy/opencv_dist.npy")
yolo2arUco=0   #frame count for delay between yolov5 and arUco
YOLO2ARUCO_DEGREE_TH=15 #角度大於定值延遲YOLO和ARUCO模式的切換

def keyboard(self, key):
    global is_flying
    global start_fly
    print("key:", key)
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'): #起飛
        self.takeoff()

    if key == ord('2'): #降落
        battery = self.get_battery()
        print ("battery: ",battery)
        self.land()
        is_flying = False
    if key == ord('3'): #懸停
        self.send_rc_control(0, 0, 0, 0)
        is_flying = False
        start_fly=False
        print("stop!!!!")
    if key == ord('w'): #前飛(速度)
        self.send_rc_control(0, fb_speed, 0, 0)
        print("forward!!!!")
    if key == ord('s'): #後退(速度)
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        print("backward!!!!")
    if key == ord('a'): #左飛(速度)
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        print("left!!!!")
    if key == ord('d'):#右飛(速度)
        self.send_rc_control(lf_speed, 0, 0, 0)
        print("right!!!!")
    if key == ord('z'):#下
        self.send_rc_control(0, 0,(-1)* ud_speed, 0)
        print("down!!!!")
    if key == ord('x'):#上
        self.send_rc_control(0, 0, ud_speed, 0)
        print("up!!!!")
    if key == ord('c'):#順時針旋轉
        self.send_rc_control(0, 0, 0, degree)
        print("rotate!!!!")
    if key == ord('v'):#逆時針旋轉
        self.send_rc_control(0, 0, 0, (-1) *degree)
        print("counter rotate!!!!")
    if key == ord('5'):#獲得高度 
        height = self.get_height()
        if height>5:
            is_flying=True #啟動自動飛行
        print("height: ",height)
    if key == ord('6'):#獲得電池量
        battery = self.get_battery()
        print ("battery: ",battery)
        
#從rotation matrix 獲得相機頃角，相對於AruCo board
def getCameraPitch(R):
    vector_x = np.mat([[1],[0],[0]])
    vector_z = np.mat([[0],[0],[1]])
    #print(vector_z.shape)
    new_z = -np.dot(R[0][0:3 , 0:3] , vector_z)
    #print(new_z)
    pitch = math.atan2(np.dot(new_z.T , vector_x) , np.dot(new_z.T , vector_z))
    pitch = pitch * 180 / np.pi
    return pitch

def control_by_YOLOv5(drone,df,width,height,coord=[0,0,-50]) : #for YOLOv5 navigation
    global is_flying
    global MAX_FB_SPEED 
    global MAX_IF_SPEED 
    global MAX_UD_SPEED
    yaw_update=y_update=z_update=x_update=0

    if len(df.index)>0:
        #print(df)

        if len(df[df['name']=='aruco_board'].index)>0:
            try:
                marker_center=((df[df['name']=='aruco_board']['xmin']+df[df['name']=='aruco_board']['xmax'])/2,\
                            (df[df['name']=='aruco_board']['ymin']+df[df['name']=='aruco_board']['ymax'])/2)
                boxheight=df[df['name']=='aruco_board']['ymax']-df[df['name']=='aruco_board']['ymin']
                if(type(boxheight) is not float):
                    boxheight=boxheight.values[0]
                    marker_center=(marker_center[0].values[0],marker_center[1].values[0])
            except:
                print("marker_center error\n",[df['name']=='aruco_board'])
                print("boxheight",boxheight)
                print(df)
                return 
        else:
            marker_center=((df.at[0,'xmax']+df.at[0,'xmin'])/2,\
                        (df.at[0,'ymax']+df.at[0,'ymin'])/2)
            boxheight=df.at[0,'ymax']-df.at[0,'ymin']
       
        #code4-1    
        if abs(marker_center[1] - height/2) > UP_DOWN_TH and (boxheight>BOXHIGHT_CONTROL_UPDOWN_TH or abs(coord[2])<SIDE_FLIGHT_TH):       #上下
            #code4-2
            y_update = (marker_center[1] - height/2)//-PIXEL_DISTANCE_TO_VELOCITY_SCALE
        elif x_update==0:
            z_update = Z_SPEED_INITIAL-coord[2]
        if z_update > MAX_FB_SPEED:
            z_update = MAX_FB_SPEED
        elif z_update < -MAX_FB_SPEED:
            z_update = 0
        #code4-3
        if abs(coord[2])>SIDE_FLIGHT_TH:
            if abs(marker_center[0] - width/2) > ROTATE_TH:                #旋轉
                #code4-4
                yaw_update = (marker_center[0] - width/2)//PIXEL_DISTANCE_TO_VELOCITY_SCALE
        else:
            #code4-5
            x_update =coord[0]
            if x_update>MAX_X_TH:
                x_update=-MAX_IF_SPEED
            elif x_update<MAX_X_TH:
                x_update=MAX_IF_SPEED
            else:
                x_update=0
            if marker_center[0]>width/2:
                x_update=MAX_IF_SPEED
            elif marker_center[0]<width/2:
                x_update=-MAX_IF_SPEED
            else:
                x_update=0

        if is_flying:
            drone.send_rc_control(int(x_update), int(z_update)  ,int(y_update) ,int(yaw_update) )
    else:
        if is_flying:
            pass
            #drone.send_rc_control(0, 0, 0, 0) 
    return x_update,z_update,y_update,yaw_update
    
def hovering_control(coord_array,scale):   #hovering
    avg_dir=np.array([0,0,0],dtype=np.float64)
    for before,after in zip(coord_array[:-2],coord_array[1:]):
        avg_dir+=np.array(after)-np.array(before)
    avg_dir/=len(coord_array)-1
    #-avg_dir/abs(avg_dir)用來判斷方向 Scale給予速度調整
    x_update=-avg_dir[0]/abs(avg_dir[0])*scale
    if abs(avg_dir[1])>0.2:
        y_update=avg_dir[1]/abs(avg_dir[1])*scale
    else:
        y_update=0
    z_update=-avg_dir[2]/abs(avg_dir[2])*scale
    return x_update,y_update,z_update
    
def removearray(L,arr):
    ind = 0
    size = len(L)
    while ind != size and not np.array_equal(L[ind],arr):
        ind += 1
    if ind != size:
        L.pop(ind)
    else:
        raise ValueError('array not found in list.')
        
def remove_ArUco_error_detection (raw_corners,raw_ids):  #排除誤偵測：刪掉ArUco ID不在0-8 , 目前找到的標記到board中心, 距離不在每個marker大小4倍 
    ids = []
    corners = []

    for c, i in zip(raw_corners, raw_ids):
        if (c[0][0] < c[0][2]).all() and i<ARUCO_MAX_ID:
            ids.append(i)
            corners.append(c)
    if len(corners)>=OUTLIER_MIN_NUM:   #另一個鬼影ArUco Marker(0-8)透過此方法去除
        for c, i in zip(corners.copy(), ids.copy()):
            avg_corner=np.array(corners)[:,:,0,:].mean(axis=0)
            if (np.absolute(c[0][0]-avg_corner)>4*np.absolute(c[0][0]-c[0][2])).any():
                removearray(corners,c) 
                ids.remove(i)
    return corners,ids

#在目前的 frame上貼上標籤    
def paste_indicator_on_img(current_frame,indicator_mark,left,top):
    h=indicator_mark.shape[0]+top
    for i in range(indicator_mark.shape[1]):
        current_frame[top:h,left+i,0] = current_frame[top:h,left+i,0]*(1-indicator_mark[:,i,3]/255) + indicator_mark[:,i,0]*(indicator_mark[:,i,3]/255)
        current_frame[top:h,left+i,1] = current_frame[top:h,left+i,1]*(1-indicator_mark[:,i,3]/255) + indicator_mark[:,i,1]*(indicator_mark[:,i,3]/255)
        current_frame[top:h,left+i,2] = current_frame[top:h,left+i,2]*(1-indicator_mark[:,i,3]/255) + indicator_mark[:,i,2]*(indicator_mark[:,i,3]/255)
    return current_frame

#判斷是否為rotation matrix
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
    
#將rotation matrix轉到三維Euler angles    
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])


 
def control_by_ArUco(result_frame,coord_array, markerCorners, markerIds):
    global MAX_FB_SPEED 
    global MAX_IF_SPEED 
    global hovering_velocity_scale
    global intrinsic
    global distortion
    global yolo2arUco
    frame_height=result_frame.shape[0]
    corners,ids=remove_ArUco_error_detection(markerCorners, markerIds)
    np_ids = np.array(ids)
    _, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, np_ids, aruco_board, intrinsic, distortion, None, None, False)
    if rvec is not None:
        z_update=y_update=yaw_update=x_update = 0

        RotationMatrix = cv2.Rodrigues(rvec)
        rmat, _ = cv2.Rodrigues(rvec)
        coord = (rmat.T@-tvec).T
        angle=rotationMatrixToEulerAngles(rmat)
        yaw = getCameraPitch(RotationMatrix)
        distance = np.sum(np.sqrt(tvec**2))

        if yaw<0:
            yaw+=180
        else:
            yaw-=180
        tan=(coord[0,0]-HALF_BOARD_SIZE)/coord[0,2]
        world_degree=math.degrees(math.atan(tan))
        yaw+=math.degrees(math.atan(tan))
        for (mark_corner, mark_id) in zip(corners, ids):
            aruco_corners = mark_corner[0].astype(np.int32)
            cv2.polylines(result_frame, [aruco_corners], isClosed=True, color=(0, 140, 255), thickness=7)
        cv2.drawFrameAxes(result_frame, intrinsic, distortion, rvec, tvec, 0.2)
        
        angle=[math.degrees(angle[0]),math.degrees(angle[1]),math.degrees(angle[2])]
        angle=np.round(np.array(angle),2)
        if abs(angle[1])>YOLO2ARUCO_DEGREE_TH and coord[0,2]>-SIDE_FLIGHT_TH:
            yolo2arUco=1         #近的時候角度過大先以YOLO偵測為主，因此延遲一秒
        if len(markerCorners)>COORD_RECORD_TH:
            coord_array.append([coord[0,0],coord[0,1],coord[0,2]])
        #ArUco Navigation
        if abs(coord[0,0] - HALF_BOARD_SIZE)< X_DRIFT_TH and abs(coord[0,1] - HALF_BOARD_SIZE)< Y_DRIFT_TH and coord[0,2]> -HOVERING_DISTANCE_TH:#滿足懸停條件
            #code6
            print("...............hovering...............")
            cv2.putText(result_frame, "hovering", np.array([20, frame_height-180]), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
            x_update,y_update,z_update=hovering_control(coord_array[-1:],hovering_velocity_scale)
            if hovering_velocity_scale > HOVERING_VELOCITY_SCALE_TH:
                hovering_velocity_scale-=0.1
        elif(abs(coord[0,2]) > ONLY_FORWARD_DISTANCE_TH): #太遠只前飛
            z_update = -coord[0,2]+ONLY_FORWARD_DISTANCE_TH 
            if z_update > MAX_FB_SPEED:
                z_update = MAX_FB_SPEED
            elif z_update < -MAX_FB_SPEED:
                z_update = 0
        else:            #透過ArUco Board導航
            #code2-1    
            if abs(coord[0,1]-HALF_BOARD_SIZE)>Y_DRIFT_TH:   #上下
                #code2-2
                if coord[0,1]-HALF_BOARD_SIZE > Y_DRIFT_TH:
                    y_update = MAX_UD_SPEED
                elif coord[0,1]-HALF_BOARD_SIZE < -Y_DRIFT_TH:
                    y_update = -MAX_UD_SPEED
            #code2-3
            if abs(coord[0,2]) > SIDE_FLIGHT_TH:        #旋轉
                z_update = Z_SPEED_INITIAL-coord[0,2]
                if z_update > MAX_FB_SPEED:
                    z_update = MAX_FB_SPEED
                elif z_update < 0:
                    z_update = 0
                #code2-5
                if (abs(yaw) > YAW_TH) :
                    if abs(world_degree)>WORLD_DEGREE_TH:
                        x_update=world_degree
                    else:
                        yaw_update = yaw
                        if yaw_update > MAX_ROTATE_SPEED:
                            yaw_update = MAX_ROTATE_SPEED
                        elif yaw_update < -MAX_ROTATE_SPEED:
                            yaw_update = -MAX_ROTATE_SPEED
            else:
                #code2-4
                if (abs(yaw-world_degree) > ROTATE_DEGREE_TH) and abs(world_degree)<WORLD_DEGREE_TH :
                    yaw_update = yaw-world_degree
                    if yaw_update > MAX_ROTATE_SPEED:
                        yaw_update = MAX_ROTATE_SPEED
                    elif yaw_update < -MAX_ROTATE_SPEED:
                        yaw_update = -MAX_ROTATE_SPEED
                else:
                    #code2-5
                    x_update =coord[0,0]
                    if x_update- HALF_BOARD_SIZE> X_DRIFT_TH:
                        x_update=-MAX_IF_SPEED
                    elif x_update- HALF_BOARD_SIZE< X_DRIFT_TH:
                        x_update=MAX_IF_SPEED
                    else:
                        #code2-6
                        x_update=0
                        z_update =Z_SPEED_INITIAL-coord[0,2]
                        if z_update > MAX_FB_SPEED:
                            z_update = MAX_FB_SPEED
                        elif z_update < 0:
                            z_update = 0
            

    return x_update,z_update,y_update,yaw_update,coord,angle,world_degree,yaw,distance,coord_array
def draw_result(result_frame,df, x_update,z_update,y_update,yaw_update,frame_height,brightness,angle=None,coord=None,world_degree=None,yaw=None,distance=None):
    img_up = cv2.resize(cv2.imread("img/上.png",cv2.IMREAD_UNCHANGED),(30, 30))
    img_down = cv2.resize(cv2.imread("img/下.png",cv2.IMREAD_UNCHANGED),(30, 30))
    img_right = cv2.resize(cv2.imread("img/右.png",cv2.IMREAD_UNCHANGED),(30, 30))
    img_left = cv2.resize(cv2.imread("img/左.png",cv2.IMREAD_UNCHANGED), (30, 30))
    img_go = cv2.resize(cv2.imread("img/前.png",cv2.IMREAD_UNCHANGED),(30, 30))
    img_clockwise = cv2.resize(cv2.imread("img/順.png",cv2.IMREAD_UNCHANGED),(30, 30))
    img_counterclock = cv2.resize(cv2.imread("img/逆.png",cv2.IMREAD_UNCHANGED),(30, 30))
    for i in df.index:
            cv2.rectangle(result_frame, (int(df.at[i,'xmin']),int(df.at[i,'ymin'])), (int(df.at[i,'xmax']), int(df.at[i,'ymax'])), (0, 0 , 255), 3, cv2.LINE_AA)
    if z_update>0:
        result_frame=paste_indicator_on_img(result_frame,img_go,20,frame_height-150)
    if x_update<0:
        result_frame=paste_indicator_on_img(result_frame,img_left,55,frame_height-150)
    elif x_update>0:
        result_frame=paste_indicator_on_img(result_frame,img_right,55,frame_height-150)
    if yaw_update<0:
        result_frame=paste_indicator_on_img(result_frame,img_counterclock,90,frame_height-150)
    elif yaw_update>0:
        result_frame=paste_indicator_on_img(result_frame,img_clockwise,90,frame_height-150)
    if y_update>0:
        result_frame=paste_indicator_on_img(result_frame,img_up,125,frame_height-150)
    elif y_update<0:
        result_frame=paste_indicator_on_img(result_frame,img_down,125,frame_height-150)
    if angle is not None:
        cv2.putText(result_frame, "Distance: {:.2f}m Angle x={},y={},z={}".format(distance,angle[0],angle[1],angle[2]), np.array([20, frame_height-20]), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
        xyz_text = "x: " + str(round(coord[0,0],3)) + "  y: " + str(round(coord[0,1], 3)) + "  z: " + str(round(coord[0,2],3))+" world degree: "+str(round(world_degree,3))+" yaw: "+str(round(yaw,3))
        cv2.putText(result_frame , xyz_text, np.array([20, frame_height-50]) , cv2.FONT_HERSHEY_SIMPLEX , 0.8 , (0,255,255) , 2 , cv2.LINE_AA)
    controll_text="brightness:"+str(round(brightness,2))+" lf_speed:" + str( round(x_update,2) )+" fb_speed:" + str(round(z_update,2)  )+" ud_speed:" + str( round(y_update,2) )+" yaw_speed:" + str( round(yaw_update,2) )
    cv2.putText(result_frame , controll_text, np.array([20, frame_height-90]) , cv2.FONT_HERSHEY_SIMPLEX , 0.7 , (0,255,255) , 2 , cv2.LINE_AA)
    
def main():
    model = torch.hub.load('yolov5', 'custom', path='./best/best.pt',source='local')
    model.iou = 0.3# 設定 IoU 門檻值
    model.conf = 0.7# 設定信心門檻值
    drone = Tello()
    drone.connect()
    global No_flying
    global is_flying
    global start_fly
    global yolo2arUco
    x_update=y_update=z_update=yaw_update=0
      

   
    drone.streamon()
    os.makedirs('record',mode=0o777,exist_ok=True)
    vid = cv2.VideoWriter('record/drone_{}.MP4'.format(time.strftime("%m_%d_%H_%M_%S",time.localtime())), cv2.VideoWriter_fourcc(*'mp4v'),24, (960, 720))
    vid_L = cv2.VideoWriter('record/drone_{}L.MP4'.format(time.strftime("%m_%d_%H_%M_%S",time.localtime())), cv2.VideoWriter_fourcc(*'mp4v'), 24, (960, 720))
    count_frame=0
    coord_array = []
    
    
    while True:
        x_update=y_update=z_update=0
        xyz_text=""
        frame = drone.get_frame_read().frame
        if is_flying:
            start_fly=True
        if (not No_flying) and start_fly:
            count_frame+=1
            if count_frame%1!=0: #2個frame給一次指令
                is_flying=False
            else:
                is_flying=True
        frame_width=frame.shape[1]
        frame_height=frame.shape[0]
        im=frame[:,:,[2,0,1]]
        # brightness
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray_frame)    
        vid.write(frame)
        # 進行物件偵測
        results = model(im)
        # 顯示結果摘要
        df=results.pandas().xyxy[0]
        result_frame=frame.copy()
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        #code1
        if len(markerCorners)>ARUCO_MODE_MARKER_TH and yolo2arUco==0:   #marker偵測數量>3時 
            #code2
            x_update,z_update,y_update,yaw_update,coord,angle,world_degree,yaw,distance,coord_array = control_by_ArUco(result_frame,coord_array, markerCorners, markerIds)
            
            draw_result(result_frame,df, x_update,z_update,y_update,yaw_update,frame_height,brightness,angle,coord,world_degree,yaw,distance)#畫圖
            try:
                if is_flying:
                    drone.send_rc_control(int(x_update), int(z_update)  , int(y_update ) , int(yaw_update) )
            except:
                print("ValueError:",x_update,z_update,y_update,yaw_update)  
        else: #YOLOv5 Navigation
            #code3
            if(len(df.index)):
                if len(coord_array)>YOLO_RESTORE_TH:  #近距離與ArUco相搭配
                    yolo2arUco+=1
                    #code5
                    x_update,z_update,y_update,yaw_update=control_by_YOLOv5(drone,df,frame_width,frame_height,np.array(coord_array[-10:]).mean(axis=0))
                    if yolo2arUco>FPS: #1s
                        yolo2arUco=0
                else:
                    #code4            #遠距離導引
                    x_update,z_update,y_update,yaw_update=control_by_YOLOv5(drone,df,frame_width,frame_height)
                #畫圖
                draw_result(result_frame,df, x_update,z_update,y_update,yaw_update,frame_height,brightness)


        
        vid_L.write(result_frame)
        cv2.imshow('framecopy' , result_frame)
        flying_info="fb_speed:" + str(round(z_update,2)  )+" ud_speed:" + str( round(y_update,2) )+" yaw_speed:" + str( round(yaw_update,2) )+"\n"+xyz_text+"\n"+df.to_string()
        print(flying_info)
        key = cv2.waitKey(1)

        if key != -1:
            keyboard(drone, key)
            if key==ord('3') or key==ord('2'):
                vid.release()
                vid_L.release()
                cv2.destroyAllWindows()
                return
            
        elif not is_flying:
            pass
            #drone.send_rc_control(0, 0, 0, 0)  


if __name__ == '__main__':
    main()    

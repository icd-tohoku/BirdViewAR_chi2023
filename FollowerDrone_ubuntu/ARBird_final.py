from __future__ import print_function
import cv2
import olympe
import time
from collections import OrderedDict
from socket import socket, AF_INET, SOCK_DGRAM
from olympe.messages import gimbal
from olympe.messages.ardrone3.Piloting import TakeOff,moveBy,moveTo, Landing, PCMD
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingState import (
    PositionChanged,
    AlertStateChanged,
    AltitudeChanged,
    NavigateHomeStateChanged,
    AttitudeChanged,
    SpeedChanged
)
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed,MaxPitchRollRotationSpeed
from olympe.messages.gimbal import attitude, set_target, set_max_speed
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
import math
import time
import os
import numpy as np
import threading
import sys
from geocalc import CalcGeography
import pygame
import csv
import datetime
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
import casadi
import subprocess
import queue
import shlex
import tempfile

HOST = ''
PORT = 3333

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

#for wi-fi connection
#DRONE_IP = "192.168.42.1"

#for sky controller
#DRONE_IP = "192.168.53.1"
DRONE_IP = os.environ.get("DRONE_IP", "192.168.53.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")
streamUrl = "rtsp:192.168.53.1/live"
#DRONE_IP = "10.202.0.1"

#bgrLow = np.array([0,0,190])
#bgrHigh = np.array([140,150,255])

bgrLow = np.array([40,30,195])
bgrHigh = np.array([115,135,255])
#bgrHigh = np.array([90,132,255])
#bgrLow = np.array([30,40,175])
#bgrHigh = np.array([110,155,255])

infoStr = ""

strdate = ""

#targetPosX = 667
#targetPosY = 447
targetPosX = 640
targetPosY = 480
pastErrX = 0
pastErrY = 0

class fpvDroneInfo:
    def __init__ (self):
        self.lat = 0.0
        self.lng = 0.0
        self.alt = 0.0
        self.vX = 0.0
        self.vY = 0.0
        self.vZ = 0.0
        self.rot = 0.0
        self.gnd = True

class arMode(Enum):
    Full = 0
    NoFPVFrame = 1
    CollisionOnly = 2

class StreamingExample:
    def __init__(self,m_drone):
        # Create the olympe.Drone object from its IP address
        self.drone = m_drone
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.tempd, "h264_stats.csv"), "w+")
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ["fps", "bitrate"]
        )
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.renderer = None

    def start(self):
        # Connect the the drone
        #self.drone.connect()

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.drone.streaming.set_output_files(
            video=os.path.join(self.tempd, "streaming.mp4"),
            metadata=os.path.join(self.tempd, "streaming_metadata.json"),
        )

        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.streaming.start()
        #self.renderer = PdrawRenderer(pdraw=self.drone.streaming)

    def stop(self):
        if self.renderer is not None:
            self.renderer.stop()
        # Properly stop the video stream and disconnect
        self.drone.streaming.stop()
        #self.drone.disconnect()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)
        

        #cv2.imshow("frame",cv2frame)
        
        #print(cv2frame.shape)

    def getCvMat(self):
        yuv_frame = self.frame_queue.get()
        cv2_cvt_color_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }[yuv_frame.format()]
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        return cv2frame

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["is_sync"]):
            while len(self.h264_frame_stats) > 0:
                start_ts, _ = self.h264_frame_stats[0]
                if (start_ts + 1e6) < frame_ts:
                    self.h264_frame_stats.pop(0)
                else:
                    break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = 8 * sum(map(lambda t: t[1], self.h264_frame_stats))
            self.h264_stats_writer.writerow({"fps": h264_fps, "bitrate": h264_bitrate})

    def show_yuv_frame(self, window_name, yuv_frame):
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()

        height, width = (  # noqa
            info["raw"]["frame"]["info"]["height"],
            info["raw"]["frame"]["info"]["width"],
        )

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        # import cv2
        # cv2_cvt_color_flag = {
        #     olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
        #     olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        # }[yuv_frame.format()]

    def fly(self):
        # Takeoff, fly, land, ...
        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=10, _policy="check_wait"
                    )
                )
            )
        ).wait()
        self.drone(MaxTilt(40)).wait().success()
        for i in range(3):
            print("Moving by ({}/3)...".format(i + 1))
            self.drone(moveBy(10, 0, 0, math.pi, _timeout=20)).wait().success()

        print("Landing...")
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
        print("Landed\n")

    def replay_with_vlc(self):
        # Replay this MP4 video file using VLC
        mp4_filepath = os.path.join(self.tempd, "streaming.mp4")
        subprocess.run(shlex.split(f"vlc --play-and-exit {mp4_filepath}"), check=True)

def clamp_stick(stickV):
    stickV = min(100,max(stickV,-100))
    return stickV

Ke = 0.52#0.6
Kd = 0.001*1.21
Ki = 0.01*1
Ke_v = 150 * 0.51
Kd_v = 0.3 #20
Ki_v = 2.0 #40

ctrCount = 0
pastXdist = 0
sumXdist = 0.0
sumYdist = 0.0
pastYdist = 0
pastXspeedErr = 0
pastYspeedErr = 0
sumXspeed = 0
sumYspeed = 0
pastUT=0

_arMode = arMode.Full

def calcStickValue(xdist, ydist,xvelo,yvelo):
    global pastXdist
    global pastYdist
    global sumXdist
    global sumYdist
    global pastXspeedErr
    global pastYspeedErr
    global sumXspeed
    global sumYspeed
    global pastUT
    UT = time.time()
    dTime = UT-pastUT
    sumXdist = sumXdist + xdist*dTime
    sumYdist = sumYdist + ydist*dTime
    targetVx = -1*(xdist * Ke + (xdist-pastXdist) * Kd + sumXdist * Ki)
    targetVy = -1*(ydist * Ke + (ydist-pastYdist) * Kd + sumYdist * Ki)
    xspeedErr = targetVx - xvelo
    yspeedErr = targetVy - yvelo
    sumXspeed = sumXspeed + xspeedErr*dTime
    sumYspeed = sumYspeed + yspeedErr*dTime
    xStick = clamp_stick(xspeedErr * Ke_v + (xspeedErr-pastXspeedErr) * Kd_v + sumXspeed * Ki_v)
    yStick = clamp_stick(yspeedErr * Ke_v + (yspeedErr-pastYspeedErr) * Kd_v+ sumYspeed * Ki_v)
    pastXdist = xdist
    pastYdist = ydist
    print("dtime:"+str(dTime))
    print("speedXerr:" + str(xspeedErr))
    pastXspeedErr = xspeedErr
    pastYspeedErr = yspeedErr
    pastUT = UT
    if(sumXspeed>20):
        sumXspeed=20
    elif(sumXspeed<-20):
        sumXspeed=-20

    if(sumYspeed>20):
        sumYspeed=20
    elif(sumYspeed<-20):
        sumYspeed=-20

    return xStick, yStick

#_fpvDroneInfo = fpvDroneInfo()
H_max = 707
W_max = 1268
VFOV = 38
HFOV = 68

def ClampCtrlValue(stick):
    if(math.fabs(stick)>2):
        return stick
    else:
        return 0

def GetDs4Value(drone,joy):
        a = ClampCtrlValue(int( joy.get_axis(0)*80 ))      # rotation
        b = ClampCtrlValue(int( joy.get_axis(1)*80 ))     # updown
        c = ClampCtrlValue(int( joy.get_axis(2)*80 ))      # side
        d = ClampCtrlValue(int( joy.get_axis(3)*80 ))   
        #print("Stick:"+str(a))
        drone(PCMD(1,c,d,a,b,timestampAndSeqNum=0))

def CalcTargerPos(Vx,Vy,fz):
    global pastXt
    global pastYt
    global pastZt
    global pastThetat
    global pastPhit
    global taskCnt
    

    targetX_r = pastXt
    targetY_r = pastYt
    targetAlt_r = pastZt
    targetRot_r = pastThetat
    targetPhi = math.degrees(pastPhit)
    
    return targetX_r,targetY_r,targetRot_r,targetAlt_r,targetPhi

def CalcErr(dx,dy,phi,dz,vx,vy,diffRot,fz):

    targetDx,targetDy,targetR,targetAl,targetPhi = CalcTargerPos(vx,vy,fz)

    distX = 0-dx
    distY = targetDy-dy

    return distX, distY, targetAl,targetR,targetPhi

def Deg2Rad(deg):
    rad = deg / 180 * math.pi

    return rad

def ExtractNums(msg):
    global _arMode
    m_fpvDroneInfo = fpvDroneInfo()
    alt_index = msg.find('alt:')
    lat_index = msg.find('lat:')
    lng_index = msg.find('lng:')
    lsX_index = msg.find('lsX:')
    lsY_index = msg.find('lsY:')
    rsX_index = msg.find('rsX:')
    rsY_index = msg.find('rsY:')
    vlX_index = msg.find('vlX:')
    vlY_index = msg.find('vlY:')
    vlZ_index = msg.find('vlZ:')
    rot_index = msg.find('rot:')
    gnd_index = msg.find('gnd:')
    arm_index = msg.find('arm:')
    m_fpvDroneInfo.lat = float(msg[lat_index+4:lng_index])
    m_fpvDroneInfo.lng = float(msg[lng_index+4:alt_index])
    m_fpvDroneInfo.alt = float(msg[alt_index+4:lsX_index])
    m_fpvDroneInfo.vX = float(msg[vlX_index+4:vlY_index])
    m_fpvDroneInfo.vY = float(msg[vlY_index+4:vlZ_index])
    m_fpvDroneInfo.vZ = float(msg[vlZ_index+4:rot_index])
    m_fpvDroneInfo.rot = float(msg[rot_index+4:gnd_index])
    m_fpvDroneInfo.gnd = bool(int(msg[gnd_index+4:arm_index]))
    _arMode = arMode(int(msg[arm_index+4:]))
    
    return m_fpvDroneInfo

def initDroneSettings():
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone(setPilotingSource(source="Controller")).wait()    
    #assert drone(TakeOff()).wait().success()
    #drone.start_piloting()
    drone(set_max_speed(
        gimbal_id=0,
        yaw=0.0,
        pitch=45.0,
        roll=45.0,
    )).wait()
    drone(MaxRotationSpeed(200))
    drone(MaxTilt(30))
    drone(set_target(
        gimbal_id=0,
        control_mode="position",
        yaw_frame_of_reference="none",   # None instead of absolute
        yaw=0.0,
        pitch_frame_of_reference="absolute",
        pitch=-45.0,
        roll_frame_of_reference="none",     # None instead of absolute
        roll=0.0,
        )).wait()
    
    '''
    assert drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()
    '''

    return drone

uds_datas = np.load('UndistortMtx.npz')
roi = uds_datas['ROI']
mapx = uds_datas['MapX']
mapy = uds_datas['MapY']

def GetDronePosImag(frame):
    frame = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)
    frame = frame[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]] 
    imgMask = cv2.inRange(frame,bgrLow,bgrHigh)
    rt, contours = cv2.findContours(imgMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE )
    cx = 0
    cy = 0
    if(contours is not None):
        M= cv2.moments(imgMask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #frame = cv2.circle(frame,(cx,cy), 63, (0,0,255), 1)
    #frame = cv2.circle(frame,(targetPosX,targetPosY), 10, (0,255,255), -1)
    #cv2.imshow('frame', frame)
    return cx,cy,frame

def UDPServerThread():
    print("server")
    global infoStr
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind((HOST,PORT))
    while True:
        msg, address = s.recvfrom(1024)
        #_fpvDroneInfo = ExtractNums(msg.decode('utf-8'))
        infoStr = msg.decode('utf-8')
        #print(infoStr)

def CheckKeyoardInput(k,drone,cx,cy,m_fpvDroneInfo):
    global targetPosX
    global targetPosY
    if(k != -1):
        print(k)

    if (k==ord('i')):
        enss = olympe.enums.ardrone3.Piloting.MoveTo_Orientation_mode(2)
        targetGeopos = CalcGeography.CalcGPSFromDist(m_fpvDroneInfo.lat,m_fpvDroneInfo.lng,-180+m_fpvDroneInfo.rot,1)
        degRot = Deg2Rad(m_fpvDroneInfo.rot)

        drone(moveTo(targetGeopos['lat'],targetGeopos['lon'],m_fpvDroneInfo.alt+2.0,enss,m_fpvDroneInfo.rot))
        #t = drone.get_state(PositionChanged)
        #tt = drone.get_state(AltitudeChanged)
        print(targetGeopos['lat'])
        drone_attitude = drone.get_state(AttitudeChanged)
        print(math.degrees(drone_attitude["yaw"]))

        print(m_fpvDroneInfo.rot)
        #print(math.degrees(degRot))
    elif (k==ord('g')):
        drone(gimbal.set_target(
        gimbal_id=0,
        control_mode="position",
        yaw_frame_of_reference="none",   # None instead of absolute
        yaw=0.0,
        pitch_frame_of_reference="absolute",
        pitch=-45.0,
        roll_frame_of_reference="none",     # None instead of absolute
        roll=0.0,
        )).wait()
        #drone(olympe.messages.gimbal.reset_orientation(gimbal_id=0)).wait()
    elif (k==ord('l')):
        drone(Landing())
    elif (k==ord('t')):
        drone(TakeOff())

rotKe = 45
rotKd = 70
altKe = 100
pastErrRot = 0
altKd = 0
pastErrAlt = 0
targetHeight=7
targetRot = 0
isTracing = False

def CalcStickValueHR(errAlt,errRot):
    global pastErrAlt
    global pastErrRot
    errDRot = errRot - pastErrRot
    rotStick = rotKe * errRot + errDRot * rotKd
    rotStick = clamp_stick(rotStick)
    altStick = altKe * errAlt
    altStick = clamp_stick(altStick)

    pastErrRot = errRot
    pastErrAlt = errAlt

    return altStick,rotStick

def ChangeGimbalRot(drone,targetPhi):
    drone(set_target(
        gimbal_id=0,
        control_mode="position",
        yaw_frame_of_reference="none",   # None instead of absolute
        yaw=0.0,
        pitch_frame_of_reference="absolute",
        pitch=targetPhi,
        roll_frame_of_reference="none",     # None instead of absolute
        roll=0.0,
    ))

def TraceFPVDrone(dx,dy,dz,diffRot,phi,fpvDroneInfo,drone):
    global pastErrX
    global pastErrY
    global targetHeight
    global targetRot
    global fname
    if(dx != 0 and dy != 0):
        if(targetPosX != 0 and targetPosY != 0):

            errX, errY,targetZ,targetR,targetPhi = CalcErr(dx,dy,phi,dz,fpvDroneInfo.vX,fpvDroneInfo.vY,diffRot,fpvDroneInfo.alt)

            drone_attitude = drone.get_state(AttitudeChanged)
            drone_speed = drone.get_state(SpeedChanged)
            modFpvVeloX = fpvDroneInfo.vX*math.cos(diffRot)-fpvDroneInfo.vY*math.sin(diffRot)
            modFpvVeloY = fpvDroneInfo.vY*math.cos(diffRot)+fpvDroneInfo.vX*math.sin(diffRot)
            orgSpeedX = -1*drone_speed["speedX"] * math.sin(drone_attitude["yaw"])+drone_speed["speedY"] * math.cos(drone_attitude["yaw"])- modFpvVeloX
            orgSpeedY = 1*drone_speed["speedY"] * math.sin(drone_attitude["yaw"])+drone_speed["speedX"] * math.cos(drone_attitude["yaw"])-modFpvVeloY
            stickX,stickY = calcStickValue(errX,errY,orgSpeedX,orgSpeedY)
            #errAlt=targetHeight-(drone_alt["altitude"]-fpvDroneInfo.alt)
            errAlt=targetZ-dz
            #errRot=targetRot-diffRot
            errRot=targetR-diffRot
            if(errRot>4.71239):
                errRot = errRot - 2*math.pi
            elif(errRot < -4.71239):
                errRot = errRot + 2*math.pi
            stickAlt,stickRot=CalcStickValueHR(errAlt,errRot)
  
            ChangeGimbalRot(drone,targetPhi-90)
            relAlt = dz
            #print("TargetR:" + str(targetR))
            #print("TargetPhi:" + str(targetPhi))
            #print("1stVx:" + str(fpvDroneInfo.vX))
            #print("1stVY:" + str(fpvDroneInfo.vY))
            fname = 'Log_'+strdate + '.csv'
            #print("stickX:" + str(stickY))
            dt_now = datetime.datetime.now()
            with open(fname, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([dt_now,relAlt,errX,errY, orgSpeedX, orgSpeedY,fpvDroneInfo.vX,fpvDroneInfo.vY])
            #drone(PCMD(1,int(stickX),int(stickY),0,0,1)) 
            drone(PCMD(1,int(stickX),int(1*stickY),int(stickRot),int(stickAlt),1))  
    else:
        drone(PCMD(1,0,0,0,0,1))

    
#Calc distance using proposed algorythm
def CalcDistFromImg(cx,cy,drone,fpvDroneInfo):
    drone_alt = drone.get_state(AltitudeChanged)
    drone_phi = drone.get_state(gimbal.attitude)
    camTilt = drone_phi[0]["pitch_absolute"]
    drone_attitude = drone.get_state(AttitudeChanged)
    diffRot = (drone_attitude["yaw"]-math.radians(fpvDroneInfo.rot))
    diffAlt = drone_alt["altitude"]-fpvDroneInfo.alt

    print(drone_attitude["yaw"])
    if(cx >=10 and cy >= 10):
        paramPhi = math.radians(-1 * camTilt)
        paramAR = diffAlt / math.cos(math.radians(90+camTilt-VFOV/2))
        paramH = paramAR * math.cos(math.radians(VFOV/2))
        paramA = paramAR * math.sin(math.radians(VFOV/2))
        #print(paramAR)

        y_ell = 2 * paramA * (cy/H_max - 0.5)
        x_ell = 2 * paramH * math.tan(math.radians(HFOV/2)) * (cx/W_max - 0.5)

        dy_m = (y_ell*(paramH + paramA*math.sin(paramPhi)*math.cos(paramPhi))-paramA*paramH*(math.cos(paramPhi))**2)/(y_ell*math.cos(paramPhi)+paramH*math.sin(paramPhi))
        dx_m = x_ell*(paramH*math.sin(paramPhi)+paramA*math.cos(paramPhi))/(y_ell*math.cos(paramPhi)+paramH*math.sin(paramPhi))

        dx = dx_m
        dy = -1*dy_m + paramH*math.cos(paramPhi)
        dz = diffAlt
        #dRot = 0
        dx_mod = dx*math.cos(-1*diffRot)-dy*math.sin(-1*diffRot)
        dy_mod = dy*math.cos(-1*diffRot)+dx*math.sin(-1*diffRot)
        phi = math.radians(90+camTilt)
        #print(dy_mod)
        #print(drone_attitude["yaw"])
        #print(cy)
    else:
        drone_pos = drone.get_state(PositionChanged)
        dist = CalcGeography.CalcDistfromGPS(fpvDroneInfo.lat,fpvDroneInfo.lng ,drone_pos["latitude"],drone_pos["longitude"])
        dy_mod = 1*dist["distance"]*math.cos(math.radians(dist["azimuth2"])-drone_attitude["yaw"])
        dx_mod = 1*dist["distance"]*math.sin(math.radians(dist["azimuth2"])-drone_attitude["yaw"])
        dz = diffAlt
        phi = math.radians(90+camTilt)

    return dx_mod,dy_mod,dz,diffRot,phi

def ChangeDist2ImgPix(dx_m,dy_m,paramA,paramH,paramPhi):
    global H_max
    global W_max
    x_ell = paramH * dx_m/(paramH+(paramA*math.sin(paramPhi)-dy_m)*math.cos(paramPhi))
    y_ell = paramH * (dy_m*math.sin(paramPhi)+paramA*math.cos(paramPhi)**2)/(paramH+(paramA*math.sin(paramPhi)-dy_m)*math.cos(paramPhi))
   
    ccy = int((y_ell /(2 * paramA) + 0.5)*H_max)
    ccx = int((x_ell/(2 * paramH * math.tan(math.radians(HFOV/2)))+0.5)*W_max)

    return ccx,ccy

g_Mat = cv2.cuda_GpuMat()

def gpuWarpPersepect(InputMat,M):

    g_Mat.upload(InputMat)
    try:

        g_Mat_out = cv2.cuda.warpPerspective(g_Mat,M,(W_max,H_max))
    except:
        return g_Mat
    return g_Mat_out

def distortCircle(dx,dy,dz,dRot,phi,drone,cx,cy,isGround):
    global W_max
    global H_max
    global _arMode
    base_img = np.zeros((H_max,W_max,3),np.uint8)
    drone_alt = drone.get_state(AltitudeChanged)
    if(isGround):
        diffAlt = drone_alt["altitude"]
    else:
        diffAlt = dz
    #diffAlt = 0.73
    #diffAlt = 1.13

    paramPhi = math.pi/2 - phi
    paramAR = diffAlt / math.cos(phi-math.radians(VFOV/2))
    paramH = paramAR * math.cos(math.radians(VFOV/2))
    paramA = paramAR * math.sin(math.radians(VFOV/2))

    coordArray = np.zeros((4,2),np.int16)
    range_r = 8
    for i in range(4):
            #print(i+j)
            #bias_dx= dx_m + 0.5 * math.sin(i*math.pi/2)
            #bias_dy= dy_m + 0.5 * math.cos(i*math.pi/2)
            dx_bias = dx + range_r * math.sin(i*math.pi/2)
            dy_bias = dy + range_r * math.cos(i*math.pi/2)
            #print(dx_bias)
            dx_mod = dx_bias*math.cos(dRot)-dy_bias*math.sin(dRot)
            dy_mod = dy_bias*math.cos(dRot)+dx_bias*math.sin(dRot)
            #print(dx_mod)
            dy_mod_m = -1*dy_mod + paramH*math.cos(paramPhi)

            coordArray[i,0],coordArray[i,1] = ChangeDist2ImgPix(dx_mod,dy_mod_m,paramA,paramH,paramPhi)
    #print(dx_m)

    #for j in range(4):
        #cv2.circle(base_img,coordArray[j],10,(255,255,0))

    dxc_mod = dx*math.cos(dRot)-dy*math.sin(dRot)
    dyc_mod = dy*math.cos(dRot)+dx*math.sin(dRot)

    dyc_mod_m = -1*dyc_mod + paramH*math.cos(paramPhi)
    dxc_mod_m = dxc_mod

    ccx,ccy = ChangeDist2ImgPix(dxc_mod_m,dyc_mod_m,paramA,paramH,paramPhi)
    ccx_n = 540
    ccy_n = 360
    #rad = abs(ccy-coordArray[2,1])
    rad = 100
    #print(rad)
    if(_arMode == arMode.CollisionOnly):
        #cv2.circle(base_img,(ccx_n,ccy_n),int(rad*5/8),(255,255,0))
        tt=2
    else:
        cv2.circle(base_img,(ccx_n,ccy_n),rad,(255,255,0),thickness=2) #no snow
        cv2.circle(base_img,(ccx_n,ccy_n),int(rad*5/8),(255,255,0),thickness=2) #no snow

        #cv2.circle(base_img,(ccx_n,ccy_n),rad,(0,0,255),thickness=2)
        #cv2.circle(base_img,(ccx_n,ccy_n),int(rad*5/8),(0,0,255),thickness=2)
    #cv2.line(base_img,(ccx,ccy),(cx,cy),(255,255,0),thickness=4)

    #cv2.circle(base_img,(ccx,ccy),rad,(0,0,255))
    #cv2.circle(base_img,(ccx,ccy),int(rad/2),(0,0,255))
    #cv2.line(base_img,(ccx,ccy),(cx,cy),(0,0,255),thickness=4)

    src_pts = np.array([[ccx_n, ccy_n+rad], [ccx_n+rad, ccy_n], [ccx_n, ccy_n-rad], [ccx_n-rad, ccy_n]], dtype=np.float32)
    dst_pts = np.array([coordArray[0], coordArray[1], coordArray[2], coordArray[3]], dtype=np.float32)

    mat = cv2.getPerspectiveTransform(src_pts, dst_pts)
    #perspective_img = cv2.warpPerspective(base_img, mat, (W_max, H_max))
    perspective_img = gpuWarpPersepect(base_img, mat)

    #cv2.imshow('img',perspective_img)

    #print(ccy)
    return perspective_img, ccx,ccy

HFOV_F = 65.5
#VFOV_F = 46.4 official

VFOV_F = 36.8

def DrawExtentionLine(frame,cx,cy,ccx,ccy):
    if(ccy != cy):
        X = -cy*(ccx-cx)/(ccy-cy)+cx
        frame = cv2.line(frame,(cx,cy),(int(X),0),(255,255,0),thickness=1)

    return frame

def DrawFPVFrame(cx,cy,dRot,dx,dy,dz,phi,s_fpvDroneInfo,frame):
    #s_fpvDroneInfo.alt = 0.25
    cambottom_dy = s_fpvDroneInfo.alt*math.tan(math.pi/2-math.radians(VFOV_F/2))+0.1 #dist to 1st drone
    cambottom_dx = cambottom_dy * math.tan(math.radians(HFOV_F/2)) #dist to 1st drone
    cambottom_dx1_mod = cambottom_dx*math.cos(dRot)-cambottom_dy*math.sin(dRot)
    cambottom_dy1_mod = cambottom_dy*math.cos(dRot)+cambottom_dx*math.sin(dRot)

    cambottom_dx2_mod = -1*cambottom_dx*math.cos(dRot)-cambottom_dy*math.sin(dRot)
    cambottom_dy2_mod = cambottom_dy*math.cos(dRot)-cambottom_dx*math.sin(dRot)

    diffAlt = dz + s_fpvDroneInfo.alt
    #diffAlt = 1.13
    print(dRot)

    paramPhi = math.pi/2 - phi
    paramAR = diffAlt / math.cos(phi-math.radians(VFOV/2))
    paramH = paramAR * math.cos(math.radians(VFOV/2))
    paramA = paramAR * math.sin(math.radians(VFOV/2))

    dx_mod = dx*math.cos(dRot)-dy*math.sin(dRot)
    dy_mod = dy*math.cos(dRot)+dx*math.sin(dRot)

    cambottom_dy1_mod_m = -1*(cambottom_dy1_mod - paramH*math.cos(paramPhi)+dy_mod)
    cambottom_dx1_mod_m = cambottom_dx1_mod +dx_mod
    
    cambottom_dy2_mod_m = -1*(cambottom_dy2_mod - paramH*math.cos(paramPhi)+dy_mod)
    cambottom_dx2_mod_m = cambottom_dx2_mod +dx_mod

    ccx1,ccy1 = ChangeDist2ImgPix(cambottom_dx1_mod_m,cambottom_dy1_mod_m,paramA,paramH,paramPhi)
    ccx2,ccy2 = ChangeDist2ImgPix(cambottom_dx2_mod_m,cambottom_dy2_mod_m,paramA,paramH,paramPhi)
    frame = cv2.circle(frame,(int(ccx1),int(ccy1)),3,(0,0,255),-1)
    paramPhi_h = math.pi/2 - phi
    paramAR_h = (diffAlt -2 * s_fpvDroneInfo.alt) / math.cos(phi-math.radians(VFOV/2))
    paramH_h = paramAR_h * math.cos(math.radians(VFOV/2))
    paramA_h = paramAR_h * math.sin(math.radians(VFOV/2))

    cambottom_dy3_mod_m = -1*(cambottom_dy1_mod - paramH_h*math.cos(paramPhi_h)+dy_mod)
    cambottom_dx3_mod_m = cambottom_dx1_mod +dx_mod

    cambottom_dy4_mod_m = -1*(cambottom_dy2_mod - paramH_h*math.cos(paramPhi_h)+dy_mod)
    cambottom_dx4_mod_m = cambottom_dx2_mod +dx_mod

    ccx3,ccy3 = ChangeDist2ImgPix(cambottom_dx3_mod_m,cambottom_dy3_mod_m,paramA_h,paramH_h,paramPhi_h)
    ccx4,ccy4 = ChangeDist2ImgPix(cambottom_dx4_mod_m,cambottom_dy4_mod_m,paramA_h,paramH_h,paramPhi_h)

    pts = np.array([[ccx1,ccy1],[ccx2,ccy2],[ccx4,ccy4],[ccx3,ccy3]])
    frame = cv2.polylines(frame,[pts],True,(255,255,0),thickness=4)

    frame = DrawExtentionLine(frame,cx,cy,ccx3,ccy3)
    frame = DrawExtentionLine(frame,cx,cy,ccx4,ccy4)

    return frame

g_overMat = cv2.cuda_GpuMat()

def gpuOverlay(InputMat,overlayGMat):
    
    g_overMat.upload(InputMat)
    
    resultgMat = cv2.cuda.max(g_overMat,overlayGMat) #no snow
    #resultgMat = cv2.cuda.subtract(g_overMat,overlayGMat)
    rst = resultgMat.download()
    return rst

def ShowAugumentedTPV(cx,cy,dRot,affCircleImg,frame,dx,dy,dz,phi,s_fpvDroneInfo,ccx,ccy):
    global _arMode
    global isTracing
    r= 60
    rot_bias = math.pi*5/6
    xbias = 40*math.cos(-1*dRot)-60*math.sin(-1*dRot)
    ybias = 40*math.sin(-1*dRot)+60*math.cos(-1*dRot)
    pts = np.array(((cx+60*math.sin(-1*dRot+rot_bias), cy-60*math.cos(-1*dRot+rot_bias)), (cx+60*math.sin(-1*dRot), cy-60*math.cos(-1*dRot)), (cx+60*math.sin(-1*dRot-rot_bias), cy-60*math.cos(-1*dRot-rot_bias))))
    pts = pts.astype(np.int32)
    #print(pts)
    if(_arMode != arMode.CollisionOnly):

        frame = cv2.polylines(frame, [pts], True, (0, 0, 255), thickness=2)
    if(_arMode == arMode.Full):
        frame = DrawFPVFrame(cx,cy,dRot,dx,dy,dz,phi,s_fpvDroneInfo,frame)
    #image = overlay(frame, affCircleImg)
    image = gpuOverlay(frame,affCircleImg)

    if(_arMode != arMode.CollisionOnly):
        cv2.line(image,(ccx,ccy),(cx,cy),(255,255,0),thickness=4)

    theta_vl = 34.8 * math.pi/180
    length_vl_1 = cy/math.sin(theta_vl-dRot)
    vl_x_1 = int(length_vl_1 * math.cos(theta_vl-dRot))
    length_vl_2 = cy/math.sin(-1*theta_vl-dRot)
    vl_x_2 = int(length_vl_2 * math.cos(-1*theta_vl-dRot))

    if(not isTracing):
        cv2.drawMarker(image, (1200, 50), (0, 255, 255), markerType=cv2.MARKER_DIAMOND, markerSize=30)

    #cv2.line(image,(cx+vl_x_1,0),(cx,cy),(0,0,255),thickness=2)
    #cv2.line(image,(cx+vl_x_2,0),(cx,cy),(0,0,255),thickness=2)

    #

    cv2.imshow('frame',image)
    return image

pastXt = 0.0
pastYt = 7.0
pastZt = 7.0
pastThetat = 0
pastPhit = math.pi/4

def OptimThread(vxf,vyf,zf):
    global pastXt
    global pastYt
    global pastZt
    global pastThetat
    global pastPhit

    opti = casadi.Opti()
 
    rad_svfov = 19*casadi.pi/180
    rad_shfov = 32.75*casadi.pi/180

    z_init = 7
    y_init = 7
    phis_init = casadi.pi/4

    xs = opti.variable()
    ys = opti.variable()
    zs = opti.variable()
    thetas = opti.variable()
    phis = opti.variable()

    obj = (xs-pastXt)**2 + (ys - pastYt)**2+(zs-pastZt)**2 + 0.01*(thetas)**2 + (zs-z_init)**2+0.5*(phis-phis_init)**2+ (ys-y_init)**2
    opti.minimize(obj)
    vyf1 = 0
    vyf2 = 0
    if(vyf>0):
        vyf1 = vyf 
    elif(vyf<0):
        vyf2 = -vyf
    
    opti.subject_to(thetas - casadi.pi/12<=0) #side rotation
    opti.subject_to(-casadi.pi/12 - thetas<=0) #side rotation

    #opti.subject_to(zs*casadi.tan(phis - rad_svfov)-ys+1+vyf2<=0) #for back range
    opti.subject_to(zs*casadi.tan(phis - rad_svfov)-ys+1.6*vyf2<=0) #for back range
    #opti.subject_to(1+vyf1 <=zs*casadi.tan(phis + rad_svfov)-ys) #for forward range
    opti.subject_to(1.6*vyf1 <=zs*casadi.tan(phis + rad_svfov)-ys) #for forward range

    opti.subject_to(phis - casadi.pi/4<=0) #for back
    opti.subject_to(phis>=0) #for back
    opti.subject_to(zs>=0)
    opti.subject_to(ys>=7)

    
    if(vxf > 0):
        opti.subject_to(((ys-zs*casadi.tan(phis-rad_svfov))*(1/casadi.cos(phis+rad_svfov)-1/casadi.cos(phis-rad_svfov))*casadi.tan(rad_shfov)+zs*(casadi.tan(phis+rad_svfov)-casadi.tan(phis-rad_svfov))/casadi.cos(phis-rad_svfov)*casadi.tan(rad_shfov))*(1+casadi.tan(thetas)/casadi.tan(casadi.pi/2-rad_shfov))>=(1.6*vxf+2)*(casadi.tan(phis+rad_svfov)-casadi.tan(phis-rad_svfov)))
    elif(vxf<0):
        print("cnnd")
        opti.subject_to(((ys-zs*casadi.tan(phis-rad_svfov))*(1/casadi.cos(phis+rad_svfov)-1/casadi.cos(phis-rad_svfov))*casadi.tan(rad_shfov)+zs*(casadi.tan(phis+rad_svfov)-casadi.tan(phis-rad_svfov))/casadi.cos(phis-rad_svfov)*casadi.tan(rad_shfov))*(1-casadi.tan(thetas)/casadi.tan(casadi.pi/2-rad_shfov))>=(-1.6*vxf+2)*(casadi.tan(phis+rad_svfov)-casadi.tan(phis-rad_svfov)))
    
    #opti.subject_to(zf*casadi.tan(casadi.pi/2-18.4*casadi.pi/180)/casadi.tan(phis+rad_svfov)+2*zf-zs+1.5<=0) # fpv frame
    opti.subject_to(zf*casadi.tan(casadi.pi/2-18.4*casadi.pi/180)+ys+1.5<=(zs-zf)*casadi.tan(phis+rad_svfov)) # new fpv frame
    opti.subject_to(xs==0)

    opti.subject_to((zs+zf)*casadi.tan(phis - rad_svfov)-ys<=-3) #for bottom point


    s_opts = {'print_level': 0}
    p_opts = {'print_time': False}
    opti.solver('ipopt',p_opts,s_opts) # Set optimization solver

    sol = opti.solve() # Optimize the position and angle

    #print(f'Evaluation function:{sol.value(obj)}')
    print(f'x: {sol.value(xs)}')
    print(f'y: {sol.value(ys)}')
    print(f'z: {sol.value(zs)}')
    print(f'theta: {sol.value(thetas)}')
    print(f'phi: {sol.value(phis)}')

    
    pastXt = sol.value(xs)
    pastYt = sol.value(ys)
    pastZt = sol.value(zs)
    pastThetat = sol.value(thetas)
    pastPhit = sol.value(phis)

taskCnt = 0

def main():
    print("main")
    global infoStr
    global pastUT
    global strdate
    global isTracing
    global taskCnt
    pygame.init()
    joy = pygame.joystick.Joystick(0)   # The number of joystick can be confirmed using jstest-gtk
    joy.init()

    threadServer = threading.Thread(target=UDPServerThread)
    threadServer.setDaemon(True)
    threadServer.start()
    drone = initDroneSettings()
    streaming_example = StreamingExample(drone)

    #cap = cv2.VideoCapture(streamUrl, cv2.CAP_FFMPEG)
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    streaming_example.start()
    isTracing = False
    #ret, frame = cap.read()
    _fpvDroneInfo = fpvDroneInfo()
    tpe = ThreadPoolExecutor(max_workers=3)
    fps = 30
    width = 1280
    height = 720
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    dt_today = datetime.date.today()
    dt_now = datetime.datetime.now()
    strdate = str(dt_today)+" "+str(dt_now.hour)+":"+str(dt_now.minute)+":"+str(dt_now.second)
    fname = 'mov_'+strdate + '.mp4'
    video = cv2.VideoWriter(fname, fourcc, fps, (width, height))
    pastUT = time.time()
    frame = streaming_example.getCvMat()
    while True:
        #Left:0 Top:0
        cx,cy,frame = GetDronePosImag(frame)
        #print(cy)
        pygame.event.pump()
        _state=drone.get_state(FlyingStateChanged)["state"]

            #TraceFPVDrone(cx,cy,_fpvDroneInfo,drone)
            #CalcPixce2Dist(cx,cy,45,0.7)
        
        if(infoStr != ""):
            _fpvDroneInfo = ExtractNums(infoStr)
            #print(str(_fpvDroneInfo.vX))#forward+ back-
            #print(str(_fpvDroneInfo.vY))#right + left-
            #print(str(_fpvDroneInfo.vZ))#
            print(_fpvDroneInfo.gnd)
            print(str(_fpvDroneInfo.rot))

        if(taskCnt%6==0):
            taskCnt = 0
            threadCalcTarget = threading.Thread(target=OptimThread,args=(_fpvDroneInfo.vX,_fpvDroneInfo.vY,_fpvDroneInfo.alt))
            threadCalcTarget.start()
        taskCnt = taskCnt + 1
        

        #affCircleImg = distortCircle(dx,dy,dz,dRot,phi,drone)
        
        #cv2.imshow('frame',frame)
        if(_state==FlyingStateChanged_State.flying or _state==FlyingStateChanged_State.hovering):
            dx,dy,dz,dRot,phi = CalcDistFromImg(cx,cy,drone,_fpvDroneInfo)
            future = tpe.submit(distortCircle,dx,dy,dz,dRot,phi,drone,cx,cy,_fpvDroneInfo.gnd)
            radd = math.degrees(dRot)
            print(radd)
            if(isTracing):
                TraceFPVDrone(dx,dy,dz,dRot,phi,_fpvDroneInfo,drone)
                affCircleImg,ccx,ccy = future.result()
                image = ShowAugumentedTPV(cx,cy,dRot,affCircleImg,frame,dx,dy,dz,phi,_fpvDroneInfo,ccx,ccy)
                video.write(image)
            else:
                
                GetDs4Value(drone,joy)
                affCircleImg,ccx,ccy = future.result()
                #video.write(frame)
                image = ShowAugumentedTPV(cx,cy,dRot,affCircleImg,frame,dx,dy,dz,phi,_fpvDroneInfo,ccx,ccy)
                
        else:
            #dx,dy,dz,dRot,phi = CalcDistFromImg(cx,cy,drone,_fpvDroneInfo)
            #future = tpe.submit(distortCircle,dx,dy,dz,dRot,phi,drone,cx,cy)
            drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))
            cv2.imshow('frame',frame)
            #affCircleImg = future.result()
            #image = ShowAugumentedTPV(cx,cy,dRot,affCircleImg,frame,dx,dy,dz,phi,_fpvDroneInfo)
            #TraceFPVDrone(dx,dy,dz,dRot,phi,_fpvDroneInfo,drone)

        #video.write(image)
        k = cv2.waitKey(1)
        #ret, frame = cap.read()
        frame = streaming_example.getCvMat()
        if(k == ord('q')):
            break
        elif(k == ord('s')):
            isTracing = not isTracing
            print(isTracing)
        elif(k==ord('j')):
            #pygame.init()
            joy = pygame.joystick.Joystick(0)
            #joy.init()
            #time.sleep(0.1)
        elif(k==ord('d')):
            isTracing = False
        else:
            #print(_fpvDroneInfo.lat)
            CheckKeyoardInput(k,drone,cx,cy,_fpvDroneInfo)

    #drone(PCMD(1,int(0),int(0),0,0,1))
    #cap.release()
    streaming_example.stop()
    cv2.destroyAllWindows()
    
    tpe.shutdown()
    #drone.stop_piloting()
    drone.disconnect()
    sys.exit()


if __name__ == "__main__":
    main()

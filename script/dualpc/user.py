#!/home/player/tsml/bin/hrpsysjy
import sys
import socket
import traceback
import math
import time
import java.lang.System

import rtm
import waitInput
import bodyinfo
import OpenHRP
import org.omg.CORBA.DoubleHolder


def init(nshC, rncC, nshR, rncR):
    print "creating components"
    createCompsRobot(nshR, rncR)
    createCompsConsole(nshC, rncC)
    global rtcList
    rtcList = rtcListR + rtcListC[1:]
    
    print "connecting components"
    connectComps()

    print "activating components"
    activateComps(rtcListR)
    activateComps(rtcListC)

    print "initialized successfully"

def activateComps(rtcList):
    rtm.serializeComponents(rtcList)
    for r in rtcList:
        #r.stop()
        r.start()

def deactivateComps():
    for r in rtcList[1:]:
        r.stop()

def initRTC(module, name, ms):
    ms.load(module)
    return ms.create(module, name)

def createCompsRobot(nshR, rncR):
    global ms, rh, servo, rtcListR
    ms = rtm.findRTCmanager(nshR, rncR)
    rh = rtm.findRTC(bodyinfo.modelName, rncR)
    servo = rtm.findRTC("creekPdServo0", rncR)

    if rh==None or servo==None:
        print "couldn't find ROBOT or SERVO"
        return
    rtcListR = [rh]

    global kf, seq, seq_svc, sh
    kf = initRTC("creekStateEstimator", "kf", ms)
    if kf != None:
        rtcListR.append(kf)

    seq = initRTC("creekSequencePlayer", "seq", ms)
    if seq != None:
        seq_svc = OpenHRP.creekSequencePlayerServiceHelper.narrow(seq.service("service0"))
        rtcListR.append(seq)

    sh  = initRTC("creekReferenceHolder", "holder", ms)
    if sh != None:
        rtcListR.append(sh)

    #return rtcList


def createCompsConsole(nshC, rncC):
    global msC, rhC, servoC, rtcListC
    msC = rtm.findRTCmanager(nshC, rncC)
    rhC = rtm.findRTC("JVRC-State", rncC)
    servoC = rtm.findRTC("creekRobotState0", rncC)

    if rhC == None:
        rtcListC = rtcListR
        servoC = initRTC("creekRobotState", "creekRobotState0", msC)
    else:
        rtcListC = [rhC]
        #rtcListC = rtcListR
        if servoC == None:
            return

    global camera, camera_svc
    camera = initRTC("creekCameraViewer", "camera", msC)
    if camera != None:
        camera_svc = OpenHRP.creekCameraViewerServiceHelper.narrow(camera.service("service0"))
        rtcListC.append(camera)

    #return rtcList


def connectComps():
    rtm.connectPorts(rh.port("q"),   sh.port("qCur"))
    rtm.connectPorts(sh.port("qOut"),   servo.port("qRef"))

    rtm.connectPorts(rh.port("gyrometer"), kf.port("rate"))
    rtm.connectPorts(rh.port("gsensor"),   kf.port("acc"))

    rtm.connectPorts(sh.port("qOut"),        seq.port("qInit"))
    rtm.connectPorts(sh.port("basePosOut"),  seq.port("basePosInit"))
    rtm.connectPorts(sh.port("baseRpyOut"),  seq.port("baseRpyInit"))
    rtm.connectPorts(sh.port("zmpRefOut"),   seq.port("zmpRefInit"))
    rtm.connectPorts(seq.port("qRef"),     sh.port("qIn"))
    rtm.connectPorts(seq.port("basePos"),  sh.port("basePosIn"))
    rtm.connectPorts(seq.port("baseRpy"),  sh.port("baseRpyIn"))
    rtm.connectPorts(seq.port("zmpRef"),   sh.port("zmpRefIn"))

    rtm.connectPorts(rh.port("q"),   servoC.port("qIn"))
    rtm.connectPorts(kf.port("rpy"),    servoC.port("baseRpy"))
    rtm.connectPorts(sh.port("basePosOut"),  servoC.port("basePos"))

    if camera != None:
        rtm.connectPorts(rh.port("rcamera"),    camera.port("rcamera"))
        rtm.connectPorts(rh.port("lcamera"),    camera.port("lcamera"))
        rtm.connectPorts(rh.port("rhcamera"),   camera.port("rhcamera"))
        rtm.connectPorts(rh.port("lhcamera"),   camera.port("lhcamera"))

        rtm.connectPorts(servoC.port("rcameraPose"),    camera.port("rcameraPose"))
        rtm.connectPorts(servoC.port("lcameraPose"),    camera.port("lcameraPose"))
        rtm.connectPorts(servoC.port("rhcameraPose"),   camera.port("rhcameraPose"))
        rtm.connectPorts(servoC.port("lhcameraPose"),   camera.port("lhcameraPose"))


def setupLogger():
    print "dummy"

def saveLog(fname='sample'):
    print 'saved'

def goInital():
    tm = bodyinfo.timeToInitialPose
    seq_svc.setJointAngles(bodyinfo.makeCommandPose(bodyinfo.initialPose), tm)
    seq_svc.setBasePos([0.0, 0.0, bodyinfo.initialWaistHeight], tm)
    seq_svc.setZmp([0.0, 0.0, -bodyinfo.initialWaistHeight], tm)
    seq_svc.waitInterpolation()

def goHalfSitting():
    tm = bodyinfo.timeToHalfsitPose
    seq_svc.setJointAngles(bodyinfo.makeCommandPose(bodyinfo.halfsitPose), tm)
    seq_svc.setBasePos(bodyinfo.halfsitBasePos, tm)
    seq_svc.setZmp([0.023, 0, -bodyinfo.halfsitBasePos[2] ], tm)
    seq_svc.waitInterpolation()

def searchOn():
    camera_svc.setSearchFlag("all")

def showQrDataList():
    camera_svc.show()

def neck():
    seq_svc.setJointAngle("NECK_Y", deg2rad(30), 3.0)

def deg2rad(deg):
    return deg*math.pi/180.0

def setJointAngle(x):
    itemlist = x.split()
    n = [ float(item) for item in itemlist[1:] ]
    seq_svc.setJointAngle(itemlist[0], deg2rad(n[0]), n[1])

if __name__ == '__main__' or __name__ == 'main':
    if len(sys.argv) > 1:
        robotHost = sys.argv[1]
    else:
        robotHost = None
    #init(robotHost)
    #setupLogger()
    #userTest()


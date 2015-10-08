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

import datetime


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

    global kf, seq, seq_svc, sh, wpg, wpg_svc, st, st_svc, arm, arm_svc
    kf = initRTC("creekStateEstimator", "kf", ms)
    if kf != None:
        rtcListR.append(kf)

    seq = initRTC("creekSequencePlayer", "seq", ms)
    if seq != None:
        seq_svc = OpenHRP.creekSequencePlayerServiceHelper.narrow(seq.service("service0"))
        rtcListR.append(seq)

    wpg = initRTC("sony", "wpg", ms)
    if wpg != None:
        wpg_svc = OpenHRP.sonyServiceHelper.narrow(wpg.service("service0"))
        rtcListR.append(wpg)

    arm = initRTC("creekArmControlCartesian", "arm", ms)
    if arm != None:
        arm_svc = OpenHRP.creekArmControlCartesianServiceHelper.narrow(arm.service("service0"))
        rtcListR.append(arm)

    sh  = initRTC("creekReferenceHolder", "holder", ms)
    if sh != None:
        rtcListR.append(sh)

    st= initRTC("Stabilizer","st", ms)
    if st != None:
        st_svc = OpenHRP.StabilizerServiceHelper.narrow(st.service("service0"))
        rtcListR.append(st)

    #return rtcList


def createCompsConsole(nshC, rncC):
    global msC, rhC, servoC, servoC_svc, rtcListC
    msC = rtm.findRTCmanager(nshC, rncC)
    rhC = rtm.findRTC("JVRC-State", rncC)
    servoC = rtm.findRTC("creekRobotState0", rncC)

    if rhC == None:
        rtcListC = rtcListR
        servoC = initRTC("creekRobotState", "creekRobotState0", msC)
        rtcListC.append(servoC)
    else:
        rtcListC = [rhC]
        if servoC == None:
            return

    servoC_svc = OpenHRP.creekRobotStateServiceHelper.narrow(servoC.service("service0"))

    global camera, camera_svc, stick, pcl, pcl_svc
    camera = None
    stick = None
    pcl = None
    

    camera = initRTC("creekCameraViewer", "camera", msC)
    if camera != None:
        camera_svc = OpenHRP.creekCameraViewerServiceHelper.narrow(camera.service("service0"))
        rtcListC.append(camera)

    stick = initRTC("GamepadRTC", "stick", msC)
    if stick != None:
        rtcListC.append(stick)

    pcl = initRTC("creekPointCloudViewer", "pcl", msC)
    if pcl != None:
        pcl_svc = OpenHRP.creekPointCloudViewerServiceHelper.narrow(pcl.service("service0"))
        rtcListC.append(pcl)

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

    rtm.connectPorts(sh.port("qOut"),        servoC.port("qRef"))
    rtm.connectPorts(sh.port("baseRpyOut"),  servoC.port("baseRpyRef"))
    rtm.connectPorts(sh.port("zmpRefOut"),   servoC.port("zmpRef"))
    rtm.connectPorts(rh.port("rfsensor"),    servoC.port("rfsensor"))
    rtm.connectPorts(rh.port("lfsensor"),    servoC.port("lfsensor"))
    rtm.connectPorts(rh.port("rhsensor"),    servoC.port("rhsensor"))
    rtm.connectPorts(rh.port("lhsensor"),    servoC.port("lhsensor"))


    if wpg != None:
        rtm.connectPorts(rh.port("rfsensor"),  wpg.port("rfsensor"))
        rtm.connectPorts(rh.port("lfsensor"),  wpg.port("lfsensor"))
        rtm.connectPorts(rh.port("rhsensor"),  wpg.port("rhsensor"))
        rtm.connectPorts(rh.port("lhsensor"),  wpg.port("lhsensor"))
        rtm.connectPorts(sh.port("qOut"),   wpg.port("mc"))
        rtm.connectPorts(wpg.port("refq"),  sh.port("qIn"))
        rtm.connectPorts(wpg.port("rzmp"), sh.port("zmpRefIn"))
        rtm.connectPorts(wpg.port("basePosOut"), sh.port("basePosIn"))
        rtm.connectPorts(wpg.port("baseRpyOut"), sh.port("baseRpyIn"))

    if st!= None:
        rtm.connectPorts(kf.port("rpy"), st.port("rpy"))
        rtm.connectPorts(rh.port("rfsensor"), st.port("forceR"))
        rtm.connectPorts(rh.port("lfsensor"), st.port("forceL"))
        rtm.connectPorts(rh.port("q"), st.port("qCurrent"))
        rtm.connectPorts(sh.port("qOut"), st.port("qRef"))
        rtm.connectPorts(sh.port("zmpRefOut"), st.port("zmpRef"))
        rtm.connectPorts(sh.port("basePosOut"), st.port("basePosIn"))
        rtm.connectPorts(sh.port("baseRpyOut"), st.port("baseRpyIn"))
        rtm.connectPorts(wpg.port("contactStates"), st.port("contactStates"))
        #rtm.connectPorts(wpg.port("localEEpos"), st.port("localEEpos"))
        rtm.connectPorts(st.port("q"),  servo.port("qRef"))
        #rtm.connectPorts(st.port("q"),  wpg.port("mc"))
    else:
        rtm.connectPorts(sh.port("qOut"),   servo.port("qRef"))

    if camera != None:
        rtm.connectPorts(rh.port("rcamera"),    camera.port("rcamera"))
        rtm.connectPorts(rh.port("lcamera"),    camera.port("lcamera"))
        rtm.connectPorts(rh.port("rhcamera"),   camera.port("rhcamera"))
        rtm.connectPorts(rh.port("lhcamera"),   camera.port("lhcamera"))

        rtm.connectPorts(servoC.port("rcameraPose"),    camera.port("rcameraPose"))
        rtm.connectPorts(servoC.port("lcameraPose"),    camera.port("lcameraPose"))
        rtm.connectPorts(servoC.port("rhcameraPose"),   camera.port("rhcameraPose"))
        rtm.connectPorts(servoC.port("lhcameraPose"),   camera.port("lhcameraPose"))

    if stick != None:
        rtm.connectPorts(stick.port("axes"), wpg.port("axes"))
        rtm.connectPorts(stick.port("buttons"), wpg.port("buttons"))
        rtm.connectPorts(stick.port("axes"), pcl.port("axes"))
        rtm.connectPorts(stick.port("buttons"), pcl.port("buttons"))
        
    if pcl != None:
        rtm.connectPorts(rh.port("ranger"), pcl.port("ranger"))
        rtm.connectPorts(rh.port("q"),   pcl.port("qCur"))
        rtm.connectPorts(kf.port("rpy"),    pcl.port("baseRpyAct"))
        #rtm.connectPorts(sh.port("baseRpyOut"), pcl.port("baseRpyAct"))
        rtm.connectPorts(sh.port("baseRpyOut"), pcl.port("baseRpy"))
        rtm.connectPorts(sh.port("basePosOut"), pcl.port("basePos"))
        rtm.connectPorts(pcl.port("baseRpyOut"), wpg.port("baseRpyInit"))
        rtm.connectPorts(pcl.port("basePosOut"), wpg.port("basePosInit"))
        rtm.connectPorts(pcl.port("baseRpyOut"), sh.port("baseRpyIn"))
        rtm.connectPorts(pcl.port("basePosOut"), sh.port("basePosIn"))

    if arm != None:
        rtm.connectPorts(sh.port("qOut"),        arm.port("qCur"))
        rtm.connectPorts(sh.port("basePosOut"),  arm.port("basePos"))
        rtm.connectPorts(sh.port("baseRpyOut"),  arm.port("baseRpy"))
        rtm.connectPorts(stick.port("axes"),     arm.port("axes"))
        rtm.connectPorts(stick.port("buttons"),  arm.port("buttons"))
        rtm.connectPorts(arm.port("qRef"),       sh.port("qIn"))


def setupLogger():
    print "dummy"

def saveLog(fname='sample'):
    print 'saved'

def goInital():
    tm = bodyinfo.timeToInitialPose
    seq_svc.setJointAngles(bodyinfo.makeCommandPose(bodyinfo.initialPose), tm)
    #seq_svc.setBasePos([0.0, 0.0, bodyinfo.initialWaistHeight], tm)
    #seq_svc.setZmp([0.0, 0.0, -bodyinfo.initialWaistHeight], tm)
    seq_svc.waitInterpolation()

def goHalfSitting():
    tm = bodyinfo.timeToHalfsitPose
    seq_svc.setJointAngles(bodyinfo.makeCommandPose(bodyinfo.halfsitPose), tm)
    #seq_svc.setBasePos(bodyinfo.halfsitBasePos, tm)
    #seq_svc.setZmp([0.023, 0, -bodyinfo.halfsitBasePos[2] ], tm)
    seq_svc.waitInterpolation()

def movePosRel(x):
    itemlist = x.split()
    n = [ float(item) for item in itemlist ]
    seq_svc.setBasePosRel(n, 3)

def searchOn():
    camera_svc.setSearchFlag("all")

def searchOff():
    camera_svc.setSearchFlag("none")

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

def wuHalfSitting():
    wpg_svc.testMove()

def wuStart():
    wpg_svc.start()

def wuStop():
    wpg_svc.stop()

def wuStepping():
    wpg_svc.stepping()

def wuOmniWalkSwitch():
    wpg_svc.omniWalkSwitch()

def stOn():
    st_svc.startStabilizer()

def stOff():
    st_svc.stopStabilizer()

def pclStart():
    wpg_svc.stop()
    time.sleep(1)

    pcl_svc.start()

    seq_svc.setJointAngle("NECK_P", deg2rad(-60), 6.0)
    seq_svc.waitInterpolation()
    
    seq_svc.setJointAngle("NECK_P", deg2rad(0), 2.0)
    seq_svc.waitInterpolation()

    seq_svc.setJointAngle("WAIST_P", deg2rad(40), 4.0)
    seq_svc.waitInterpolation()

    seq_svc.setJointAngle("NECK_P", deg2rad(40), 4.0)
    seq_svc.waitInterpolation()

    seq_svc.setJointAngle("NECK_P", deg2rad(0), 2.0)
    seq_svc.waitInterpolation()

    seq_svc.setJointAngle("WAIST_P", deg2rad(0), 2.0)
    seq_svc.waitInterpolation()


    pcl_svc.stop()

    time.sleep(1)
    #wpg_svc.start()

def pclCangeMode():
    pcl_svc.changeMode()

def pclDetectLandingPointR():
    p = pclGetLandingPointR()
    if pcl_svc.detectLandingPoint(p[0], p[1], p[5], 0):
        pf = pclGetLandingPointR()
        wpg_svc.setFootPosR( pf[0], pf[1], pf[2], pf[3], pf[4], pf[5] )

def pclDetectLandingPointL():
    p = pclGetLandingPointL()
    if pcl_svc.detectLandingPoint(p[0], p[1], p[5], 1):
        pf = pclGetLandingPointL()
        wpg_svc.setFootPosL( pf[0], pf[1], pf[2], pf[3], pf[4], pf[5] )

def pclGetLandingPointR():
    x = org.omg.CORBA.DoubleHolder()
    y = org.omg.CORBA.DoubleHolder()
    z = org.omg.CORBA.DoubleHolder()
    r = org.omg.CORBA.DoubleHolder()
    p = org.omg.CORBA.DoubleHolder()
    w = org.omg.CORBA.DoubleHolder()
    pcl_svc.getLandingPoint(x,y,z,r,p,w,0)

    ret = [x.value, y.value, z.value, r.value, p.value, w.value]
    print '(x,y,z,r,p,w) = '
    for v in ret:
        print '%6.3f,'%v,
    print '\n'
    return ret

def pclGetLandingPointL():
    x = org.omg.CORBA.DoubleHolder()
    y = org.omg.CORBA.DoubleHolder()
    z = org.omg.CORBA.DoubleHolder()
    r = org.omg.CORBA.DoubleHolder()
    p = org.omg.CORBA.DoubleHolder()
    w = org.omg.CORBA.DoubleHolder()
    pcl_svc.getLandingPoint(x,y,z,r,p,w,1)
    
    ret = [x.value, y.value, z.value, r.value, p.value, w.value]
    print '(x,y,z,r,p,w) = '
    for v in ret:
        print '%6.3f,'%v,
    print '\n'
    return ret

def cameraSave(x):
    path = "/home/player/tsml/log/"+x
    camera_svc.saveData(path)

def rarmActive():
    arm_svc.setArm(0)

def larmActive():
    arm_svc.setArm(1)

def setArmVelocity(x):
    itemlist = x.split()
    n = [ float(item) for item in itemlist ]
    arm_svc.setVelocity(n[0], n[1], n[2])

def gamepadWalk():
    pcl_svc.detectModeOff()
    arm_svc.setArm(2)
    time.sleep(1)
    wpg_svc.omniWalkSwitchOn()

def gamepadPcl():
    arm_svc.setArm(2)
    wpg_svc.omniWalkSwitchOff()
    time.sleep(1)
    pcl_svc.detectModeOn()

def gamepadArmR():
    pcl_svc.detectModeOff()
    wpg_svc.omniWalkSwitchOff()
    time.sleep(1)
    arm_svc.setArm(0)

def gamepadArmL():
    pcl_svc.detectModeOff()
    wpg_svc.omniWalkSwitchOff()
    time.sleep(1)
    arm_svc.setArm(1)


def setObjectV(x):
    itemlist = x.split()
    n = [ float(item) for item in itemlist ]
    wpg_svc.setObjectV(n[0], n[1], n[2], n[3], n[4], n[5])

def setObjectZero():
    wpg_svc.setObjectV(0,0,0,0,0,0)

def showRobotState():
    servoC_svc.showState()

def logRobotState():
    date=datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    servoC_svc.logStart(date)
    wpg_svc.logStart(date)

def testStair():
    wpg_svc.setFootPosR(  0.110, -0.543,  0.188, -0.000, -0.007,  1.549 )
    wpg_svc.setFootPosL( -0.093, -0.573,  0.187, -0.000, -0.007,  1.554 )

def pclSave(x):
    path = "/home/player/tsml/log/"+x
    pcl_svc.save(x)


if __name__ == '__main__' or __name__ == 'main':
    if len(sys.argv) > 1:
        robotHost = sys.argv[1]
    else:
        robotHost = None
    #init(robotHost)
    #setupLogger()
    #userTest()

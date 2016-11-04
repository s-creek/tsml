#!/opt/tsml/bin/hrpsysjy
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

def init(robotHost=None):
    if robotHost != None:
      print 'robot host = '+robotHost
      java.lang.System.setProperty('NS_OPT',
          '-ORBInitRef NameService=corbaloc:iiop:'+robotHost+':2809/NameService')
      rtm.initCORBA()

    print "creating components"
    rtcList = createComps(robotHost)

    print "connecting components"
    connectComps()

    print "activating components"
    activateComps(rtcList)

    print "initialized successfully"

def activateComps(rtcList):
    rtm.serializeComponents(rtcList)
    for r in rtcList:
        #r.stop()
        r.start()

def deactivateComps():
    for r in rtcList[1:]:
        r.stop()

def initRTC(module, name):
    ms.load(module)
    return ms.create(module, name)

def createComps(hostname=socket.gethostname()):
    global ms, rh, servo, rtcList
    ms = rtm.findRTCmanager(hostname)
    rh = rtm.findRTC(bodyinfo.modelName)
    servo = rtm.findRTC("creekPdServo0")

    if rh==None or servo==None:
        print "couldn't find ROBOT or SERVO"
        return
    rtcList = [rh]


    global kf, seq, seq_svc, sh, wpg, wpg_svc

    kf = initRTC("creekAttitudeEstimator", "kf")
    if kf != None:
        rtcList.append(kf)

    seq = initRTC("creekSequencePlayer", "seq")
    if seq != None:
        seq_svc = OpenHRP.creekSequencePlayerServiceHelper.narrow(seq.service("service0"))
        rtcList.append(seq)

    sh  = initRTC("creekReferenceHolder", "holder")
    if sh != None:
        rtcList.append(sh)

    wpg = initRTC("creekGaitGenerator", "wpg")
    if wpg != None:
        wpg_svc = OpenHRP.creekGaitGeneratorServiceHelper.narrow(wpg.service("service0"))
        rtcList.append(wpg)

    return rtcList

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

    rtm.connectPorts(sh.port("qOut"),        wpg.port("qInit"))
    rtm.connectPorts(sh.port("basePosOut"),  wpg.port("basePosInit"))
    rtm.connectPorts(sh.port("baseRpyOut"),  wpg.port("baseRpyInit"))
    rtm.connectPorts(sh.port("zmpRefOut"),   wpg.port("zmpRefInit"))
    rtm.connectPorts(wpg.port("qRef"),     sh.port("qIn"))
    rtm.connectPorts(wpg.port("basePos"),  sh.port("basePosIn"))
    rtm.connectPorts(wpg.port("baseRpy"),  sh.port("baseRpyIn"))
    rtm.connectPorts(wpg.port("zmpRef"),   sh.port("zmpRefIn"))


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

def deg2rad(deg):
    return deg*math.pi/180.0

def startStepping():
    wpg_svc.startStepping()

def stopStepping():
    wpg_svc.stopStepping()

def testWalk():
    wpg_svc.test()

if __name__ == '__main__' or __name__ == 'main':
    if len(sys.argv) > 1:
        robotHost = sys.argv[1]
    else:
        robotHost = None
    init(robotHost)
    #setupLogger()
    #userTest()

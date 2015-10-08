import os
from user import *

funcList = [
    ["stop-rt-system", deactivateComps],
    " ",
    "seq play",
    goInital,
    goHalfSitting,
    " ",
    ["move pos rel", movePosRel, "io", "0 0 -0.1"],
    " ",
    ["setJointAngle", setJointAngle, "io", "NECK_Y 30 3.0"],
    " ",
    "camera",
    searchOn,
    searchOff,
    showQrDataList,
    ["save data", cameraSave, "io", "D1_O1_01-2.csv"],
    "\n",
    "wu walk",
    wuHalfSitting,
    wuStart,
    wuStepping,
    #["setv5", setObjectV, "io", "5 0 0 0 0 0"],
    #setObjectZero,
    " ",
    wuStop,
    wuOmniWalkSwitch,
    " ",
    "st",
    stOn,
    stOff,
    " ",
    "pcl",
    pclStart,
    pclCangeMode,
    " ",
    pclDetectLandingPointR,
    pclDetectLandingPointL,
    " ",
    ["save pcl", pclSave, "io", "D1_3D_01.csv"],
    "\n",
    "arm",
    rarmActive,
    larmActive,
    ["set arm velocity", setArmVelocity, "io", "0.04 0.2 0.2"],
    " ",
    "gamepad",
    ["walk", gamepadWalk],
    ["PCL",  gamepadPcl],
    ["rarm", gamepadArmR],
    ["larm", gamepadArmL],
    " ",
    " ",
    showRobotState,
    ["log start", logRobotState],
    #" ",
    #testStair,
]
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
  neck,
  ["setJointAngle", setJointAngle, "io", "NECK_Y 30 3.0"],
  " ",
  "camera",
  searchOn,
  showQrDataList,
  ["save data", cameraSave, "io", "qrdata.csv"],
  "\n",
  "wu walk",
  wuHalfSitting,
  wuStart,
  wuStepping,
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
  "\n",
  "arm",
  rarmActive,
  larmActive,
]

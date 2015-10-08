import math

modelName = "JVRC-TSML"

timeToHalfsitPose = 5.0 # [sec]
halfsitPoseJVRC = [ -25.6, 0, 0, 57.8, 0, -32.2,
                     -25.6, 0, 0, 57.8, 0, -32.2,
                     0, 0, 0,
                     0, 0, 0,
                     10, -5, 0, -30, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     10,  5, 0, -30, 0, 0, 0,
                     0, 0, 0, 0, 0, 0 ]

squatPoseJVRC = [ -35.6, 0, 0, 77.8, 0, -42.2,
                   -35.6, 0, 0, 77.8, 0, -42.2,
                   0, 0, 0,
                   0, 0, 0,
                   30, -5, 0, -90, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   30,  5, 0, -90, 0, 0, 0,
                   0, 0, 0, 0, 0, 0 ]


halfsitPose=halfsitPoseJVRC
initialWaistHeight = 0.846
halfsitBasePos = [ -0.02, 0.0, 0.782861 ]

dof = len(halfsitPose)

timeToInitialPose = 5.0 # [sec]
initialPose = [0] * dof


# this is for compatibility between different robots
def makeCommandPose(pose):
    return [jv*math.pi/180.0 for jv in pose]

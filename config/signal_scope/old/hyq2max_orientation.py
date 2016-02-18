import transformations
import math

def getRoll(msg):
    rpy = transformations.euler_from_quaternion((msg.orientation[1],msg.orientation[2],msg.orientation[3],msg.orientation[0]))
    return msg.utime, 180*rpy[0]/math.pi

def getPitch(msg):
    rpy = transformations.euler_from_quaternion((msg.orientation[1],msg.orientation[2],msg.orientation[3],msg.orientation[0]))
    return msg.utime, 180*rpy[1]/math.pi

def getYaw(msg):
    rpy = transformations.euler_from_quaternion((msg.orientation[1],msg.orientation[2],msg.orientation[3],msg.orientation[0]))
    return msg.utime, 180*rpy[2]/math.pi

addSignalFunction('POSE_GROUND_TRUTH',getRoll)
addSignalFunction('POSE_BODY',getRoll)
addPlot()
addSignalFunction('POSE_GROUND_TRUTH',getPitch)
addSignalFunction('POSE_BODY',getPitch)
addPlot()
#addSignal('HYQ_GRF',msg.utime,msg.pos[2])
#addSignal('HYQ_GRF',msg.utime,msg.vel[2])
#addSignal('HYQ_GRF',msg.utime,msg.accel[2])
#addSignal('HYQ_GRF',msg.utime,msg.rotation_rate[2])
addSignalFunction('POSE_GROUND_TRUTH',getYaw)
addSignalFunction('POSE_BODY',getYaw)



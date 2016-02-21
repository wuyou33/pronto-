'''
Here is a basic example to plot a signal from an lcm message.
In this example, the channel is POSE_BODY.

The X coordinate is the message timestamp in microseconds,
and the Y value is pos[0], or the first value of the pos array.

Note, msg is a pre-defined variable that you must use in order
for this to work.  When you define a signal, the msg variable
is used to record the attribute lookups that are required to
extract the signal data from the lcm message in the future.
'''

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



addSignals('BTP_TRANSF', msg.utime, msg.pos,[0,1,2])
addSignals('BTP_TRANSF',msg.utime,msg.vel,[0,1,2])
addSignals('BTP_TRANSF',msg.utime,msg.rotation_rate,[0,1,2])

addPlot()
addSignalFunction('BTP_TRANSF',getRoll)
addSignalFunction('BTP_TRANSF',getPitch)
addSignalFunction('BTP_TRANSF',getYaw)

addPlot()
addSignals('BTT_TRANSF', msg.utime, msg.pos,[0,1,2])

addPlot()
addSignalFunction('BTT_TRANSF',getRoll)
addSignalFunction('BTT_TRANSF',getPitch)
addSignalFunction('BTT_TRANSF',getYaw)


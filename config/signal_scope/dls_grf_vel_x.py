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

addSignal('HYQ_GRF', msg.utime, msg.LF[2])
addSignal('HYQ_GRF', msg.utime, msg.RF[2])
addSignal('HYQ_GRF', msg.utime, msg.LH[2])
addSignal('HYQ_GRF', msg.utime, msg.RH[2])
addSignal('HYQ_GRF_DES', msg.utime, msg.LF[2])
addSignal('HYQ_GRF_DES', msg.utime, msg.RF[2])
addSignal('HYQ_GRF_DES', msg.utime, msg.LH[2])
addSignal('HYQ_GRF_DES', msg.utime, msg.RH[2])

#addPlot()
#addSignal('HYQ_STANCE_LEGS',msg.utime, msg.LF[0])
#addSignal('HYQ_STANCE_LEGS',msg.utime, msg.RF[0])
#addSignal('HYQ_STANCE_LEGS',msg.utime, msg.LH[0])
#addSignal('HYQ_STANCE_LEGS',msg.utime, msg.RH[0])

addPlot()
#addSignal('POSE_GROUND_TRUTH',msg.utime, msg.vel[0])
#addSignal('POSE_RAW_KIN',msg.utime,msg.vel[0])
#addSignal('POSE_BODY',msg.utime, msg.vel[0])
addSignal('HYQ_VELDEBUG',msg.utime,msg.LF[0])
addSignal('HYQ_VELDEBUG',msg.utime,msg.RF[0])
addSignal('HYQ_VELDEBUG',msg.utime,msg.LH[0])
addSignal('HYQ_VELDEBUG',msg.utime,msg.RH[0])
addSignal('POSE_GROUND_TRUTH',msg.utime, msg.vel[0])


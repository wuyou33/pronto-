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
addSignal('POSE_GROUND_TRUTH', msg.utime, msg.rotation_rate[0])
addSignal('MICROSTRAIN_INS',msg.utime, msg.gyro[0])
addSignal('POSE_BODY', msg.utime, msg.rotation_rate[0])

addPlot()
addSignal('POSE_GROUND_TRUTH', msg.utime, msg.rotation_rate[1])
addSignal('MICROSTRAIN_INS',msg.utime, msg.gyro[1])
addSignal('POSE_BODY', msg.utime, msg.rotation_rate[1])


addPlot()
addSignal('POSE_GROUND_TRUTH', msg.utime, msg.rotation_rate[2])
addSignal('MICROSTRAIN_INS',msg.utime, msg.gyro[2])
addSignal('POSE_BODY', msg.utime, msg.rotation_rate[2])

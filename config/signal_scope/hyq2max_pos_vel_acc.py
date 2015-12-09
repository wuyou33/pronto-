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

addSignal('POSE_GROUND_TRUTH', msg.utime, msg.pos[0])
addSignal('POSE_BODY', msg.utime, msg.pos[0])

addSignal('POSE_GROUND_TRUTH', msg.utime, msg.pos[1])
addSignal('POSE_BODY', msg.utime, msg.pos[1])


addSignal('POSE_GROUND_TRUTH', msg.utime, msg.pos[2])
addSignal('POSE_BODY', msg.utime, msg.pos[2])

addPlot()
addSignal('POSE_GROUND_TRUTH', msg.utime, msg.vel[0])
addSignal('POSE_BODY', msg.utime, msg.vel[0])

addSignal('POSE_GROUND_TRUTH', msg.utime, msg.vel[1])
addSignal('POSE_BODY', msg.utime, msg.vel[1])


addSignal('POSE_GROUND_TRUTH', msg.utime, msg.vel[2])
addSignal('POSE_BODY', msg.utime, msg.vel[2])

addPlot()
addSignal('POSE_GROUND_TRUTH', msg.utime, msg.accel[0])
addSignal('POSE_BODY', msg.utime, msg.accel[0])

addSignal('POSE_GROUND_TRUTH', msg.utime, msg.accel[1])
addSignal('POSE_BODY', msg.utime, msg.accel[1])


addSignal('POSE_GROUND_TRUTH', msg.utime, msg.accel[2])
addSignal('POSE_BODY', msg.utime, msg.accel[2])


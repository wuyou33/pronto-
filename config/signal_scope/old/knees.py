findLCMTypes(os.path.expanduser('../lib/python2.7/dist-packages/*'))


addSignal('HYQ_TAUDEBUG', msg.utime, msg.pos[2]);
addSignal('HYQ_STATE',msg.utime,msg.joint_effort[2]);


addPlot()
addSignal('HYQ_TAUDEBUG', msg.utime, msg.vel[2]);
addSignal('HYQ_STATE',msg.utime,msg.joint_effort[5])



addPlot()
addSignal('HYQ_TAUDEBUG', msg.utime, msg.accel[2]);
addSignal('HYQ_STATE',msg.utime,msg.joint_effort[8]);


addPlot()
addSignal('HYQ_TAUDEBUG', msg.utime, msg.rotation_rate[2]);
addSignal('HYQ_STATE',msg.utime,msg.joint_effort[11])

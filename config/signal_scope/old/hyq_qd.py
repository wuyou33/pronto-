
# Some lcm messages define an array of floats and an array of strings.
# This example shows how to use a string array to find an index into the float array.


# or define many signals at once:

findLCMTypes(os.path.expanduser('../../build/lib/python2.7/dist-packages/*'))

joints = ['lf_haa_joint', 'rf_haa_joint', 'lh_haa_joint', 'rh_haa_joint']
addSignals('HYQ_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lf_hfe_joint', 'rf_hfe_joint', 'lh_hfe_joint', 'rh_hfe_joint']
addSignals('HYQ_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=msg.joint_name)

addPlot()
joints = ['lf_kfe_joint', 'rf_kfe_joint', 'lh_kfe_joint', 'rh_kfe_joint']
addSignals('HYQ_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=msg.joint_name)

# You can also define your own function to do the string lookup.
# Note, in the following function, the string array is searched
# for every message that is received, but by using the syntax above,
# the string lookup is performed just once and then cached.

#def myFunction(msg):
#    '''l_leg_kny position'''
#    idx = msg.joint_name.index('l_leg_kny')
#    return msg.utime, msg.joint_position[idx]

#addSignalFunction('EST_ROBOT_STATE', myFunction)

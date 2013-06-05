import time
import numpy as np

from numpy import pi
#from openhubo import planning
import openhubo as oh
from openhubo import hands
from openravepy import RaveCreateTrajectory,planningutils,CollisionReport

# DRC Hubo
lArmDOFs = [1,2,3,4,5,6,7]
rArmDOFs = [19,20,21,22,23,24,25]
lHandDOFs = [8,9,10]
lHandVels = [1,1,-1]
rHandDOFs = [26,27,28]
rHandVels = [1,1,-1]

# Hubo+ ?
#lArmDOFs = [14,16,18,20,22,24]
#rArmDOFs = [13,15,17,19,21,23]
#lHandDOFs = [42,43,44, 45,46,47, 48,49,50, 51,52,53, 54,55,56]
#rHandDOFs = [27,28,29, 30,31,32, 33,34,35, 36,37,38, 39,40,41]

def printOutJoints(robot, index = 0):
    if index == 1:
        dofs = rArmDOFs
    else:
        dofs = lArmDOFs
    jnts = robot.GetJoints()
    for i in dofs:
        #pdb.set_trace()
        print "joint %s: %s, [%s, %s]"%(jnts[i].GetName(),jnts[i].GetValues()[0],jnts[i].GetLimits()[0],jnts[i].GetLimits()[1])

def padJointLimits(robot, margin = 0.03):
    for j in robot.GetJoints():
        [lower,upper]=j.GetLimits()
        print j.GetName() + " old limits are [{}, {}]".format(lower[0],upper[0])
        j.SetLimits(lower+margin,upper-margin)
        [lower_new,upper_new]=j.GetLimits()
        print j.GetName() + " new limits are [{}, {}]".format(lower_new[0],upper_new[0])


def autoGrasp(env,robot,hand=0):
    #TODO: fix this, now working now.
    s = 0.1
    e = 0.001
    startoffsets = [s,s,s]
    endoffsets = [e,e,e]

    if hand == 0:
        fingers=lHandDOFs
        linkIndices=lHandLinks
        vels = lHandVels
    else:
        fingers=rHandDOFs
        linkIndices=rHandLinks
        vels = rHandVels
    pose = robot.GetDOFValues()
    keepmoving = True
    offsets = startoffsets
    while(keepmoving):
        #close the fingers by offsets
        for i in range(3):
            pose[fingers[i]] = pose[fingers[i]] + offsets[i] * vels[i]
        robot.SetDOFValues(pose)
        #check collision
        report = CollisionReport()
        collide=[0,0,0]
        links = robot.GetLinks()
        for i in range(3):
            collision = env.CheckCollision(links[linkIndices[i]], report=report)
            if collision:#len(report.contacts) > 0:
                collide[i] = 1
        #move back and modify the offset
        for i in range(3):
            if collide[i] == 1:
                pose[fingers[i]] = pose[fingers[i]] - offsets[i] * vels[i]
                offsets[i] = offsets[i] / 2

        #check below threshold
        total = 0
        for i in range(3):
            if offsets[i] < endoffsets[i]:
                total = total + 1
        if total == 3:
            keepmoving = False
        time.sleep(0.01)
        #print 'going'
        #pdb.set_trace()

def printJointValues(robot, arm='leftArm'):
    if type(arm) == str:
        print robot.GetManipulator(arm).GetArmJoints()
    elif type(arm) == int:
        print robot.GetManipulators()[arm].GetArmJoints()

def getHandInObject(env, robot, object, arm):
    with env:
        handInWorld = robot.GetManipulators()[arm].GetEndEffectorTransform()
        objectInWorld = object.GetTransform()
        handInObject = np.dot( np.linalg.inv(objectInWorld), handInWorld )
    return handInObject

def loadOpenRAVETrajFromFile(robot, filename):
    f=open(filename,'r')
    trajstring=f.read()
    f.close()

    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    traj.deserialize(trajstring)

    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

    return traj

def RunOpenRAVETraj(robot, filename):
    f=open(filename,'r')
    trajstring=f.read()
    f.close()

    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    traj.deserialize(trajstring)

    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

    controller = robot.GetController()
    controller.SetPath(traj)
    controller.SendCommand('start')
    robot.WaitForController(0)


    while(controller.IsDone() == False):
        time.sleep(0.01)
        print "waiting"

def ConvertOpenRAVETraj2HuboTraj(robot, filein, fileout):
    """convert a trajectory from OpenRAVE format to Hubo format"""
    f = open(filein, 'r')
    trajstring = f.read()
    f.close()

    traj = RaveCreateTrajectory(robot.GetEnv(), '')
    traj.deserialize(trajstring)

    """Start to output the trajectory"""
    outputfile = open(fileout,'w')

    """The header"""
    conf = traj.GetConfigurationSpecification()
    jointGroup = conf.GetGroupFromName('joint_values')
    velocityGroup = conf.GetGroupFromName('joint_velocities')
    #info stores all the info about the joint name
    info = jointGroup.name.split(' ')
    print 'robot name: %s'%(info[1])
    jnts = robot.GetJoints()
    jointIndices = [int(s) for s in info[2:len(info)]]
    print jointIndices
    #output the joint names
    for i in range(0, len(jointIndices)):
        outputfile.write(jnts[jointIndices[i]].GetName())
        if i < len(info)-1:
            outputfile.write(' ')
    outputfile.write('\n')

    """Output the sign line"""
    for i in range (len(info) - 2):
        outputfile.write('+')
        if i < len(info)-1:
            outputfile.write(' ')
    outputfile.write('\n')

    #initialize the array for the joint trajectory
    jointTraj = list()
    velocityTraj = list()
    deltatimeList = list()
    """Collect the joint data"""
    for i in range(traj.GetNumWaypoints()):
        waypoint = traj.GetWaypoint(i)
        #fill in the jointWaypoint
        jointWaypoint = list()
        velocityWaypoint = list()
        for j in range(jointGroup.dof):
            jointWaypoint.append(waypoint[j+jointGroup.offset])
            velocityWaypoint.append(waypoint[j+velocityGroup.offset])
        deltatimeList.append(waypoint[conf.GetGroupFromName('deltatime').offset])

        velocityTraj.append(velocityWaypoint)
        jointTraj.append(jointWaypoint)

    #check and smooth
    newTraj = sampleTraj(traj, robot, jointIndices, 10)

    checkJointTraj(robot, newTraj, jointIndices)

    for i in range(len(newTraj)):
        joints = newTraj[i]
        for j in range(len(joints)):
            outputfile.write('%s'%(joints[j]))
            if j != len(joints)-1:
                outputfile.write(' ')
            else:
                outputfile.write('\n')

    #print jointTraj
    outputfile.close()
    print 'exit'


def sampleTraj(traj, robot, jointIndices, executeTime, dt=0.01):
    #env = robot.GetEnv()
    spec=traj.GetConfigurationSpecification() # get the configuration specification of the trajrectory

    print 'total simulation time: %s'%(traj.GetDuration())
    numSample = executeTime / dt
    simDt = traj.GetDuration() / numSample
    newTraj = list()

    for i in range(int(numSample+1)):
        simTime = simDt * i
        #with env: # have to lock environment since accessing robot
        trajdata=traj.Sample(simTime)
        values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
        robot.SetDOFValues(values)
        newTraj.append(values[jointIndices])
        time.sleep(dt)

    return newTraj

def checkJointTraj(robot, jointTraj, jointIndices, dt = 0.01):
    """
    jointTraj - the list of joint trajectory in an array format, the last element in each entry is the delta time
    robot - the robot the traj is running for
    jointIndices - the joint the trajectory is controlling
    """
    print 'checking traj...'

    DOFVelocityLimits = robot.GetDOFVelocityLimits()
    DOFAccelerationLimits = robot.GetDOFAccelerationLimits()
    velocityLimits = DOFVelocityLimits[jointIndices]
    accelerationLimits = DOFAccelerationLimits[jointIndices]

    print 'print limits:'
    print velocityLimits
    print accelerationLimits

    velocityTraj = list()
    for i in range(len(jointTraj) - 1): #examine pair <i, i+1>
        j1 = np.array(jointTraj[i])
        j2 = np.array(jointTraj[i+1])
        velocityTraj.append( (j2 - j1)/dt )
        if checkLimits(j1, j2, dt, velocityLimits) == False:
               return False
        if i > 0:
            if checkLimits(velocityTraj[i-1], velocityTraj[i], dt, accelerationLimits) == False:
                return False
    return True

def checkLimits(d1, d2, dt, limit):
    """
    check the derivative between d2 and d1
    """
    for i in range(len(d1)):
        val = abs( (d2[i] - d1[i])/dt )
        if val > limit[i]:
            print '%s vs. %s achieved %s which exceeds %s'%(d2[i], d1[i], val, limit[i])
            return False
    return True

def writeToFile(filename, string):
    f = open(filename, 'w')
    f.write(string)
    f.close()

def openHand(robot, index = 1):
    return hands.open_huboplus_hand(robot,index)

def closeHand(robot,angle=pi/2):
    return hands.close_huboplus_hand(robot,angle)



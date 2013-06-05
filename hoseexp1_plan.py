import time
import numpy as np
from numpy import pi,array,mat
import openhubo
from openhubo import comps
from openravepy import databases,planningutils,IkParameterization,interfaces,RaveCreateProblem
from utilities_hao import rHandDOFs,lHandDOFs,lArmDOFs,rArmDOFs,writeToFile

mGraspInHoseRightHand = array([[ 1, 0,  0, 0],
                               [ 0, 1,  0, -0.025],
                               [ 0, 0,  1,  -0.08],
                               [ 0, 0,  0,  1.0000]])

#Hubo+ Hand location
mGraspInHoseLeftHand = array([[ 1, 0,  0, 0.0],
                               [ 0, 1,  0, 0.025],
                               [ 0, 0,  1,  -0.08],
                               [ 0, 0,  0,  1.0000]])

mHoseInHydrant = array([[ 1, 0,  0, 0],
                       [ 0, 1,  0, 0],
                       [ 0, 0,  1, -0.18],
                       [ 0, 0,  0,  1.0000]])


armName = ['leftArm', 'rightArm']
useArm = 0

def getHandInObject(env, robot, object, arm = 0):
    with env:
        handInWorld = robot.GetManipulators()[arm].GetEndEffectorTransform()
        objectInWorld = object.GetTransform()
        handInObject = np.dot( np.linalg.inv(objectInWorld), handInWorld )
    return handInObject

def attachHoseToHydrant(env, prob_cbirrt, robot, hydrant, hose):
    global useArm

    with env:
        hydrantInWorld = hydrant.GetTransform()

    #handInHose = getHandInObject(env, robot, hose, useArm)
    global mHoseInHydrant
    goalHoseInWorld = np.dot( hydrantInWorld, mHoseInHydrant )
    #goalHandInWorld = np.dot( goalHoseInWorld, handInHose )

    Bw = mat([0.0, 0.0,
              0.0, 0.0,
              0.0, 0.0,
              0, 0,
              0, 0,
              -np.pi, np.pi])
    graspInHose = getHandInObject(env, robot, hose, useArm)
    T0_w = comps.Transform(goalHoseInWorld[0:3,0:3], np.mat(goalHoseInWorld[0:3,3]).T)
    Tw_e = comps.Transform(graspInHose[0:3,0:3], np.mat(graspInHose[0:3,3]).T)

    moveCBiRRT(env, prob_cbirrt, robot, 'attachhose.txt', T0_w, Tw_e, Bw, useArm)

def moveHandUp(env, basemanip, robot, dist, filename):
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
                                                                iktype=IkParameterization.Type.Transform6D)

    if not ikmodel.load():
        print 'not loaded, auto generate'
        ikmodel.autogenerate()

    with robot:
        stepsize = 0.001
        traj = basemanip.MoveHandStraight(direction=[0,0,1],
                                          stepsize=stepsize,
                                          minsteps=1,
                                          maxsteps=dist/stepsize,
                                          execute=False,
                                          outputtraj=True,
                                          outputtrajobj=True)
        result=planningutils.RetimeTrajectory(traj, False, 0.1, 0.1)

    if result:
        writeToFile(filename,traj.serialize())
        robot.GetController().SetPath(traj)
        robot.WaitForController(0)
    return result

def insertHoseToHydrant(env, prob_cbirrt, basemanip, robot, dist):
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
                                                                iktype=IkParameterization.Type.Transform6D)

    if not ikmodel.load():
        print 'not loaded, auto generate'
        ikmodel.autogenerate()

    with robot:
        stepsize = 0.001
        traj = basemanip.MoveHandStraight(direction=[0,0,1],
                                          stepsize=stepsize,
                                          minsteps=1,
                                          maxsteps=dist/stepsize,
                                          execute=False,
                                          outputtraj=True,
                                          outputtrajobj='insert.txt')
        planningutils.RetimeTrajectory(traj, False, 0.1,0.1)

    writeToFile('insert.txt',traj.serialize())
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

def moveCBiRRT(env, prob_cbirrt, robot, filename, T0_w, Tw_e, Bw, armIndex):

    prob=comps.Cbirrt(prob_cbirrt,filename=filename)
    prob.quicksetup(T0_w,Tw_e,Bw,armIndex)
    openhubo.pause()
    prob.run()
    if prob.solved:
        prob.playback()
    return prob.solved


def moveStraight(env, prob_manip, robot, filename, armIndex = 0):
    #TODO: Fix this function, since it's not correct apparently
    if armIndex == 0:
        activedof = lArmDOFs
    else:
        activedof = rArmDOFs
    robot.SetActiveDOFs(activedof)

    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
                                                                iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        print 'not loaded, auto generate'
        ikmodel.autogenerate()

    with robot:
        result = prob_manip.SendCommand('MoveHandStraight direction 0 0 1 maxdist 0.05 writetraj moveup.txt')
    if result:
        prob_manip.SendCommand('traj liftarmtraj.txt')
        robot.WaitForController(0)
    return result

def graspHose(env, prob_cbirrt, basemanip, robot, hose):
    global useArm
    global armName

    with env:
        hoseInWorld = hose.GetTransform()

    global mGraspInHoseRightHand
    global mGraspInHoseLeftHand
    if useArm == 0:
        graspInHose = mGraspInHoseLeftHand
    else:
        graspInHose = mGraspInHoseRightHand

    Bw = [0.0, 0.0,
          0.0, 0.0,
          -0.1, 0.1,
          0, 0,
          0, 0,
          -np.pi, np.pi]

    #TODO: use new transform function to avoid this
    T0_w = comps.Transform(hoseInWorld[0:3,0:3], np.mat(hoseInWorld[0:3,3]).T)
    Tw_e = comps.Transform(graspInHose[0:3,0:3], np.mat(graspInHose[0:3,3]).T)

    result=moveCBiRRT(env, prob_cbirrt, robot, 'grasphose.txt', T0_w, Tw_e, Bw, useArm)

    if result:
        robot.SetActiveManipulator(armName[useArm])
        robot.Grab(hose)
        objName = "hose"
        prob_cbirrt.SendCommand('GrabBody name %s'%(objName))
    return result

if __name__ == "__main__":

    (env,options)=openhubo.setup('qtcoin')
    options.scenefile='scenes/hoseexp1.env.xml'
    options.robotfile=None

    [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)

    # initialization boilerplate
    print "Setup goals and transforms"
    hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
    hydrant_vertical = env.GetKinBody('hydrant_vertical')
    hose = env.GetKinBody('hose')
    hose.Enable(0)

    #Use new pose class to simplify initial pose
    pose = openhubo.Pose(robot)
    pose.reset()

    basemanip = interfaces.BaseManipulation(robot)

    #create problem instances
    prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(prob_cbirrt,'huboplus')

    prob_manip = RaveCreateProblem(env,'Manipulation')
    env.LoadProblem(prob_manip,'huboplus')

    #grasp the hose
    res = graspHose(env, prob_cbirrt, basemanip, robot, hose)

    #move up
    res = moveHandUp(env, basemanip, robot, 0.05, 'moveup.txt') if res else res

    #move to the hydrant
    res = attachHoseToHydrant(env, prob_cbirrt, robot, hydrant_vertical, hose) if res else res

    #insertion
    dist = 0.1
    res = insertHoseToHydrant(env, prob_cbirrt, basemanip, robot, dist) if res else res


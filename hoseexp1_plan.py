import time
import numpy as np
from numpy import pi,array,mat
import openhubo
from openhubo import comps,trajectory
from openravepy import databases,planningutils,IkParameterization,interfaces,RaveCreateProblem
from utilities_hao import writeToFile

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

useArm = 0

#FIXME: base this on some combo of grasp direction and palm coordinate system
#comment if using Hubo+
mGraspInHoseLeftHand=comps.Transform([pi,0,0])*mGraspInHoseLeftHand
mGraspInHoseLeftHand[1,3]*=-1
mHoseInHydrant=comps.Transform([0,0,pi])*mHoseInHydrant

def getHandInObject(env, robot, object, arm = 0):
    with env:
        handInWorld = robot.GetManipulators()[arm].GetEndEffectorTransform()
        objectInWorld = object.GetTransform()
        handInObject = np.dot( np.linalg.inv(objectInWorld), handInWorld )
    return handInObject

def attachHoseToHydrant(prob_cbirrt, robot, hydrant, hose):
    global useArm
    env=robot.GetEnv()
    with env:
        hydrantInWorld = hydrant.GetTransform()

    #handInHose = getHandInObject(env, robot, hose, useArm)
    global mHoseInHydrant
    goalHoseInWorld = np.dot( hydrantInWorld, mHoseInHydrant )
    #goalHandInWorld = np.dot( goalHoseInWorld, handInHose )

    Bw = [0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0,
          0, 0,
          0, 0,
          -np.pi, np.pi]
    graspInHose = getHandInObject(env,robot, hose, useArm)
    T0_w = comps.Transform(goalHoseInWorld)
    Tw_e = comps.Transform(graspInHose)

    return moveCBiRRT(prob_cbirrt, robot, 'attachhose.traj', T0_w, Tw_e, Bw, useArm)

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
                                          outputtrajobj=True)
        result=planningutils.RetimeTrajectory(traj, False, 0.1, 0.1)

    if result:
        writeToFile(filename,traj.serialize())
        robot.GetController().SetPath(traj)
        robot.WaitForController(0)
    return result

def insertHoseToHydrant(prob_cbirrt, basemanip, robot, dist):
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
                                          outputtrajobj=True)
        planningutils.RetimeTrajectory(traj, False, 0.1,0.1)

    writeToFile('insert.traj',traj.serialize())
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

def moveCBiRRT(prob_cbirrt, robot, filename, T0_w, Tw_e, Bw, index):

    tsr=comps.TSR(T0_w,Tw_e,Bw,index)
    chain=comps.TSRChain(0,1,0,tsr=tsr)
    problem = comps.Cbirrt(prob_cbirrt,chain,filename)
    print problem.Serialize()
    problem.activate(robot)
    print problem.Serialize()
    problem.run()
    problem.playback()
    while not robot.GetController().IsDone():
        time.sleep(.1)

    return problem.solved

def graspHose(prob_cbirrt, basemanip, robot, hose, index):
    """Plan motion to grasp hose from initial condition"""
    global mGraspInHoseRightHand
    global mGraspInHoseLeftHand
    manip=robot.GetManipulators()[index]

    if manip.GetName() == 'leftArm':
        graspInHose = mGraspInHoseLeftHand
    else:
        graspInHose = mGraspInHoseRightHand

    Bw = [0.0, 0.0,
          0.0, 0.0,
          0, 0,
          0, 0,
          0, 0,
          -.02, .02]

    #Transform of workspace wrt hose link (identity matrix)
    T0_w = comps.Transform(hose.GetTransform())
    Tw_e = comps.Transform(graspInHose)
    tsr=comps.TSR(T0_w,Tw_e,Bw,index)
    chain=comps.TSRChain(0,1,0,tsr=tsr)
    grasp_problem = comps.Cbirrt(prob_cbirrt,chain,'grasphose.traj')

    pose=openhubo.Pose(robot)
    pose.useregex=True
    pose['.EP']=-pi/4
    pose['LSR']=pi/4
    pose['RSR']=pi/4
    grasp_problem.set_ikguess_pose(pose=pose)
    grasp_problem.activate(robot)
    result=grasp_problem.run()

    if result:
        grasp_problem.playback()
        while not robot.GetController().IsDone():
            time.sleep(.1)
        robot.SetActiveManipulator(manip)
        robot.Grab(hose)
        prob_cbirrt.SendCommand('GrabBody name %s'%(hose.GetName()))
    return result,grasp_problem

if __name__ == "__main__":

    (env,options)=openhubo.setup('qtcoin')
    options.scenefile='scenes/hoseexp1.env.xml'

    [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)
    env.SetDebugLevel(5)

    # initialization boilerplate
    print "Setup goals and transforms"
    hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
    hydrant_vertical = env.GetKinBody('hydrant_vertical')
    hose = env.GetKinBody('hose')
    #hose.Enable(False)

    #Use new pose class to simplify initial pose
    #pose = openhubo.Pose(robot)
    #pose.useregex=True
    #pose['.EP']=-pi/4
    #pose.send()

    basemanip = interfaces.BaseManipulation(robot)

    #create problem instances
    prob_cbirrt=comps.Cbirrt.createProblem(robot)

    prob_manip = RaveCreateProblem(env,'Manipulation')
    env.LoadProblem(prob_manip,robot.GetName())

    #Set up grap coordinate system
    manip = robot.GetManipulators()[useArm]
    direction = manip.GetDirection()

    #hack to get DRC Hubo left hand oriented correctly.

    #grasp the hose
    res,grasp_problem = graspHose(prob_cbirrt, basemanip, robot, hose, useArm)

    #move up
    res = moveHandUp(env, basemanip, robot, 0.05, 'moveup.traj') if res else res

    #move to the hydrant
    res = attachHoseToHydrant(prob_cbirrt, robot, hydrant_vertical, hose) if res else res

    #insertion
    dist = 0.1
    res = insertHoseToHydrant(prob_cbirrt, basemanip, robot, dist) if res else res


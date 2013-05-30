import time
import numpy as np
from numpy import pi,array,mat
import openhubo
from openhubo import comps
from openravepy import databases,planningutils,IkParameterization,interfaces,RaveCreateProblem
from utilities_hao import rHandDOFs,lHandDOFs,lArmDOFs,rArmDOFs

mGraspInHoseRightHand = array([[ 1, 0,  0, 0],
                               [ 0, 1,  0, -0.025],
                               [ 0, 0,  1,  -0.08],
                               [ 0, 0,  0,  1.0000]])

mGraspInHoseLeftHand = array([[ 0, -1,  0, -0.025],
                              [ 1, 0,  0, 0],
                              [ 0, 0,  1,  -0.07],
                              [ 0, 0,  0,  1.0000]])

mHoseInHydrant = array([[ 1, 0,  0, 0],
                       [ 0, 1,  0, 0],
                       [ 0, 0,  1, -0.18],
                       [ 0, 0,  0,  1.0000]])


armName = ['leftArm', 'rightArm']
useArm = 1

def writeToFile(filename, string):
    f = open(filename, 'w')
    f.write(string)
    f.close()

def setToInit(basemanip,robot):

    j = robot.GetActiveDOFValues()
    goal = [0]*len(j)

    #left
    goal[1] = 0
    goal[2] = 0
    goal[3] = 0
    goal[4] = 0
    goal[5] = 0
    goal[6] = 0
    goal[7] = 0

    #right
    goal[19] = 0
    goal[20] = 0
    goal[21] = 0
    goal[22] = 0
    goal[23] = 0
    goal[24] = 0
    goal[25] = 0
 
    goal[41] = 0

    robot.SetActiveDOFValues( goal )

def openHand(robot, index = 1):
    #pdb.set_trace()
    if index == 1:
        robot.SetActiveDOFs(rHandDOFs)
        dof = [0.1,0.1,-0.1, 0.1,0.1,-0.1, 0.1,0.1,-0.1, 0.1,0.1,-0.1, 0.1,0.1,0.1]
    else:
        robot.SetActiveDOFs(lHandDOFs)
        dof = [0.1,0.1,-0.1, 0.1,0.1,-0.1, 0.1,0.1,-0.1, 0.1,0.1,-0.1, 0.1,0.1,0.1]
    robot.SetActiveDOFValues(dof)
    #raw_input('handopened')

def closeHand(robot):
    robot.SetActiveDOFs(rHandDOFs)
    dof = [0.1,0.1,-1.5, 0.1,0.1,-1.5, 0.1,0.1,-1.5, 0.1,0.1,-1.5, 0.1,0.1,-1.4]
    robot.SetActiveDOFValues(dof)

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

    handInHose = getHandInObject(env, robot, hose, useArm)
    global mHoseInHydrant
    goalHoseInWorld = np.dot( hydrantInWorld, mHoseInHydrant )
    goalHandInWorld = np.dot( goalHoseInWorld, handInHose )

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
        planningutils.RetimeTrajectory(traj, False, 0.1, 0.1)

    writeToFile(filename,traj.serialize())
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

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
    if armIndex == 0:
        activedof = lArmDOFs
    else:
        activedof = rArmDOFs
    robot.SetActiveDOFs(activedof)

    #ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
    #                                                            iktype=IkParameterization.Type.Transform6D)

    #if not ikmodel.load():
        #print 'not loaded, auto generate'
        #ikmodel.autogenerate()

    tsr = comps.TSR(T0_w,Tw_e,Bw,armIndex,'NULL',)
    chain = comps.TSRChain(0,1,0)
    chain.insertTSR(tsr)
    print tsr,chain

    prob=comps.Cbirrt(prob_cbirrt,chain,filename,30)
    print prob.TSRs

    prob.run()
    if prob.solved:
        prob.playback()


'''not correctly'''
def moveStraight(env, prob_manip, robot, filename, armIndex = 0):
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
        resp = prob_manip.SendCommand('MoveHandStraight direction 0 0 1 maxdist 0.05 writetraj moveup.txt')
    prob_manip.SendCommand('traj liftarmtraj.txt')
    robot.WaitForController(0)

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
    graspInWorld = np.dot(hoseInWorld, graspInHose)
    robot.GetManipulators()[useArm].GetEndEffector().SetTransform(graspInWorld)
    #print graspInWorld
    #raw_input("check")

    Bw = [0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0,
          0, 0,
          0, 0,
          -np.pi, np.pi]
    T0_w = comps.Transform(hoseInWorld[0:3,0:3], np.mat(hoseInWorld[0:3,3]).T)
    Tw_e = comps.Transform(graspInHose[0:3,0:3], np.mat(graspInHose[0:3,3]).T)

    moveCBiRRT(env, prob_cbirrt, robot, 'grasphose.txt', T0_w, Tw_e, Bw, useArm)
    robot.SetActiveManipulator(armName[useArm])
    robot.Grab(hose)
    objName = "hose"
    prob_cbirrt.SendCommand('GrabBody name %s'%(objName))

if __name__ == "__main__":


        





        






        
    (env,options)=openhubo.setup('qtcoin')
    options.scenefile='scenes/hoseexp1.env.xml'
    options.robotfile=None

    [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)
    #initialization
    hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
    hydrant_vertical = env.GetKinBody('hydrant_vertical')
    hose = env.GetKinBody('hose')

    basemanip = interfaces.BaseManipulation(robot)

    #create problem instances
    prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(prob_cbirrt,'huboplus')

    prob_manip = RaveCreateProblem(env,'Manipulation')
    env.LoadProblem(prob_manip,'huboplus')

    setToInit(basemanip, robot)
    #time.sleep(1)

    #grasp the hose
    #recorder.start()
    graspHose(env, prob_cbirrt, basemanip, robot, hose)

    #move up
    moveHandUp(env, basemanip, robot, 0.05, 'moveup.txt')

    #move to the hydrant
    attachHoseToHydrant(env, prob_cbirrt, robot, hydrant_vertical, hose)

    #insertion
    dist = 0.1
    insertHoseToHydrant(env, prob_cbirrt, basemanip, robot, dist)

    time.sleep(1)
    recorder.start()

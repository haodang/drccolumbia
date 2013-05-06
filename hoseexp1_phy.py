from openravepy import *
from TransformMatrix import *
from TSR import *
from str2num import *
from rodrigues import *
import time
import numpy
import pdb
from BodyMotion import *
import openhubo
from openhubo import pause
from planning import *
from utilities_hao import *

mGraspInHose = array([[ 1, 0,  0, 0],
                     [ 0, 1,  0, -0.025],
                     [ 0, 0,  1,  -0.12],
                     [ 0, 0,  0,  1.0000]])

mHoseInHydrant = array([[ 1, 0,  0, 0],
                       [ 0, 1,  0, 0],
                       [ 0, 0,  1, -0.3],
                       [ 0, 0,  0,  1.0000]])


lArmDOFs = [14,16,18,20,22,24]
rArmDOFs = [13,15,17,19,21,23]
lHandDOFs = [42,43,44, 45,46,47, 48,49,50, 51,52,53, 54,55,56]
rHandDOFs = [27,28,29, 30,31,32, 33,34,35, 36,37,38, 39,40,41]


def moveCBiRRT(env, prob_cbirrt, robot, T0_w, Tw_e, Bw, armIndex = 0):
    if armIndex == 0:
        activedof = lArmDOFs
    else:
        activedof = rArmDOFs
    robot.SetActiveDOFs(activedof)

    #ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
    #                                                            iktype=IkParameterization.Type.Transform6D)
        
    #if not ikmodel.load():
    #    print 'not loaded, auto generate'
    #    ikmodel.autogenerate()

    TSRstring = SerializeTSR(armIndex,'NULL',T0_w,Tw_e,Bw)
    TSRChainString = SerializeTSRChain(0,1,0,1,TSRstring,'NULL',[])

    with robot:
        #call the cbirrt planner and generate a file with the trajectory
        resp = prob_cbirrt.SendCommand('RunCBiRRT timelimit 30 psample 0.25 %s'%(TSRChainString))

    #method1
    #prob_cbirrt.SendCommand('traj cmovetraj.txt')
    #robot.WaitForController(0)

    #method2
    planner = Cbirrt(prob_cbirrt)
    planner.filename='cmovetraj.txt'
    RunTrajectoryFromFile(robot, planner)
    raw_input('wait until trajector is done')


    #method3
    #traj = loadTrajFromFile(robot, 'cmovetraj.txt')
    #pdb.set_trace()
    #controller = robot.GetController()
    #controller.SetPath(traj)
    #print 'begin to wait'
    #controller.SendCommand('start')


def attachHoseToHydrant(env, prob_cbirrt, robot, hydrant, hose):
    with env:
        hydrantInWorld = hydrant.GetTransform()

    handInHose = getHandInObject(env, robot, hose, 1)
    global mHoseInHydrant
    global mGraspInHose
    goalHoseInWorld = numpy.dot( hydrantInWorld, mHoseInHydrant )
    goalHandInWorld = numpy.dot( goalHoseInWorld, handInHose )

    Bw = mat([0.0, 0.0,
              0.0, 0.0,
              0.0, 0.0,
              0, 0,
              0, 0,
              -numpy.pi, numpy.pi])
    graspInHose = getHandInObject(env, robot, hose, 1)
    T0_w = MakeTransform(goalHoseInWorld[0:3,0:3], numpy.mat(goalHoseInWorld[0:3,3]).T)
    Tw_e = MakeTransform(graspInHose[0:3,0:3], numpy.mat(graspInHose[0:3,3]).T)
    moveCBiRRT(env, prob_cbirrt, robot, T0_w, Tw_e, Bw, 1)
    
def insertHoseToHydrant(env, prob_cbirrt, basemanip, robot, dist):
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
                                                                iktype=IkParameterization.Type.Transform6D)
        
    if not ikmodel.load():
        print 'not loaded, auto generate'
        ikmodel.autogenerate()

    success = basemanip.MoveHandStraight(direction=[0,0,1],
                                         stepsize=0.01,
                                         minsteps=1,
                                         maxsteps=40)

   
def graspHose(env, prob_cbirrt, basemanip, robot, hose):
    global mGraspInHose

    with env:
        hoseInWorld = hose.GetTransform()

    graspInWorld = numpy.dot(hoseInWorld, mGraspInHose)

    Bw = mat([0.0, 0.0,
              0.0, 0.0,
              0.0, 0.0,
              0, 0,
              0, 0,
              -numpy.pi, numpy.pi])
    T0_w = MakeTransform(hoseInWorld[0:3,0:3], numpy.mat(hoseInWorld[0:3,3]).T)
    Tw_e = MakeTransform(mGraspInHose[0:3,0:3], numpy.mat(mGraspInHose[0:3,3]).T)

    moveCBiRRT(env, prob_cbirrt, robot, T0_w, Tw_e, Bw, 1)
    robot.SetActiveManipulator('rightArm')
    robot.Grab(hose)

def gotoRobotStartingPose():
    handles=[]
    for k in links:
        handles.append(plotBodyCOM(env,k))
    #pause()
    
    grips = makeGripTransforms(links) 
    griphandles=plotTransforms(env,grips,array([0,0,1]))
    
    #pause()
    # make a list of Link transformations
    
    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    
    env.LoadProblem(probs_cbirrt,'hubo')
    
    setInitialPose(robot)
    time.sleep(1)
    
    #Define manips used and goals


if __name__ == "__main__":

    try:

        #initialization
        env = Environment()
        env.SetViewer('qtcoin')
        #we do not want to print out too many warning messages
        #RaveSetDebugLevel(DebugLevel.Error)
        time.sleep(0.25)

        timestep = 0.0005
        with env:
            #load from an xml file
            env.StopSimulation()
            env.Load('scenes/hoseexp1.env.xml')
            time.sleep(0.25)

            hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
            hydrant_vertical = env.GetKinBody('hydrant_vertical')
            hose = env.GetKinBody('hose')
            robot = env.GetRobot('huboplus')

            #initialize the servo controller
            controller=RaveCreateController(env,'trajectorycontroller')
            robot.SetController(controller)

            #collision checking system
            #bullet = RaveCreateCollisionChecker(env,'bullet')
            ode = RaveCreateCollisionChecker(env,'ode')
            env.SetCollisionChecker(ode)
                
            #basemanipulation interface
            basemanip = interfaces.BaseManipulation(robot)
        
            #create problem instances
            prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
            env.LoadProblem(prob_cbirrt,'huboplus')
        
            prob_manip = RaveCreateProblem(env,'Manipulation')
            env.LoadProblem(prob_manip,'huboplus')
        
            #Set an initial pose before the simulation starts
            ind = openhubo.makeNameToIndexConverter(robot)
            robot.SetDOFValues([pi/8,-pi/8],[ind('LSR'),ind('RSR')])
            controller.SendCommand('set gains 50 0 7 .9998 .1')

            #Use the new SetDesired command to set a whole pose at once.
            pose=array(zeros(60))
            #Manually align the goal pose and the initial pose so the thumbs clear
            pose[ind('RSR')]=-pi/3
            pose[ind('LSR')]=pi/3
            controller.SetDesired(pose)
        
            env.StartSimulation(timestep=timestep)

        raw_input('wait1')
        pose[ind('RSR')]=-pi/3
        pose[ind('LSR')]=pi/3
        controller.SetDesired(pose)
        raw_input('wait2')
        
        #grasp the hose
        env.StopSimulation()
        graspHose(env, prob_cbirrt, basemanip, robot, hose)
        
        #move to the hydrant
        attachHoseToHydrant(env, prob_cbirrt, robot, hydrant_vertical, hose)

        #insertion
        #dist = 0.05
        #insertHoseToHydrant(env, prob_cbirrt, basemanip, robot, dist)

        raw_input('enter to move to initial')
        pose[ind('RSR')]=-pi/4
        pose[ind('LSR')]=pi/4
        controller.SetDesired(pose)
        while(controller.IsDone() == False):
            time.sleep(0.1)
            print 'not done'
            
        raw_input('enter to exit')
        
    finally:
        env.Destroy()

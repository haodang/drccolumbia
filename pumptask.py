from openravepy import *
from TransformMatrix import *
from TSR import *
from str2num import *
from rodrigues import *
import time
import numpy
import pdb
from BodyMotion import *


ode = []
bullet = []

initWrenchInWorld = numpy.eye(4)

loc_pump_old = numpy.eye(4)
loc_pump_new = numpy.eye(4)

tipInWrench = array([[1,0,0,0.106],
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]])

graspInWrench = array([[ 1, 0,  0, -0.04],
                       [ 0, 1,  0, -0.06],
                       [ 0, 0,  1,  0.08],
                       [ 0, 0,  0,  1.0000]])

screwInPump = array([[1,0,0,-0.062*0.75],
                     [0,1,0,-0.219*0.75],
                     [0,0,1,-0.07*0.75],
                     [0,0,0,1]])

lArmDOFs = [14,16,18,20,22,24]
rArmDOFs = [13,15,17,19,21,23]
lHandDOFs = [42,43,44, 45,46,47, 48,49,50, 51,52,53, 54,55,56]
rHandDOFs = [27,28,29, 30,31,32, 33,34,35, 36,37,38, 39,40,41]

lGraspInPump = array([[-0.46589827,  0.1733781 , -0.86768591, -0.05279184],
                      [-0.54148992,  0.71968781,  0.43455509,  0.2064889 ],
                      [ 0.6998053 ,  0.67230164, -0.24141881, -0.04483006],
                      [ 0.        ,  0.        ,  0.        ,  1.        ]])
                     

rGraspInPump = array([[-0.46589827, -0.1733781,  -0.86768591, -0.05279184],
                      [ 0.54148992,  0.71968781, -0.43455509, -0.2204889 ],
                      [ 0.6998053,  -0.67230164, -0.24141881, -0.04483006],
                       [ 0.      ,    0.       ,   0.       ,  1.        ]])

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

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
    
def getCurrentGraspInWrench(env, robot, wrench):
    with env:
        with robot:
            graspInWorld = robot.GetManipulators()[1].GetEndEffectorTransform()
        wrenchInWorld = wrench.GetTransform()
        return numpy.dot( numpy.linalg.inv(wrenchInWorld), graspInWorld )

def getCurrentGraspInRobot(env, robot):
    with env:
        with robot:
            graspInWorld = robot.GetManipulators()[1].GetEndEffectorTransform()
            robotInWorld = robot.GetTransform()
        return numpy.dot( numpy.linalg.inv(robotInWorld), graspInWorld )

def getCurrentRobotXYZ(env, robot):
    with env:
        with robot:
            t = robot.GetTransform()
            return t[0:3,3]

def moveToPlaceNewPump(env, robot, basemanip, pump):
    with robot:
        t = robot.GetTransform()[0:3,3]
    global loc_pump_old
    global loc_pump_new
    t_diff = loc_pump_old[0:3,3] - loc_pump_new[0:3,3]
    goal = [t[0] + t_diff[0], t[1] + t_diff[1]]
    walkTo(env, robot, basemanip, goal)

def walk(env, robot, basemanip, offset):
    with robot:
        t = robot.GetTransform()[0:3,3]
        goal = [t[0] + offset[0], t[1] + offset[1]]
    walkTo(env, robot, basemanip, goal)
    

def moveToPumpForManipulation(env, robot, pump, basemanip):
    with env:
        t = pump.GetTransform()[0:3,3]
    goal = [t[0] - 0.25, t[1] + 0.05]
    walkTo(env, robot, basemanip, goal)

def moveToPumpForGrasping(env, robot, pump, basemanip):
    with env:
        t = pump.GetTransform()[0:3,3]
        
    goal = [t[0] - 0.355, t[1]]
    walkTo(env, robot, basemanip, goal)
    return

    
    goal = [t[0] - 0.355, t[1], t[2]]
    with robot:
        print 'height adjusted'
        t2 = robot.GetTransform()[0:3,3]
        print t[2] - t2[2]
    global ode
    global bullet
    env.SetCollisionChecker(ode)
    with env:
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z)#|DOFAffine.RotationAxis,[0,0,1])
        with robot:
            T = robot.GetTransform()
            robot.SetActiveDOFValues(goal)
            incollision = env.CheckCollision(robot)
        if incollision:
            print 'goal in collision!!'

    basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1,maxtries=10)
    robot.WaitForController(0)
    env.SetCollisionChecker(bullet)

def moveToReleasePump(env, robot, pump, basemanip):
    walk(env, robot, basemanip, [-0.5,0])
    walk(env, robot, basemanip, [0,-1.3])
    walk(env, robot, basemanip, [0.5,0])
    """
    with robot:
        t = robot.GetTransform()[0:3,3]
    goal = [t[0] - 0.5, t[1]]
    walkTo(env, robot, basemanip, goal)
    goal = [t[0] - 0.5, t[1] - 1.3]
    walkTo(env, robot, basemanip, goal)
    goal = [t[0] - 0, t[1] - 1.3]
    walkTo(env, robot, basemanip, goal)
    """
    #robot.Release(pump)
    #walk(env, robot, basemanip, [-0.2,0])
    """
    goal = [t[0] - 0.2, t[1] - 1.3]
    walkTo(env, robot, basemanip, goal)
    """

def releasePump(robot, pump, basemanip):
    robot.Release(pump)
    #walk(env, robot, basemanip, [-0.2,0])
    
def moveHandTo(env, prob_cbirrt, robot, t, armIndex = 0):

    print 'move hand to: '
    print t

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

    #place the TSR's reference frame at the object's frame relative to world frame
    T0_w = MakeTransform(t[0:3,0:3], numpy.mat(t[0:3,3]).T)

    #get the TSR's offset frame in w coordinates
    identity = numpy.eye(4)
    Tw_e = MakeTransform(identity[0:3,0:3], numpy.mat(identity[0:3,3]).T)

    #define bounds
    Bw = mat([-0.02, 0.02,
              -0.02, 0.02,
              -0.02, 0.02,
              0, 0,
              0, 0,
              0, 0])
    print T0_w
    print Tw_e

    TSRstring = SerializeTSR(armIndex,'NULL',T0_w,Tw_e,Bw)
    TSRChainString = SerializeTSRChain(0,1,0,1,TSRstring,'NULL',[])

    #call the cbirrt planner and generate a file with the trajectory
    resp = prob_cbirrt.SendCommand('RunCBiRRT timelimit 30 psample 0.25 %s'%(TSRChainString))
    prob_cbirrt.SendCommand('traj cmovetraj.txt')
    robot.WaitForController(0)

def releaseWrench(env, basemanip, prob_manip, robot, wrench):
    
    print 'move to original wrench location'

    global initWrenchInWorld
    t = initWrenchInWorld[0:3,3]
    goal = [t[0] - 0.3, t[1] + 0.23]
    walkTo(env, robot, basemanip, goal)

    initG = robot.GetManipulators()[1].GetEndEffectorTransform()
    print 'put wrench back'
    graspInWrench = getCurrentGraspInWrench(env, robot, wrench)
    graspInWorld = numpy.dot(initWrenchInWorld, graspInWrench)
    print graspInWorld
    moveHandTo(env, prob_cbirrt, robot, graspInWorld, 1)

    print 'release the wrench'
    robot.Release(wrench)
    #prob_manip.SendCommand('releaseall')
    openHand(robot)
    #taskmanip = interfaces.TaskManipulation(robot)
    #taskmanip.ReleaseFingers()
    #robot.WaitForController(0)
    moveHandTo(env, prob_cbirrt, robot, initG, 1)

def setToInit(basemanip,robot):

    j = robot.GetActiveDOFValues()
    goal = [0]*len(j)

    #left
    goal[14] = -1.8
    goal[16] = 0.5
    goal[18] = -1
    goal[20] = -1
    goal[22] = 0.0
    goal[24] = 0.0
    goal[56] = 0.1

    #right
    goal[13] = -1.8
    goal[15] = -0.5
    goal[17] = 1
    goal[19] = -1
    goal[21] = 0.0
    goal[23] = 0.0
    goal[41] = 0.1
 
    robot.SetActiveDOFValues( goal )
    openHand(robot,0)
    openHand(robot,1)

    #print "initial ee pose is"
    #print robot.GetManipulators()[1].GetEndEffectorTransform()

def goToInit(basemanip, robot):

    global lArmDOFs
    global rArmDOFs
    robot.SetActiveManipulator('leftArm')
    robot.SetActiveDOFs(lArmDOFs)
    basemanip.MoveManipulator([-1.8,0.5,-1,-1,0.0,0.0]) # call motion planner with goal joint angles
    robot.WaitForController(0) # wait

    robot.SetActiveDOFs(rArmDOFs)
    robot.SetActiveManipulator('rightArm')
    basemanip.MoveManipulator([-1.8,-0.5,1,-1,0.0,0.0]) # call motion planner with goal joint angles
    robot.WaitForController(0) # wait

    openHand(robot,0)
    openHand(robot,1)
    
def execTraj(env, robot, traj):
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

#dummy walking function
def walkTo(env, robot, basemanip, goal):

    env.SetCollisionChecker(ode)
    
    print 'move the robot to '
    print goal
    with env:
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y)
        with robot:
            T = robot.GetTransform()
            robot.SetActiveDOFValues(goal)
            incollision = env.CheckCollision(robot)
        if incollision:
            print 'goal in collision!!'

    basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1,maxtries=10)
    robot.WaitForController(0)

    env.SetCollisionChecker(bullet)

def graspPump(env, prob, robot, basemanip, pump):
    global lGraspInPump
    global rGraspInPump

    with env:
        pumpInWorld = pump.GetTransform()
    lGraspInWorld = numpy.dot(pumpInWorld, lGraspInPump)
    rGraspInWorld = numpy.dot(pumpInWorld, rGraspInPump)

    moveHandTo(env, prob, robot, lGraspInWorld, 0)
    moveHandTo(env, prob, robot, rGraspInWorld, 1)

    robot.Grab(pump)
    

#grasp the wrench with predefined grasps
#it is a hard-coded one for now
def fetchWrench(env, prob_cbirrt, prob_manip, robot, basemanip, wrench):
    #pdb.set_trace()
    print 'move to wrench'
    with env:
        t = wrench.GetTransform()[0:3,3]
    goal = [t[0] - 0.3, t[1] + 0.23]
    walkTo(env, robot, basemanip, goal)
    
    print "grasping target %s" %wrench
    global rArmDOFs
    activedof = rArmDOFs
    robot.SetActiveDOFs(activedof)

    with env:
        wrenchInWorld = wrench.GetTransform()
    initGraspInRobot = getCurrentGraspInRobot(env, robot)
    initGraspInWorld = robot.GetManipulators()[1].GetEndEffectorTransform()
    global graspInWrench
    graspInWorld = numpy.dot( wrenchInWorld, graspInWrench )
    robot.SetActiveManipulator('rightArm')

    #with robot:
    #    g2 = robot.GetManipulators()[1].GetEndEffectorTransform()
    #moveHandTo(env, prob_cbirrt, robot, g2, 1)
    #raw_input('done')

    
    #ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
    #                                                            iktype=IkParameterization.Type.Transform6D)
        
    #if not ikmodel.load():
    #    print 'not loaded, auto generate'
    #    ikmodel.autogenerate()

    #pdb.set_trace()
    #traj = basemanip.MoveToHandPosition(matrices=[graspInWorld], maxiter=4000,maxtries=10,seedik=20, outputtrajobj=True)
    #robot.WaitForController(0)

    #pdb.set_trace()
    """
    CBiRRT implementation
    """
    #place the TSR's reference frame at the object's frame relative to world frame
    T0_w = MakeTransform(wrenchInWorld[0:3,0:3], numpy.mat(wrenchInWorld[0:3,3]).T)

    #get the TSR's offset frame in w coordinates
    Tw_e = MakeTransform(graspInWrench[0:3,0:3], numpy.mat(graspInWrench[0:3,3]).T)

    #define bounds to only allow translation along the x axis of the wrench
    Bw = mat([-0.01, 0.01,   0, 0,   -0.01, 0.01,   0, 0,   0, 0,   0, 0])

    TSRstring = SerializeTSR(1,'NULL',T0_w,Tw_e,Bw)
    TSRChainString = SerializeTSRChain(0,1,0,1,TSRstring,'NULL',[])

    #call the cbirrt planner and generate a file with the trajectory
    resp = prob_cbirrt.SendCommand('RunCBiRRT timelimit 30 psample 0.25 %s'%(TSRChainString))
    prob_cbirrt.SendCommand('traj cmovetraj.txt')
    robot.WaitForController(0)

    #taskmanip = interfaces.TaskManipulation(robot)
    #taskmanip.CloseFingers()
    #robot.WaitForController(0)
    closeHand(robot)

    #prob_manip.SendCommand('setactivemanip index 1')
    #prob_manip.SendCommand('GrabBody name wrench')
    robot.Grab(wrench)

    moveHandTo(env, prob_cbirrt, robot, initGraspInWorld, 1)



def attachWrench(env, prob_cbirrt, robot, basemanip, pump, wrench):
    activedof = rArmDOFs
    robot.SetActiveDOFs(activedof)

    with env:
        pumpInWorld = pump.GetTransform()
        global screwInPump
        screwInWorld = numpy.dot(pumpInWorld, screwInPump)
        tipInWorld = numpy.dot(screwInWorld, matrixFromAxisAngle([0,0,0]))
        currentGraspInWrench = getCurrentGraspInWrench(env, robot, wrench)
        global tipInWrench
        graspInTip = numpy.dot(numpy.linalg.inv(tipInWrench), currentGraspInWrench)
        graspInWorld = numpy.dot(tipInWorld, graspInTip)

    #h=misc.DrawAxes(env,graspInWorld)

    #basemanip.MoveToHandPosition(matrices=[graspInWorld])
    #robot.WaitForController(0)

    """
    CBiRRT implementation
    """
    #place the TSR's reference frame at the object's frame relative to world frame
    T0_w = MakeTransform(tipInWorld[0:3,0:3], numpy.mat(tipInWorld[0:3,3]).T)

    #get the TSR's offset frame in w coordinates
    Tw_e = MakeTransform(graspInTip[0:3,0:3], numpy.mat(graspInTip[0:3,3]).T)

    #define bounds
    Bw = mat([0, 0,   0, 0,   -0.005, 0.005,   0,0, 0,0,   -numpy.pi/6, numpy.pi/6])

    TSRstring = SerializeTSR(1,'NULL',T0_w,Tw_e,Bw)
    TSRChainString = SerializeTSRChain(0,1,0,1,TSRstring,'NULL',[])

    #call the cbirrt planner and generate a file with the trajectory
    resp = prob_cbirrt.SendCommand('RunCBiRRT timelimit 60 psample 0.25 %s'%(TSRChainString))
    if resp == '0':
        resp = prob_cbirrt.SendCommand('RunCBiRRT timelimit 60 psample 0.25 %s'%(TSRChainString))
    prob_cbirrt.SendCommand('traj cmovetraj.txt')
    robot.WaitForController(0)
    

def manipWrench(env, robot, basemanip, wrench):
    robot.SetActiveManipulator('rightArm')
    #ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
    #                                                             iktype=IkParameterization.Type.Transform6D)
    
    #if not ikmodel.load():
    #    print 'not loaded, auto generate'
    #    ikmodel.autogenerate()
        
    with env:
        robot.SetActiveDOFs(ikmodel.manip.GetArmIndices())
        traj = RaveCreateTrajectory(env,'')
        spec = IkParameterization.GetConfigurationSpecificationFromType(IkParameterizationType.Transform6D,'linear')
        traj.Init(spec)

        wrenchInWorld = wrench.GetTransform()
        global tipInWrench
        global graspInWrench
        tipInWorld = numpy.dot(wrenchInWorld, tipInWrench)

        for angle in numpy.arange(0, numpy.pi/180, 0.01):
            newTipInWorld = numpy.dot(tipInWorld, matrixFromAxisAngle([0,0,angle]))
            newWrenchInWorld = numpy.dot(newTipInWorld, numpy.linalg.inv(tipInWrench))
            newHandInWorld = numpy.dot(newWrenchInWorld,graspInWrench)
            traj.Insert(traj.GetNumWaypoints(),poseFromMatrix(newHandInWorld))

        planningutils.RetimeAffineTrajectory(traj,maxvelocities=numpy.ones(7),maxaccelerations=5*numpy.ones(7))

    #h=misc.DrawAxes(env,tipInWorld)

    with env:

        print 'planning for turning lever'
        planner = RaveCreatePlanner(env,'workspacetrajectorytracker')
        params = Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetExtraParameters('<workspacetrajectory>%s</workspacetrajectory>'%traj.serialize(0))
        planner.InitPlan(robot,params)

        outputtraj = RaveCreateTrajectory(env,'')
        success=planner.PlanPath(outputtraj)
        if not success:
            print 'not success in planning a path'
            return False

        print 'now play'

    # also create reverse the trajectory and run infinitely
    trajectories = [outputtraj,planningutils.ReverseTrajectory(outputtraj)]
    print trajectories
    i = 1
    while i < 10:
        for traj in trajectories:
            robot.GetController().SetPath(traj)
            robot.WaitForController(0)

        i = i + 1
    return True
        
if __name__ == "__main__":

    try:

        #initialization
        env = Environment()
        env.SetViewer('qtcoin')
        viewer = env.GetViewer()
        viewer.SetSize(640,480)
        #we do not want to print out too many warning messages
        #RaveSetDebugLevel(DebugLevel.Error)

        #load from an xml file
        env.Load('scenes/pump_task.env.xml')
        pump_old = env.GetKinBody('pump_old')
        pump_new = env.GetKinBody('pump_new')
        wrench = env.GetKinBody('wrench')
        robot = env.GetRobot('huboplus')
        bullet = RaveCreateCollisionChecker(env,'bullet')
        ode = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(ode)

        with env:
            print robot.GetTransform()
            print pump_old.GetTransform()
            loc_pump_old = pump_old.GetTransform()
            loc_pump_new = pump_new.GetTransform()
        basemanip = interfaces.BaseManipulation(robot)

        #create problem instances
        prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
        env.LoadProblem(prob_cbirrt,'huboplus')

        prob_manip = RaveCreateProblem(env,'Manipulation')
        env.LoadProblem(prob_manip,'huboplus')

        #ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,
        #                                                             iktype=IkParameterization.Type.Transform6D)
        
        #if not ikmodel.load():
        #    print 'not loaded, auto generate'
        #    ikmodel.autogenerate()


        cdmodel = databases.convexdecomposition.ConvexDecompositionModel(robot)
        if not cdmodel.load():
            cdmodel.autogenerate()
        else:
            print 'loaded'

        setToInit(basemanip, robot)

        time.sleep(0.5)

        #raw_input('')
        #find the relative pose
        #print numpy.dot( numpy.linalg.inv(pump_old.GetTransform()),
        #                 robot.GetManipulators()[0].GetEndEffectorTransform())

        #remember the wrench location so that we can put it back later

        recorder = RaveCreateModule(env,'viewerrecorder')
        env.AddModule(recorder,'')
        filename = 'openrave.mpg'
        codec = 13 # mpeg2
        #recorder.SendCommand('Start 640 480 30 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))

        with env:
            #global initWrenchInWorld
            initWrenchInWorld = wrench.GetTransform()

        
        fetchWrench(env, prob_cbirrt, prob_manip, robot, basemanip, wrench)

        moveToPumpForManipulation(env, robot, pump_old, basemanip)

        with env:
            initGraspInWorld = robot.GetManipulators()[1].GetEndEffectorTransform()

        attachWrench(env, prob_cbirrt, robot, basemanip, pump_old, wrench)
        print ' unscrew '
        time.sleep(3.0)
        #manipWrench(env, robot, basemanip, wrench)
        moveHandTo(env, prob_cbirrt, robot, initGraspInWorld, 1)

        releaseWrench(env, basemanip, prob_manip, robot, wrench)
        t = getCurrentRobotXYZ(env, robot)
        t[0] = t[0] - 0.2
        walkTo(env, robot, basemanip, [t[0], t[1]])
        
        moveToPumpForGrasping(env, robot, pump_old, basemanip)
        crouch(robot)
        graspPump(env, prob_cbirrt, robot, basemanip, pump_old)
        lift(robot)

        moveToReleasePump(env, robot, pump_old, basemanip)
        crouch(robot)
        releasePump(robot, pump_old, basemanip)
        lift(robot)
        walk(env, robot, basemanip, [-0.2,0])
        goToInit(basemanip, robot)
        
        moveToPumpForGrasping(env, robot, pump_new, basemanip)
        crouch(robot)
        graspPump(env, prob_cbirrt, robot, basemanip, pump_new)
        lift(robot)

        moveToPlaceNewPump(env, robot, basemanip, pump_new)
        crouch(robot)
        releasePump(robot, pump_new, basemanip)
        lift(robot)
        walk(env, robot, basemanip, [-0.2,0])
        goToInit(basemanip, robot)
                   

        time.sleep(3.0)
        #recorder.SendCommand('Stop')

        time.sleep(1)
        raw_input('enter to exit')
    finally:
        env.Destroy()

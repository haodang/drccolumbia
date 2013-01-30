from openravepy import *
from TransformMatrix import *
from TSR import *
from str2num import *
from rodrigues import *
import time
import numpy
import pdb
import sys
from utilities_hao import *

def setToInit(basemanip,robot):

    j = robot.GetActiveDOFValues()
    goal = [0]*len(j)
    '''
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
    '''
    #left
    goal[14] = 0
    goal[16] = pi/2
    goal[18] = 0
    goal[20] = 0
    goal[22] = 0
    goal[24] = 0
    goal[56] = 0

    #right
    goal[13] = 0
    goal[15] = -pi/2
    goal[17] = 0
    goal[19] = 0
    goal[21] = 0
    goal[23] = 0
    goal[41] = 0
 
    robot.SetActiveDOFValues( goal )


if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        print 'please input the active arm index'
        exit
    activeArm = sys.argv[1]

    try:
        #initialization
        env = Environment()
        env.SetViewer('qtcoin')
        viewer = env.GetViewer()
        viewer.SetSize(640,480)
        #we do not want to print out too many warning messages
        #RaveSetDebugLevel(DebugLevel.Error)
        
        #load from an xml file
        env.Load('hoseexp1_plan.env.xml')
        hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
        hydrant_vertical = env.GetKinBody('hydrant_vertical')
        hose = env.GetKinBody('hose')
        robot = env.GetRobot('huboplus')
        bullet = RaveCreateCollisionChecker(env,'bullet')
        ode = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(ode)

        basemanip = interfaces.BaseManipulation(robot)

        #create problem instances
        prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
        env.LoadProblem(prob_cbirrt,'huboplus')

        prob_manip = RaveCreateProblem(env,'Manipulation')
        env.LoadProblem(prob_manip,'huboplus')

        setToInit(basemanip, robot)
        time.sleep(1)

        prob_cbirrt.SendCommand('traj grasphose.txt')
        robot.WaitForController(0)
        #printOutJoints(robot)
        #print hose.GetTransform()
        #prob_cbirrt.SendCommand('GrabBody name hose')
        robot.SetActiveManipulator(activeArm) #'rightArm'/'leftArm'
        robot.Grab(hose)
        prob_cbirrt.SendCommand('traj moveup.txt')
        robot.WaitForController(0)
        prob_cbirrt.SendCommand('traj attachhose.txt')
        robot.WaitForController(0)
        prob_cbirrt.SendCommand('traj insert.txt')
        robot.WaitForController(0)

        #printOutJoints(robot)
        raw_input('enter to exit')
        
    finally:
        env.Destroy()

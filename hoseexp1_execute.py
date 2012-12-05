from openravepy import *
from TransformMatrix import *
from TSR import *
from str2num import *
from rodrigues import *
import time
import numpy
import pdb
import openhubo
from openhubo import pause
from utilities_hao import *

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
            env.Load('hoseexp1_physics.env.xml')
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
            #basemanip = interfaces.BaseManipulation(robot)
        
            #create problem instances
            prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
            env.LoadProblem(prob_cbirrt,'huboplus')
        
            #prob_manip = RaveCreateProblem(env,'Manipulation')
            #env.LoadProblem(prob_manip,'huboplus')
        
            #Set an initial pose before the simulation starts
            ind = openhubo.makeNameToIndexConverter(robot)
            robot.SetDOFValues([pi/8,-pi/8],[ind('LSR'),ind('RSR')])
            controller.SendCommand('set gains 50 1 5 .9998 .1')

            #Use the new SetDesired command to set a whole pose at once.
            pose=array(zeros(60))
            #Manually align the goal pose and the initial pose so the thumbs clear
            pose[ind('RSR')]=-pi/2
            pose[ind('LSR')]=pi/2
            controller.SetDesired(pose)
        
            env.StartSimulation(timestep=timestep)

        recorder = RaveCreateModule(env,'viewerrecorder')
        env.AddModule(recorder,'')
        filename = 'hoseexp1_execute.mpg'
        codec = 13 # mpeg2
        recorder.SendCommand('Start 640 480 30 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))

        raw_input('wait until the initial pose is achieved and click enter (why should we wait here?)')
        planner = Cbirrt(prob_cbirrt)
        planner.filename='grasphose.txt'
        #RunTrajectoryFromFile(robot, planner)
        RunOpenRAVETraj(robot, planner.filename)
        raw_input('wait until trajector is done and click enter')
        #CloseRightHand(robot)
        #print hose.GetLinks()[0].GetMass()
        #hose.GetLinks()[0].SetMass(0)
        #print hose.GetLinks()[0].GetMass()
        #raw_input('wait until hand is closed and click enter')
        #robot.SetActiveManipulator('rightArm')
        #robot.Grab(hose)
        RunOpenRAVETraj(robot, 'moveup.txt')
        raw_input('wait until trajector is done and click enter')
        planner.filename='attachhose.txt'
        #RunTrajectoryFromFile(robot, planner)
        RunOpenRAVETraj(robot, planner.filename)
        raw_input('wait until trajector is done and click enter')
        RunOpenRAVETraj(robot, 'insert.txt')
        raw_input('wait until trajector is done and click enter')
        
            
        raw_input('enter to exit')
        recorder.SendCommand('Stop')        
    finally:
        env.Destroy()

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
        env.SetDebugLevel(DebugLevel.Debug)
        time.sleep(0.25)

        timestep = 0.0002
        with env:
            #load from an xml file
            env.StopSimulation()
            env.Load('hoseexp1_physics.env.xml')
            time.sleep(0.25)

            hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
            hydrant_vertical = env.GetKinBody('hydrant_vertical')
            hose = env.GetKinBody('hose')
            robot = env.GetRobot('drchubo')

            #initialize the servo controller
            controller=RaveCreateController(env,'trajectorycontroller')
            robot.SetController(controller)

            #collision checking system
            #bullet = RaveCreateCollisionChecker(env,'bullet')
            #Rob: change to pqp from ode because
            ode = RaveCreateCollisionChecker(env,'pqp')
            env.SetCollisionChecker(ode)

            #basemanipulation interface
            #basemanip = interfaces.BaseManipulation(robot)

            #create problem instances
            prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
            env.LoadProblem(prob_cbirrt,'drchubo')

            #prob_manip = RaveCreateProblem(env,'Manipulation')
            #env.LoadProblem(prob_manip,'huboplus')

            #Set an initial pose before the simulation starts
            ind = openhubo.makeNameToIndexConverter(robot)
            robot.SetDOFValues([pi/8,-pi/8],[ind('LSR'),ind('RSR')])
            controller.SendCommand('set gains 10 1 5 .9998 .1')

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
        recorder.SendCommand('Start 640 480 15 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))

        raw_input('wait until the initial pose is achieved and click enter (why should we wait here?)')


        planner = Cbirrt(prob_cbirrt)
        planner.filename='grasphose.txt'
        #RunTrajectoryFromFile(robot, planner)
        RunOpenRAVETraj(robot, planner.filename)
        #printOutJoints(robot)
        raw_input('wait until trajector is done and click enter')

        #we need to change the time step to get fine simulation results
        #ode could crash if the timestep is too big
        env.StopSimulation()
        env.StartSimulation(0.00015)
        CloseLeftHand(robot,pi)
        env.StopSimulation()
        env.StartSimulation(timestep)

        robot.SetActiveManipulator('rightArm')
        robot.Grab(hose)

        RunOpenRAVETraj(robot, 'moveup.txt')
        raw_input('wait until trajector is done and click enter')
        planner.filename='attachhose.txt'

        RunOpenRAVETraj(robot, planner.filename)
        raw_input('wait until trajector is done and click enter')
        RunOpenRAVETraj(robot, 'insert.txt')
        raw_input('wait until trajector is done and click enter')
        #printOutJoints(robot)

        raw_input('enter to exit')
        recorder.SendCommand('Stop')
    finally:
        env.Destroy()

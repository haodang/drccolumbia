from openhubo import comps
import openhubo

from openravepy import RaveCreateProblem
from numpy import pi
import openravepy as rave
import utilities_hao as hao

(env,options)=openhubo.setup('qtcoin')
options.scene='scenes/hoseexp1.env.xml'
options.robot=None

[robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)
#initialization
hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
hydrant_vertical = env.GetKinBody('hydrant_vertical')
hose = env.GetKinBody('hose')

basemanip = rave.interfaces.BaseManipulation(robot)
prob_manip = RaveCreateProblem(env,'Manipulation')
env.LoadProblem(prob_manip,'huboplus')

hao.setToInit(basemanip, robot)

#create problem instances
prob_cbirrt = RaveCreateProblem(env,'CBiRRT')
env.LoadProblem(prob_cbirrt,'huboplus')

env.StartSimulation(openhubo.TIMESTEP)

pose=openhubo.Pose(robot,ctrl)
pose['LSR']=(90-15)*pi/180
pose['RSR']=-pose['LSR']
pose.send()

planner = comps.Cbirrt(prob_cbirrt)
planner.filename='grasphose.txt'

hao.RunOpenRAVETraj(robot, planner.filename)

if openhubo.check_physics(env):
    hao.CloseLeftHand(robot,pi)

robot.SetActiveManipulator(hao.activeArm) #'rightArm'/'leftArm'
robot.Grab(hose)

hao.RunOpenRAVETraj(robot, 'moveup.txt')

robot.WaitForController(0)
planner.filename='attachhose.txt'

hao.RunOpenRAVETraj(robot, planner.filename)

robot.WaitForController(0)
hao.RunOpenRAVETraj(robot, 'insert.txt')
robot.WaitForController(0)

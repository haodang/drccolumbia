from openravepy import *
from TransformMatrix import *
from TSR import *
from str2num import *
from rodrigues import *
import time
import numpy
import pdb
from utilities_hao import *

if __name__ == "__main__":
'''export an OpenRAVE trajectory to Hubo format'''

    try:

        #initialization
        env = Environment()
        env.SetViewer('qtcoin')
        viewer = env.GetViewer()
        viewer.SetSize(640,480)
        #we do not want to print out too many warning messages
        RaveSetDebugLevel(DebugLevel.Info)
        
        #load from an xml file
        env.Load('hoseexp1_plan.env.xml')
        hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
        hydrant_vertical = env.GetKinBody('hydrant_vertical')
        hose = env.GetKinBody('hose')
        robot = env.GetRobot('huboplus')
        bullet = RaveCreateCollisionChecker(env,'bullet')
        ode = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(ode)

        filein = sys.argv[1]
        fileout = sys.argv[2]
        print 'converting OpenRAVE trajectory in file %s to Hubo format in file %s'%(filein, fileout)
        ConvertOpenRAVETraj2HuboTraj(robot, filein, fileout)
        

    finally:
        env.Destroy()

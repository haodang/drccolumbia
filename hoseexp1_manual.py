import numpy as np
from numpy import pi,array,mat
import openhubo
from openhubo import comps
from openravepy import databases,planningutils,IkParameterization,interfaces,RaveCreateProblem
from utilities_hao import rHandDOFs,lHandDOFs,lArmDOFs,rArmDOFs,writeToFile

#Get the planned goal to see if it works
from hoseexp1_plan import mGraspInHoseLeftHand

def setup_hose_tsr(prob_cbirrt, robot, hose):
    global mGraspInHoseLeftHand
    graspInHose = mGraspInHoseLeftHand

    Bw = [0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0,
          0.0, 0.0,
          -.5, .5]

    T0_w = comps.Transform()
    Tw_e = comps.Transform(graspInHose)

    tsr=comps.TSR(T0_w,Tw_e,Bw,0,hose)
    return tsr


if __name__ == "__main__":

    (env,options)=openhubo.setup('qtcoin')
    options.scenefile='scenes/hoseexp1.env.xml'

    [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)

    # initialization boilerplate
    print "Setup goals and transforms"
    hydrant_horizontal = env.GetKinBody('hydrant_horizontal')
    hydrant_vertical = env.GetKinBody('hydrant_vertical')
    hose = env.GetKinBody('hose')

    #Use new pose class to simplify initial pose
    pose = openhubo.Pose(robot)
    pose.reset()

    basemanip = interfaces.BaseManipulation(robot)

    #create problem instances
    prob_cbirrt=comps.Cbirrt.createProblem(robot)

    tsr=setup_hose_tsr(prob_cbirrt,robot,hose)
    ik=comps.GeneralIK(robot,prob_cbirrt,tsr,True)

    ik.continuousSolve(1000,show=True)



from openravepy import *
import openhubo
import numpy
import pdb

lowerBodyDOFs = [0,1,2,3,4,5,6,7,8,9,10,11,12]

def helloWorld():
    print "hello world"

def read_youngbum_traj(filename,robot,dt=.01,scale=1.0):
    """ Read in trajectory data stored in Youngbum's format (100Hz data):
        HPY LHY LHR ... RWP   (3-letter names)
        + - + ... +           (sign of joint about equivalent global axis + / -)
        0.0 5.0 2.0 ... -2.0  (Offset of joint from openHubo "zero" in YOUR sign convention)
        (data by row, single space separated)
    """
    #TODO: handle multiple spaces
    #Setup trajectory and source file
    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    ind=openhubo.makeNameToIndexConverter(robot)
    #Affine DOF are not controlled, so fill with zeros
    affinedof=numpy.zeros(7) 

    f=open(filename,'r')

    #Read in header row to find joint names
    header=f.readline().rstrip()
    print header.split(' ')

    indices=[ind(s) for s in header.split(' ')]

    #Read in sign row
    signlist=f.readline().rstrip().split(' ')
    signs=[]
    print signlist
    for s in signlist:
        if s == '+':
            signs.append(1)
        else:
            signs.append(-1)
    
    #Read in offset row (fill with zeros if not used)
    offsetlist=f.readline().rstrip().split(' ')
    print offsetlist
    offsets=[float(x) for x in offsetlist]

    k=0
    trajlist = []
    while True: 
        string=f.readline().rstrip()
        if len(string)==0:
            break
        jointvals=[float(x) for x in string.split(' ')]
        data=numpy.zeros(robot.GetDOF())

        for i in range(len(jointvals)):
            data[indices[i]]=(jointvals[i]+offsets[i])*numpy.pi/180.0*signs[i]*scale

        waypt=list(data)
        waypt.extend(affinedof)
        waypt.append(dt)
        traj.Insert(k,waypt)
        k=k+1

        if k==10:
            #rint k
            trajlist.append(traj)
            k = 0
            traj=RaveCreateTrajectory(robot.GetEnv(),'')
            config=robot.GetConfigurationSpecification()
            config.AddDeltaTimeGroup()
            traj.Init(config)

    #rint 'waypt: %d' %k
    #rint trajlist

    for traj in trajlist:
        planningutils.RetimeActiveDOFTrajectory(traj,robot,True)
        
    return trajlist

def execTrajList(robot, trajlist):
    controller=robot.GetController()
    for traj in trajlist:
        #before
        with robot:
            loc_foot = robot.GetManipulators()[2].GetEndEffectorTransform()[0:3,3]
            loc_robot = robot.GetTransform()[0:3,3]
        diff0 = loc_robot[2] - loc_foot[2]

        #action
        controller.SetPath(traj)
        robot.WaitForController(0)

        #after
        with robot:
            loc_foot = robot.GetManipulators()[2].GetEndEffectorTransform()[0:3,3]
            loc_robot = robot.GetTransform()[0:3,3]
            diff1 = loc_robot[2] - loc_foot[2]
            t = robot.GetTransform()
        diff = diff0 - diff1
        
        t[2,3] = t[2,3] - diff
        robot.SetTransform(t)

def crouch(robot):
    robot.SetActiveDOFs(lowerBodyDOFs)
    trajlist=read_youngbum_traj('crouch_traj.txt',robot,.015,.9)
    execTrajList(robot, trajlist)

def lift(robot):
    robot.SetActiveDOFs(lowerBodyDOFs)
    trajlist=read_youngbum_traj('crouch_traj.txt',robot,.015,.9)
    trajlist.reverse()
    trajlist2 = []
    for traj in trajlist:
        trajlist2.append(planningutils.ReverseTrajectory(traj))
    execTrajList(robot, trajlist2)
    

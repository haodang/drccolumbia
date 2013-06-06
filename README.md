To run the planner, run:

openhubo hoseexp1_plan.py

To execute the planned trajectories, run:

openhubo hoseexp1_execute.py (--no-physics)

Where the argument is optional to disable physics simulation.

To use other robots, use the --robot option (though grasp transforms may not work out-of-the-box):

openhubo hoseexp1_plan.py --robot path/to/robot/robot.xml

Symlinking the robot folder into the drccolumbia folder will make this easier.

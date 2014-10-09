# Multi-block grasping 

This directory contains an experiment for using force closure to maintain a 
grasp on multiple blocks using the [RPI-Matlab-simulator](https://code.google.com/p/rpi-matlab-simulator) (r469).

# Setting up RPI-sim

In MATLAB, change to the directory containing the simulator. From the MATLAB
prompt, then type the following command:

    >> setupRPIsim

# Running the grasp experiment

A grasp experiment can be run by typing:

    >>  grasp(nboxes, num-iterations)

 where _nboxes_ is the number of boxes to use and _num-iterations_ is the number
of iterations to run the simulation for. When the simulation has terminated,
the total number of seconds that it ran will be output.

# Which contact model?

The default contact model is Stewart-Trinkle with the LEMKE solver. Change
to the no-slip contact model by uncommenting line 75 of **rpi-matlab-simulator/examples/grasp.m**. If you want to disable the Principal Pivoting Method (PPM)
implementation, change ``if 1`` to ``if 0`` on line 85 and ``if (0 || errcode < 0)`` to ``if 1`` on line 102 in **rpi-matlab-simulator/engine/solvers/Drumwright/NoSlip.m**.


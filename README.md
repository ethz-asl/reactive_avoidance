This is the __preliminary__ code release for ICRA Submission "Obstacle avoidance using raycasting and Riemannian Motion Policies at kHz                                                 rates for MAVs"
 
While all the code for the policies & planner is included, we will add easy-to-run examples and 
documentation soon (and clean up the code).
 
### Please check back in approx 2-3 weeks. ###


## Prerequisites: 
- ROS Noetic
- CUDA (10.2 - 11.5 are tested by nvblox developers, others may or may not work)

System Dependencies for nvblox (nvblox itself is cloned as a submodule of this repository which is done automatically):

```
sudo apt-get install -y libgoogle-glog-dev libgtest-dev libgflags-dev python3-dev
cd /usr/src/googletest && sudo cmake . && sudo cmake --build . --target install
```

## Creating the environment and building
The following commands creates a new catkin environment, clones the required repositories and builds. 
```
mkdir rmp_planning
cd rmp_planning
mkdir src
catkin init
cd src
git clone git@github.com:ethz-asl/eigen_catkin.git
wstool update
git clone https://github.com/ethz-asl/reactive_avoidance --recurse-submodules
catkin build
```

## Possible issues:

Gflags dependency conflict (build fails when compiling nvblox with errors about gflags). Go to the src directory and run:

```
rm -rf rmpcpp/rmpcpp_planner/nvblox
git clone -b gflags_namespace_fix git@github.com:Isarm/nvblox.git rmpcpp/rmpcpp_planner/nvblox
catkin build
``` 

# Architecture

All planner types are compiled into the same executable and can be selected via command line arguments. 
See the `parser` class for the full list of options. I’ll briefly list the most important classes for the RMP planner. 

The policies are implemented in 
- NVBlox Raycasting obstacle avoidance policy: `rmpcpp_planner/src/policies/raycasting_CUDA.cc` 
- Lidar ray obstacle avoidance policy: `rmpcpp_planner/src/policies/lidarray_CUDA.cc`
- ESDF policy: `rmpcpp_planner/src/policies/simple_ESDF.cc`


The `tester` class sets up testing runs, using the `worldgen` class to generate random worlds or load custom worlds. 
There are a lot of options you can pass to the executable, which are all defined in the `parser` class. 
Check out `rmpcpp_planner/src/testing/parser.cc` for the full list of options. 

# nvblox
Some of our code heavily depends on nvblox - check it out here: https://github.com/nvidia-isaac/nvblox, its a great open-source package for 3D mapping on robots.



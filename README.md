# SimpleCar-Planning
Planning and Decision Making(RO47005) course project of group 9.

**Group members**: Jianfeng Cui - 5225256, Yulei Qiu - ... 

This repository is for the planning and control of the simple car model. It is developed based on C++ using [OMPL](http://ompl.kavrakilab.org/) and [ACADO Toolkit](http://acado.github.io/). In this scenario the kinematics car plans the path with RRT star and is controlled by MPC.

## 1. Prerequisites
The library is tested on **Ubuntu 18.04**, but it should be easy to complie in other platforms.
### OMPL
We use [OMPL](http://ompl.kavrakilab.org/) to perform the planning task. Download and install instructions can be found at: http://ompl.kavrakilab.org/download.html. We recommend to install the OMPL.app from source. OMPL.app provides GUI and bindings to FCL and also includes the core OMPL library. **Tested with OMPL.app 1.5.1**

### ACADO Toolkit
For the task of control and with optimization, we use [ACADO Toolkit](http://acado.github.io/) to perform the Model Predictive Control. Instructions of building with C++ library can be found at: http://acado.github.io/install_linux.html. 
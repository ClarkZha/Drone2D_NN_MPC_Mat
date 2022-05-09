# Drone2D_NN_MPC
This project starts as the final course project for UC Berkeley's CS 289 course. 

The goal of this project is to investigate the idea of using neural net to approximate an MPC controller. 
After being trained, the neural net can be used as a controller itself, or as an MPC accelerator by providing a good initial guess. 
This project focuses on using neural net to approximate an MPC for a multicopter moving in a 2D plane.

A video presentation can be found at: https://youtu.be/Pn3t_DQKFUo

## Dependency 
This project uses Yalmip(https://yalmip.github.io/) and Matlab 2022a with deep learning toolbox, optimization tool box, and statistics and machine learning toolbox.

## Quick start guide: 
1. Run DataCollectionMPC[datasetNumber].m to generate dataset. 
2. Run prepareData.m to combine and store the generated dataset. 
3. Run trainNNMPC.m to train the neural net approximating the MPC controller
4. Run flightSim.m to run a quick simulation comparing MPC, NN, NN-MPC controller. 

## Some reference
1. Chen, Steven, et al. "Approximating explicit model predictive control using constrained neural networks." 2018 Annual American control conference (ACC). IEEE, 2018.
2. Matlab example (https://www.mathworks.com/help/reinforcement-learning/ug/imitate-nonlinear-mpc-controller-for-flying-robot.html)

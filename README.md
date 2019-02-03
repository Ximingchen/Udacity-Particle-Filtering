## Particle Filtering and Localization of Vehicle in Virtual Environment
This repository contains a project on localizing a vehicle. Please see the original repository for all the dependencies of this project: https://github.com/udacity/CarND-Kidnapped-Vehicle-Project


---
### Introduction to particle filters
The goal of this project is to use particle filter for estimating the position of a vehicle in a virtual environment. 

Particle filter is a type of Bayesian filter. It is commonly used to obtain the estimate on the state of a dynamical system given a sequence of observations. Interested readers are refered to: (i) https://www.stats.ox.ac.uk/~doucet/doucet_johansen_tutorialPF2011.pdf, and (ii) http://appliedmaths.sun.ac.za/~herbst/MachineLearning/ExtraNotes/ParticleFilters.pdf.

The algorithm of particle filter is shown in the figure below
![alt text](https://github.com/Ximingchen/Udacity-Particle-Filtering/blob/master/images/particlefilter.png)

Essentially, it is a recursive algorithm consisting of the following steps: (step 1) pass the given collection of particles (estimates on the state) through the (stochastic) dynamical model, (step 2) calculate the probability of obtaining an observation for all particles and (step 3) resample the particles according to the probabilities calculated in step 2, i.e., obtaining posterior distribution of states.

In our project, or in practical scenarios, the pipeline is more complicated than the aforementioned algorithm, see the following figure.
![alt_text](https://github.com/Ximingchen/Udacity-Particle-Filtering/blob/master/images/framework.png)

We discuss several challenges in implementations



### Implementation challenges

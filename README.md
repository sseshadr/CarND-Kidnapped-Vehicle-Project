# CarND-Kidnapped-Vehicle-Project
Use a particle filter to localize a vehicle based on existing landmark positions.

#### NOTE: This project involves the Term 2 Simulator which can be downloaded [here] (https://github.com/udacity/self-driving-car-sim/releases).

[//]: # (Image References)

[image1]: ./readme_images/passed.gif "Final Result"

---

![alt text][image1]

#### Project Notes
Implementing a particle filter involved the following steps:

#### 1. Initializing
We can use GPS measurements to initialize the particles' pose. While doing this, we can pull from a Gaussian distribution that takes into account the sensor noise as well so that our initial particles take that effect into account. The weights are initialized to 1.

#### 2. Predicting
For the prediction, we calculate each of the particle's poses based on a bicycle motion model. We feed in the vehicle velocity and the yaw rate to predict what the pose would be. We try to realistically predict the pose by incorporating the GPS noise here as well. Another important thing to consider here is to account for a close to zero yaw rate condition under which the general bicycle model gets close to division by zero. We need to handle this with a specific apporximation as an edge case.

#### 3. Updating
In Particle Filters, updates refer to the weight calculation. This is calculated based on the distance between an observation and its associated predicted landmark location. The pseudo code for this is as follows:

##### a. Data Association 
First we need to associate prospective landmarks to observations. To do this,

1. Compute landmarks that are 'in range' of the particle position. These are the candidate landmark points so that we don't have to search through the entire map.

2. Compute observations in map coordinate system. The landmark positions are in that coordinate system whereas the observations are in the vehicle coordinate system . So before making any comparisons, we need to standardize the system.

3. For each observation, find the closest landmark using the euclidean distance as a measure. Use id property to associate the landmark's id to the observation.

##### b. Calculate Weights 
Now for each mapped landmark to observation, calculate the probabilities using a multi-variate gaussian. For each particle, take the product of all these probabilities to compute the final weight for that particle. 

##### c. Set Associations 
For the simulator, set associations to the mapped ids, and sense_x and sense_y to the observations in map coordinates.

#### 4. Resampling
Once we have the new weights, we will resample from this set for the next step. Resampling is choosing particles from the existing step for the next time step proportional to their weights.

#### Links that helped:
https://youtu.be/-3HI3Iw3Z9g

https://discussions.udacity.com/t/updating-weights-reinitialize-to-1-every-time/635524/5

https://discussions.udacity.com/t/code-stuck-after-1-update-issue-with-updating-associations-sense-x-and-sense-y/636212/2

https://discussions.udacity.com/t/drifting-from-ground-truth/636371/2




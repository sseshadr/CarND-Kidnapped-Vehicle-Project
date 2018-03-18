/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	cout << "Initializing.." << endl;
	// Initialize random engine.
	default_random_engine gen;

	// Create a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std[0]);

	// Create a normal (Gaussian) distribution for y.
	normal_distribution<double> dist_y(y, std[1]);

	// Create a normal (Gaussian) distribution for theta.
	normal_distribution<double> dist_theta(theta, std[2]);

	// Set number of particles
	num_particles = 1;

	// Initialize particles and weights.
	for (int idx = 0; idx < num_particles; idx++) {

		// Create Particle
		Particle particle;
		particle.id = idx;
		// Sample particles from the normal distrubtions.
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		// Initialize weights to 1.
		particle.weight = 1.0;
		// Assign
		particles.push_back(particle);
		weights.push_back(1.0);
	}

	// Set init flag.
	is_initialized = true;
	cout << "Initialized with " << num_particles << " particles." << endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	//cout << "Predicting.." << endl;
	// Initialize random engine.
	default_random_engine gen;

	// Update particle x,y and theta based on bicycle motion model.
	for (int idx = 0; idx < num_particles; idx++) {

		// Get pose for easy access.
		double x = particles[idx].x;
		double y = particles[idx].y;
		double theta = particles[idx].theta;

		// Predict.
		if (fabs(yaw_rate) > 0.001) {
			// For |yaw_rate| != 0 
			particles[idx].x = x + ((velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta)));
			particles[idx].y = y + ((velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t))));
			particles[idx].theta = theta + (yaw_rate * delta_t);
		}
		else {
			// For |yaw_rate| == 0, avoids division by zero
			particles[idx].x = x + (velocity * delta_t * cos(theta));
			particles[idx].y = y + (velocity * delta_t * sin(theta));
			particles[idx].theta = theta;
		}

		// Add random Gaussian Noise.
		// Create a normal (Gaussian) distribution for x and randomly sample.
		normal_distribution<double> dist_x(particles[idx].x, std_pos[0]);
		particles[idx].x = dist_x(gen);
		// Create a normal (Gaussian) distribution for y and randomly sample.
		normal_distribution<double> dist_y(particles[idx].y, std_pos[1]);
		particles[idx].y = dist_y(gen);
		// Create a normal (Gaussian) distribution for theta and randomly sample.
		normal_distribution<double> dist_theta(particles[idx].theta, std_pos[2]);
		particles[idx].theta = dist_theta(gen);

	}
	//cout << "Predicted.." << endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//cout << "Associating.." << endl;

	// Init min to max double for running minimum
	LandmarkObs nearestLM;

	for (int idx = 0; idx < observations.size(); idx++) {
		double minDist = numeric_limits<double>::max();
		for (int jdx = 0; jdx < predicted.size(); jdx++) {

			// Calculate distance to all landmarks
			double distDiff = dist(predicted[jdx].x, predicted[jdx].y,
				observations[idx].x, observations[idx].y);

			// Iteratively get minimum and leave with the correct association
			if (distDiff < minDist)
			{
				minDist = distDiff;
				observations[idx].id = predicted[jdx].id;
				nearestLM = predicted[jdx];
			}
			
		}
		cout << "Observation: " << observations[idx].x << ", " << observations[idx].y << " Id: " << observations[idx].id << endl;
		cout << "Landmark: " << nearestLM.x << ", " << nearestLM.y << " Id: " << nearestLM.id << endl;
	}

	//cout << "Associated.." << endl;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	//cout << "Updating.." << endl;
		
	for (int idx = 0; idx < num_particles; idx++) {

		// Get landmarks in range
		vector<LandmarkObs> map_inrangelandmarks;
		Map::single_landmark_s slandmark;
		LandmarkObs inrangelandmark;

		for (int jdx = 0; jdx < map_landmarks.landmark_list.size(); jdx++) {

			slandmark = map_landmarks.landmark_list[jdx];

			// Check if landmarks in range of particles
			if ((fabs(slandmark.x_f - particles[idx].x) <= sensor_range) &&
				(fabs(slandmark.y_f - particles[idx].y) <= sensor_range))
			{
				// Create a vector of landmarks in range
				inrangelandmark.id = slandmark.id_i;
				inrangelandmark.x = slandmark.x_f;
				inrangelandmark.y = slandmark.y_f;
				map_inrangelandmarks.push_back(inrangelandmark);
			}
		}
		
		
		// Transform observations to map coordinates
		vector<LandmarkObs> map_observations;
		LandmarkObs obs;
		vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;

		for (int jdx = 0; jdx < observations.size(); jdx++) {

			LandmarkObs obstrans;
			obs = observations[jdx];

			//if (particles[idx].theta > 0) {
				// Counter clockwise
				// Perform transformation
				obstrans.x = particles[idx].x + ((obs.x * cos(particles[idx].theta)) - (obs.y * sin(particles[idx].theta)));
				obstrans.y = particles[idx].y + ((obs.x * sin(particles[idx].theta)) + (obs.y * cos(particles[idx].theta)));
			//}
			//else {
				// Clockwise
				// Perform transformation
				//obstrans.x = particles[idx].x + ((obs.x * cos(particles[idx].theta)) + (obs.y * sin(particles[idx].theta)));
				//obstrans.y = particles[idx].y - ((obs.x * sin(particles[idx].theta)) + (obs.y * cos(particles[idx].theta)));
			//}
			
			obstrans.id = observations[jdx].id;
			map_observations.push_back(obstrans);
						
			}

		// Associate global observations to nearest landmarks
		dataAssociation(map_inrangelandmarks, map_observations);

		// Reinitialize weights before updating them
		particles[idx].weight = 1.0;
		weights[idx] = 1.0;
		// For all observations
		for (int jdx = 0; jdx < map_observations.size(); jdx++){

			// For all landmarks
			for (int kdx = 0; kdx < map_inrangelandmarks.size(); kdx++) {

				// Calculate probability
				double normalizer = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
				//cout << "Norm: " << normalizer << endl;
				double xterm = pow((map_observations[jdx].x - map_inrangelandmarks[kdx].x) / std_landmark[0], 2) / 2.0;
				//cout << "x: " << xterm << endl;
				double yterm = pow((map_observations[jdx].y - map_inrangelandmarks[kdx].y) / std_landmark[1], 2) / 2.0;
				//cout << "y: " << yterm << endl;
				double probw = normalizer * exp(-(xterm + yterm));
				//cout << "p: " << probw << endl;
				particles[idx].weight *= probw;
				weights[idx] = particles[idx].weight;

			}
			// Create vectors for associations, sense_x and sense_y
			associations.push_back(map_observations[jdx].id);
			sense_x.push_back(map_observations[jdx].x);
			sense_y.push_back(map_observations[jdx].y);
		}
		// Assign particle associations, sense_x and sense_y based on vectors
		particles[idx].associations = associations;
		particles[idx].sense_x = sense_x;
		particles[idx].sense_y = sense_y;
		// cout << "p" << idx << ": " << particles[idx].weight << endl;
	}
	//cout << "Updated.." << endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//cout << "Resampling.." << endl;
			
	// Initialize random engine.
	default_random_engine gen;

	// Create a discrete distribution for weights.
	discrete_distribution<int> dist_w(weights.begin(), weights.end());

	vector<Particle> resampled;

	for (int idx = 0; idx < num_particles; idx++) {

		resampled.push_back(particles[dist_w(gen)]);
	}

	particles = resampled;
	//cout << "Resampled.." << endl;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

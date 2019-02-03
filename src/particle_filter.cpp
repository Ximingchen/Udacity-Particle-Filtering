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
#include "helper_functions.h"
#include "particle_filter.h"

using namespace std;
/*

struct Particle {

int id;
double x;
double y;
double theta;
double weight;
std::vector<int> associations;
std::vector<double> sense_x;
std::vector<double> sense_y;
};

*/

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (is_initialized) {
		return;
	}
	std::default_random_engine gen;

	// Initializing the number of particles
	num_particles = 100;

	// Extracting standard deviations
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	// Creating normal distributions
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	// Generate particles with normal distribution with mean on GPS values.
	for (int i = 0; i < num_particles; i++) {

		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;

		particles.push_back(particle);
		weights.push_back(1.0);
	}
	cout << " sucessfully initialzied " << endl;
	// The filter is now initialized.
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	if (particles.size() == 0) return;
	
	normal_distribution<double> Dx(0, std_pos[0]); // create normal distribution for x,y,theta respectively
	normal_distribution<double> Dy(0, std_pos[1]);
	normal_distribution<double> Dtheta(0, std_pos[2]);

	default_random_engine rand_gen;
	// the vehicle model is defined by:
	// x_t = x_0 + v/yaw_rate[sin(theta_0 + yaw_rate * dt) - sin(theta_0)]
	// y_t = y_0 + v/yawrate[cos(theta_0) - cos(theta_0 + yaw_rate * dt)]
	// theta_t = theta_0 + yaw_rate dt;
	double tolerence = 0.00001;
	for (int i = 0; i < particles.size(); i++) {
		double x0 = particles[i].x;
		double y0 = particles[i].y;
		double theta0 = particles[i].theta;
		double error_x = Dx(rand_gen);
		double error_y = Dy(rand_gen);
		double error_theta = Dtheta(rand_gen);
		if (fabs(yaw_rate) < tolerence) {
			particles[i].x = x0 + velocity * delta_t * cos(theta0) + error_x;
			particles[i].y = y0 + velocity * delta_t * sin(theta0) + error_y;
		}
		else {
			particles[i].x = x0 + velocity / yaw_rate * (sin(theta0 + yaw_rate * delta_t) - sin(theta0)) + error_x;
			particles[i].y = y0 + velocity / yaw_rate * (cos(theta0) - cos(theta0 + yaw_rate * delta_t)) + error_y;
		}
		particles[i].theta = theta0 + yaw_rate * delta_t + error_theta;
	}
	cout << " running predictions " << endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and 
	// assign the observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	//
	unsigned int nObservations = observations.size();
	unsigned int nPredictions = predicted.size();

	for (unsigned int i = 0; i < nObservations; i++) { // For each observation

													   // Initialize min distance as a really big number.
		double minDistance = numeric_limits<double>::max();

		// Initialize the found map in something not possible.
		int mapId = -1;

		for (unsigned j = 0; j < nPredictions; j++) { // For each predition.

			double xDistance = observations[i].x - predicted[j].x;
			double yDistance = observations[i].y - predicted[j].y;

			double distance = xDistance * xDistance + yDistance * yDistance;

			// If the "distance" is less than min, stored the id and update min.
			if (distance < minDistance) {
				minDistance = distance;
				mapId = predicted[j].id;
			}
		}

		// Update the observation identifier.
		observations[i].id = mapId;
	}
	cout << " data associations " << endl;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a multivariate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// transformation from one system to another:
	double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	cout << " starting to update weight " << endl;
	for (int i = 0; i < num_particles; i++) {
		// step 0: retrieving current information of the particle
		double x_pos = particles[i].x;
		double y_pos = particles[i].y;
		double theta = particles[i].theta;

		// step 1: find the landmarks that can be measured by the vehicle in the current position
		vector<LandmarkObs> closelandmarks;
		for (int idx_lm = 0; idx_lm < map_landmarks.landmark_list.size(); idx_lm++) {
			double lm_x_pos = map_landmarks.landmark_list[idx_lm].x_f;
			double lm_y_pos = map_landmarks.landmark_list[idx_lm].y_f;
			int landmark_id = map_landmarks.landmark_list[idx_lm].id_i;
			if (dist(x_pos, y_pos, lm_x_pos, lm_y_pos) <= sensor_range) { // landmark within the range of vehicle
				closelandmarks.push_back(LandmarkObs{ landmark_id, lm_x_pos, lm_y_pos });
			}
		}
		// step 2: tranform observations into the map coordinates

		vector<LandmarkObs> transformed_obs(observations.begin(), observations.end());
		for (int idx_obs = 0; idx_obs < observations.size(); idx_obs++) {
			double obs_x = observations[idx_obs].x, obs_y = observations[idx_obs].y;
			double mapped_x = cos(theta) * obs_x - sin(theta) * obs_y + x_pos;
			double mapped_y = sin(theta) * obs_x + cos(theta) * obs_y + y_pos;
			transformed_obs[idx_obs].x = mapped_x;
			transformed_obs[idx_obs].y = mapped_y;
		}

		// step 3: data association - assigning observations to landmarks
		dataAssociation(closelandmarks, transformed_obs);

		// step 4: weight calculations
		double particle_obs_prob = 1.0;

		for (int idx_obs = 0; idx_obs < transformed_obs.size(); idx_obs++) {
			// retrieving the transformed observations
			double o_x = transformed_obs[idx_obs].x, o_y = transformed_obs[idx_obs].y;
			int landmark_id = transformed_obs[idx_obs].id;

			// find the position of the landmark (in the global coordinate) whose id matches the landmark id
			double lm_x = 0, lm_y = 0;
			for (auto lm : closelandmarks) {
				if (lm.id == landmark_id) {
					lm_x = lm.x;
					lm_y = lm.y;
					break;
				}
			}

			// calculating the probability

			double dx = o_x - lm_x, dy = o_y - lm_y;
			double obs_prob = (1 / (2*M_PI*std_x*std_y)) * exp(-(dx*dx / (2 * std_x* std_x) + (dy*dy / (2 * std_y * std_y))));
			if (obs_prob == 0) {
				particle_obs_prob *= 0.00000001;
			} else {
				particle_obs_prob *= obs_prob;
			}
		}
		particles[i].weight = particle_obs_prob; // assigning probability
		weights[i] = particle_obs_prob;
	}
	cout << " weight updated " << endl;
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	// normalizing the weights
	default_random_engine rand_gen;
	cout << " resampling ... " << endl;
	discrete_distribution<> dist(weights.begin(), weights.end());
	vector<Particle> newParticles;
	for (int i = 0; i < num_particles; i++) {
		int sampled_idx = dist(rand_gen);
		newParticles.push_back(particles[sampled_idx]);
	}
	cout << " new samples generated " << endl;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
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

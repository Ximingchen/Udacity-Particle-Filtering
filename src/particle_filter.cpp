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
	is_initialized = true;
	num_particles = 100;  // Set the number of particles
	default_random_engine generator;

	normal_distribution<double> Dx(x, std[0]); // create normal distribution for x,y,theta respectively
	normal_distribution<double> Dy(y, std[1]);
	normal_distribution<double> Dtheta(theta, std[2]);

	particles.resize(num_particles); // set the size for vectors
	weights.resize(num_particles);
	for (int i = 0; i < num_particles; i++) {
		// initialize all particles to first 
		weights[i] = 1.0;
		Particle part;
		part.id = i;
		part.x = Dx(generator);
		part.y = Dy(generator);
		part.theta = Dtheta(generator);
		part.weight = 1.0;
		particles[i] = part;
	}
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
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and 
	// assign the observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	//
	for (auto & obs_landmark : observations) {
		double obs_lm_xpos = obs_landmark.x;
		double obs_lm_ypos = obs_landmark.y;
		double min_dist = numeric_limits<double>::max();
		int ID = -1;
		for (auto pred_landmark : predicted) {
			double cur_dist = dist(obs_lm_xpos, obs_lm_ypos, pred_landmark.x, pred_landmark.y);
			if (cur_dist < min_dist) {
				min_dist = cur_dist;
				ID = pred_landmark.id;
			}
		}
		obs_landmark.id = ID; // cannot find any associations
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
	double stdLandmarkRange = std_landmark[0];
	double stdLandmarkBearing = std_landmark[1];
	std::default_random_engine gen;
	for (int i = 0; i < num_particles; i++) {

		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		// Find landmarks in particle's range.
		double sensor_range_2 = sensor_range * sensor_range;
		vector<LandmarkObs> inRangeLandmarks;
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			float landmarkX = map_landmarks.landmark_list[j].x_f;
			float landmarkY = map_landmarks.landmark_list[j].y_f;
			int id = map_landmarks.landmark_list[j].id_i;
			double dX = x - landmarkX;
			double dY = y - landmarkY;
			if (dX*dX + dY * dY <= sensor_range_2) {
				inRangeLandmarks.push_back(LandmarkObs{ id, landmarkX, landmarkY });
			}
		}

		// Transform observation coordinates.
		vector<LandmarkObs> mappedObservations;
		for (unsigned int j = 0; j < observations.size(); j++) {
			double xx = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
			double yy = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
			mappedObservations.push_back(LandmarkObs{ observations[j].id, xx, yy });
		}

		// Observation association to landmark.
		dataAssociation(inRangeLandmarks, mappedObservations);

		// Reseting weight.
		particles[i].weight = 1.0;
		// Calculate weights.
		for (unsigned int j = 0; j < mappedObservations.size(); j++) {
			double observationX = mappedObservations[j].x;
			double observationY = mappedObservations[j].y;

			int landmarkId = mappedObservations[j].id;

			double landmarkX, landmarkY;
			unsigned int k = 0;
			unsigned int nLandmarks = inRangeLandmarks.size();
			bool found = false;
			while (!found && k < nLandmarks) {
				if (inRangeLandmarks[k].id == landmarkId) {
					found = true;
					landmarkX = inRangeLandmarks[k].x;
					landmarkY = inRangeLandmarks[k].y;
				}
				k++;
			}

			// Calculating weight.
			double dX = observationX - landmarkX;
			double dY = observationY - landmarkY;

			double weight = (1 / (2 * M_PI*stdLandmarkRange*stdLandmarkBearing)) * exp(-(dX*dX / (2 * stdLandmarkRange*stdLandmarkRange) + (dY*dY / (2 * stdLandmarkBearing*stdLandmarkBearing))));
			if (weight == 0) {
				particles[i].weight *= 0.00001;
			}
			else {
				particles[i].weight *= weight;
			}
		}
		weights[i] = particles[i].weight;
	}
	cout << " weight updated " << endl;
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	// normalizing the weights
	default_random_engine gen;
	cout << " resampling ... " << endl;
	vector<double> weightslocal;
	double maxWeight = numeric_limits<double>::min();
	for (int i = 0; i < num_particles; i++) {
		weightslocal.push_back(particles[i].weight);
		if (particles[i].weight > maxWeight) {
			maxWeight = particles[i].weight;
		}
	}

	// Creating distributions.
	uniform_real_distribution<double> distDouble(0.0, maxWeight);
	uniform_int_distribution<int> distInt(0, num_particles - 1);

	// Generating index.
	int index = distInt(gen);

	double beta = 0.0;

	// the wheel
	vector<Particle> resampledParticles;
	for (int i = 0; i < num_particles; i++) {
		beta += distDouble(gen) * 2.0;
		while (beta > weightslocal[index]) {
			beta -= weightslocal[index];
			index = (index + 1) % num_particles;
		}
		resampledParticles.push_back(particles[index]);
	}

	particles = resampledParticles;
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

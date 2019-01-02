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

// random engine for initializing random seed
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    
	// Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
    
    // set number of particles - lower number of particles is faster at runtime
    num_particles = 100;
    
    // resize particles set vector to match number of initialised particles
    particles.resize(num_particles);
    // resize weights vector to match particle count
    weights.resize(num_particles);
    
    // create random numbers around distribtuion mean x,y and theta with std_dev std[n]
    normal_distribution<double> dist_x(x, std[0]);  // std[0] is std_x, sigma_pos[0] = 0.3
    normal_distribution<double> dist_y(y, std[1]); // std[1] is std_y, sigma_pos[0] = 0.3
    normal_distribution<double> dist_theta(theta, std[2]); // std[2] is std_theta, sigma_pos[0] = 0.01
    
    // loop through each particle that exists
    for (int i = 0; i < num_particles; i++) {
        
        // create temporary particle to store in initialise values
        Particle init_p;
        // set id to loop value
        init_p.id = i;
        
        // Add random Gaussian noise to particle to guess initial location
        // spawn around best guess
        init_p.x = dist_x(gen);
        init_p.y = dist_y(gen);
        init_p.theta = dist_theta(gen);
        // initialise weight to 1.0
        init_p.weight = 1.0;
        
        // update actual particles from temp particle
        particles[i] = init_p;
        // update weights to same as particle
        weights[i] = init_p.weight;
        
    }
    // set initialisation flag to true
    is_initialized = true;
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// adding noise using std::normal_distribution and std::default_random_engine
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // create random numbers around distribtuion mean x,y and theta with std_dev std[n]
    normal_distribution<double> dist_x(0.0, std_pos[0]);  // std[0] is std_x, sigma_pos[0] = 0.3
    normal_distribution<double> dist_y(0.0, std_pos[1]); // std[1] is std_y, sigma_pos[0] = 0.3
    normal_distribution<double> dist_theta(0.0, std_pos[2]); // std[2] is std_theta, sigma_pos[0] = 0.01
    
    // loop through each particle that exists
    for (int i = 0; i < num_particles; i++) {

        // store particle theta into variable
        double p_theta = particles[i].theta;

        // determine if straight or curved heading
        if (fabs(yaw_rate) < 1e-5) {  // zero values indicate straight
            // if now yaw then simple vector calculation
            particles[i].x += velocity*delta_t*cos(p_theta);
            particles[i].y += velocity*delta_t*sin(p_theta);
            // yaw rate is near zero, therefore no change to yawrate
            
        } else {
            // if yaw is not zero then curve
            particles[i].x += velocity/yaw_rate*(sin(p_theta+yaw_rate*delta_t) - sin(p_theta));
            particles[i].y += velocity/yaw_rate*(cos(p_theta)-cos(p_theta+yaw_rate*delta_t));
            particles[i].theta += yaw_rate*delta_t;
        }
        
        // upate particle and add gaussian noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
        
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    
    // match landmark measurements to observed/predicted measurements - nearest neighbour method
    // predicted measurements and landmark measurements are in same coordinate system

    // loop through each predicted measurment
    for (int i =0; i < observations.size(); i++) {
        
        // track closest measurements id - initialised as something not possible in map
        int landmark_id = -1;
        
        // set minimum distance to start as a large distance
        double minimum_distance = numeric_limits<double>::max();
        
        // loop through observed predictions
        for (int j = 0; j < predicted.size(); j++) {
            
            // calculate distance difference between two vectors - predicted and map landmark
            double distance = fabs(dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y));
            
            // find closet point
            if (distance < minimum_distance) {
                // if calculated distance is less than current minimum distance, make it the new minimum distance
                minimum_distance = distance;
                // store shorter distance id
                landmark_id = predicted[j].id;
            }
        }
        // update observation id with closet distance landmark id
        observations[i].id = landmark_id;
    }
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
    
    
    //sigma x, sigma y and fixed variables part of multivariate gaussian probability calculation
    const double sigma_x = std_landmark[0];
    const double sigma_y = std_landmark[1];
    const double fixed_normaliser = 1.0/(2*M_PI*sigma_x*sigma_y);
    const double x_denominator = 2*(pow(sigma_x,2));
    const double y_denominator = 2*(pow(sigma_y,2));

    
    // 3 steps:
    // Transform car sensor coordinates to landmark coordinates
    // Assosicate transformed observations to nearest map landmarks
    // Update Particle weight:
            // apply multivariate gaussian probability for each measurement
            // get product of measurement probabllities
            // calculated weight is posterior probability
    
    // iterate through each particle avaialble - vehicle position in map cooridinate reference
    for (int p = 0; p < num_particles; p++) {
        
        // car potetential position in map cooridinate
        double ego_x = particles[p].x;
        double ego_y = particles[p].y;
        double ego_theta = particles[p].theta;
        
        
        // --------------------- Transfrom observations to map coordianates ---------------------------------
        // vector to hold transformed observations in map coordinates
        vector<LandmarkObs> trans_observations;
        // loop through observations and transfrom into map coordinate system
        for (int ob = 0; ob < observations.size(); ob++) {
            
            // create temp variable to hold translated observation
            LandmarkObs trans_obs;
            
            // perform transform from vehicle coordinates to map coordinates
            trans_obs.id = observations[ob].id;
            trans_obs.x = ego_x + ((observations[ob].x * (cos(ego_theta))) - (observations[ob].y * (sin(ego_theta))));
            trans_obs.y = ego_y + ((observations[ob].x * (sin(ego_theta))) + (observations[ob].y * (cos(ego_theta))));
            trans_observations.push_back(trans_obs);
            
        }
        
        // --------------------- Associate observations to map Landmarks -------------------------------------

        // get landmarks within sensor range and transform to map coordinates
        
        // vector to store landmarks that are in range of the particle
        std::vector<LandmarkObs> inrange_landmarks;
        
        // get map landmarks in range of particle
        for (int lm = 0; lm < map_landmarks.landmark_list.size(); lm++) { // 42 elements long
            
            // temp store for landmark data
            int lm_id_i = map_landmarks.landmark_list[lm].id_i; // Landmark ID
            double lm_x_d = map_landmarks.landmark_list[lm].x_f; // Landmark x-position in the map (global coordinates)
            double lm_y_d = map_landmarks.landmark_list[lm].y_f; // Landmark y-position in the map (global coordinates)

            // filter out all landmarks which are out of range of sensor based on vehicle current position
            // Computes the Euclidean distance between two 2D points.
            if (fabs(dist(lm_x_d, lm_y_d, ego_x, ego_y)) <= sensor_range) {
                // if result is less than sensor range then add landmark to within range list
                inrange_landmarks.push_back(LandmarkObs{lm_id_i,lm_x_d,lm_y_d});
            }
        }
        
        // data association between transformed observed points and map points within range - update id in trans_observations with landmark id
        dataAssociation(inrange_landmarks, trans_observations);
        
        // ---------------------------------------------------------------------------------------------------
        
        // --------------------- Update Particle weight ------------------------------------------------------
        
        // ----- Calculate multivariate gaussian for each measurement --------
        
        
         // create variables to hold annotations lines information for simulator viewing
        vector<int> associations;
        vector<double> sense_x;
        vector<double> sense_y;
        
        // variable to hold resulting product of weights
        double mv_weight = 1.0;
        
        // for each observation
        // loop through each transformed observations find the error to the landmark
        for (int i = 0; i < trans_observations.size(); i++) {
            
            // measurment parameters
            int meas_id = trans_observations[i].id;
            double meas_x = trans_observations[i].x;
            double meas_y = trans_observations[i].y;
            // map parameters
            double map_x = 0.0, map_y = 0.0;
            
            // secondary loop through the in range landmarks
            for (int j = 0;  j < inrange_landmarks.size(); j++) {
                // find id match in landmarks which matches the current id in trans_observations
                if (inrange_landmarks[j].id == meas_id) {
                    // assign match landmark to
                    map_x = inrange_landmarks[j].x;
                    map_y = inrange_landmarks[j].y;
                }
            }
            
            // for multivariate gaussian calculation - find difference between map location and measured prediction of same point
            double x_factor = pow((meas_x - map_x),2);
            double y_factor = pow((meas_y - map_y),2);
            
            // multivaritate gaussian calculation, summing weight product at each loop
            mv_weight *= (fixed_normaliser) * exp(-((y_factor/x_denominator) + (x_factor/(y_denominator))));
            
            // add associations info for annotating prediction lines on simulator
            associations.push_back(meas_id);
            sense_x.push_back(meas_x);
            sense_y.push_back(meas_y);

        }
        
        // update weight with calculated weight as particle posterior
        particles[p].weight = mv_weight;
        weights[p] = mv_weight;
        
        // set associations for annotation in simulator
        SetAssociations(particles[p], associations, sense_x, sense_y);
    }
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // probabilty of selectiing sample is based on value of weight
    // 1 1 2 4 - sum list = 8, so 1 in 8  chance, but 4 in 8 of 4
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    
    vector<Particle> resample_particles;
    resample_particles.resize(num_particles);
    
    // loop through all particles
    for (int i = 0; i < num_particles; i++) {
        
        resample_particles[i] = particles[distribution(gen)];
    }
    // update all particles
    particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    // particle: the particle to assign each listed association, and association's (x,y) map coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    
    // clear associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();
    
    // update associations with passed in values
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    
    // return updated particle
    return particle;
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

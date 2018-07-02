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
	//DONE: TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;
	num_particles = 20;
	// DONE :TODO: Create normal distributions for y and theta 
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i =0; i< num_particles; i++){
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;

		particles.push_back(p);
		weights.push_back(1);
	};	
	is_initialized = true;
	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
 	default_random_engine gen;	
	// TODO: Create normal distributions for y and theta
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
	for(int i =0; i < num_particles; i++){
		double last_x = particles[i].x;
		double last_y = particles[i].y;
		double last_theta = particles[i].theta;

		double new_x;
		double new_y;
		double new_theta;
		
		if(yaw_rate  == 0){
			new_x = last_x + velocity * delta_t * cos(last_theta);
			new_y = last_y + velocity * delta_t * sin(last_theta);
			new_theta = last_theta;
		}else{
			new_x = last_x + velocity/yaw_rate * (sin(last_theta+ yaw_rate * delta_t)- sin(last_theta));
			new_y = last_y + velocity/yaw_rate * (cos(last_theta) - cos(last_theta + yaw_rate * delta_t) );
			new_theta = last_theta + yaw_rate * delta_t;
		}
		//particles[i].x = last_x + velocity/last_theta * (
		//	sin(last_theta + yaw_rate *delta_t) - sin(yaw_rate));
		//particles[i].y = last_y + velocity/last_theta * (
		//	cos(yaw_rate) -cos(last_theta + yaw_rate * delta_t ));	
		//particles[i].theta = last_theta + yaw_rate * delta_t;

		particles[i].x = new_x +dist_x(gen);	
		particles[i].y = new_y +  dist_y(gen);
		particles[i].theta = new_theta + dist_theta(gen);	
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	LandmarkObs obs;
	for(int i =0; i< observations.size();i++){
		obs = observations[0];
		double last_distance = numeric_limits<double>::max();
		int id = -1;
		//find min distance
		for(int j =0; j< predicted.size();j++){
			double distance =
				pow(predicted[j].x - observations[i].x,2) 
				+ 
				pow(predicted[j].y - observations[i].y,2);
				
			if(distance < last_distance){
				id = predicted[j].id;
				last_distance = distance;				
			}
		}
		observations[i].id = id;
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
	double range = std_landmark[0];	
	double beraing = std_landmark[1];
	for(int p_num =0; p_num < num_particles; p_num++){
		double x = particles[p_num].x;
		double y = particles[p_num].y;
		double theta = particles[p_num].theta;

		double sensor_range_2 = sensor_range * sensor_range;
		vector<LandmarkObs> rangedLandmarks;
		//find in ranged landmarks
		for(int r_index = 0; r_index < map_landmarks.landmark_list.size();r_index++){
			float map_x = map_landmarks.landmark_list[r_index].x_f;
			float map_y = map_landmarks.landmark_list[r_index].y_f;
			double distance_x =  x - map_x;
			double distance_y = y - map_y;
			if((distance_x* distance_x + distance_y * distance_y )< sensor_range_2){
				rangedLandmarks.push_back(LandmarkObs{ 
					map_landmarks.landmark_list[r_index].id_i,
					map_x,
					map_y,
				});
			}			
		}

		//transform observation
		std::vector<LandmarkObs> transform_obs_vector;
		//LandmarkObs obs;
		for(int i =0 ; i < observations.size(); i++){
			//LandmarkObs transform_obs;
			//obs = observations[i];			
			//transform_obs.x = 
			//transform_obs.y = y + (obs.x * sin(theta) + obs.y * cos(theta));
			//transform_obs.id = obs.id;
			transform_obs_vector.push_back(LandmarkObs{
				observations[i].id,
				x + (observations[i].x * cos(theta) - observations[i].y * sin(theta)),
				y + (observations[i].x * sin(theta) + observations[i].y * cos(theta)),
			});
		};	
		dataAssociation(rangedLandmarks,transform_obs_vector);
		
		//refresh weight
		particles[p_num].weight = 1;
		for(int t_index = 0;t_index < transform_obs_vector.size(); t_index++){
			double t_x = transform_obs_vector[t_index].x;
			double t_y = transform_obs_vector[t_index].y;

			//int found_index = -1;
			double d_x = 0;
			double d_y = 0;
			double f_x, f_y;

			for(int f_index =0; f_index < rangedLandmarks.size();f_index++){
				if(rangedLandmarks[f_index].id == transform_obs_vector[t_index].id){
					f_x = rangedLandmarks[f_index].x;
					f_y = rangedLandmarks[f_index].y;
					break;				
					//break;
				}
			}
			d_x = t_x - f_x;
			d_y = t_y - f_y;
			
			double weight =  ( 1/(2*M_PI*range*beraing)) * exp( -( d_x*d_x/(2*range*range) + (d_y*d_y/(2*beraing*beraing))));			
			if(weight ==0){
				particles[p_num].weight *= 0.00001;
			}else{
				particles[p_num].weight *= weight;
			}
			
		    //cout << "WEIGHT = " << weight << endl;
			/*if(weight ==0){
				particles[p_num].weight *= weight;			
			}else{
				particles[p_num].weight *= weight;			

			}			*/
		}
		//weights[p_num] *= particles[p_num].weight;
		//cout<< "set weights" << endl;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Get weights and max weight.   
	default_random_engine gen;
  	vector<double> weights;
  	double max_weight = numeric_limits<double>::min();
  	for(int i = 0; i <  particles.size(); i++) {
    	weights.push_back(particles[i].weight);
    	if ( particles[i].weight > max_weight ) {
      		max_weight = particles[i].weight;
    	}
  }

  // Creating distributions.
  uniform_real_distribution<double> dist_weight(0.0, max_weight * 2);
  uniform_int_distribution<int> dist_praticle(0, num_particles - 1);

  // Generating index.
  int index = dist_praticle(gen);

  double beta = 0.0;

  // the wheel
  vector<Particle> resampledParticles;
  for(int i = 0; i < num_particles; i++) {
    beta += dist_weight(gen);
    while( beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
  }

  particles = resampledParticles;
  /*
   default_random_engine gen;
   vector<double> weights_new;
  // Creating particle distributions.
  uniform_int_distribution<int> particle_index(0, num_particles - 1);
  cout << "resample" << endl;

  double maxWeight = numeric_limits<double>::min();
  for(int i = 0; i < num_particles; i++) {
    weights_new.push_back(particles[i].weight);
    if ( particles[i].weight > maxWeight ) {
      maxWeight = particles[i].weight;
    }
  }
  int index = particle_index(gen);

  uniform_real_distribution<double> random_weight(0, maxWeight);

  double beta = 0;
 
  vector<Particle> resampled_particles;
  cout << "maxweight = "<< maxWeight << endl;

  //get max weight
  //double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());
  for (int i = 0; i < particles.size(); i++) {
	  beta += random_weight(gen) * 2;
	cout << "STARTWHILE" << endl;
	  while (beta > weights_new[index]) {
	    beta -= weights_new[index];
	    index = (index + 1) % num_particles;
	  }
    cout << "ENDWHILE" << endl;

	  resampled_particles.push_back(particles[index]);
	}
	particles = resampled_particles;
 */

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
	//clear
	particle.sense_x.clear();
	particle.sense_y.clear();
	particle.associations.clear();
	//copy
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

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

/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  num_particles = 4000;  // TODO: Set the number of particles

  std::default_random_engine gen;

  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> distribution_x(x, std[0]);
  normal_distribution<double> distribution_y(y, std[1]);
  normal_distribution<double> distribution_theta(theta, std[2]);


  //initializing 3 particales around this reading
   for (int i = 0; i < num_particles; ++i)
   {
      // Sample from these normal distributions
		particles.push_back(Particle());
		weights.push_back(1.0);

		particles[i].id		= i;
		particles[i].x		= distribution_x(gen);
		particles[i].y		= distribution_y(gen);
		particles[i].theta	= distribution_theta(gen);
		particles[i].weight 	= 1.0;
   }
  is_initialized = true;
std::cout << "end init"<< std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
	double yaw_dt = yaw_rate*delta_t;
std::cout << "predict_start"<< std::endl;
	for (int i = 0; i < num_particles; ++i)
	   {
	      // Sample from these normal distributions
	//	double init_x = particles[i].x;
	//	double init_y = particles[i].y;
		double init_theta = particles[i].theta;

			particles[i].x		+= (velocity/yaw_rate)*(sin(init_theta+yaw_dt)-sin(init_theta));
			particles[i].y		+= (velocity/yaw_rate)*(cos(init_theta)-cos(init_theta+yaw_dt));
			particles[i].theta	+= (yaw_dt);

		//Add noise for all particales around this new measurement

		std::default_random_engine gen;
		normal_distribution<double> distribution_x(particles[i].x, std_pos[0]);
		normal_distribution<double> distribution_y(particles[i].y, std_pos[1]);
		normal_distribution<double> distribution_theta(particles[i].theta, std_pos[2]);

			particles[i].x		= distribution_x(gen);
			particles[i].y		= distribution_y(gen);
			particles[i].theta	= distribution_theta(gen);
	   }
std::cout << "predict+end"<< std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

	// Transformation for each observation for each particle
	// Then Association for this Transformed observation
	// Then calculate weight(probability) of this observation
	// Then calculate overall weight of this particle
	std::vector<LandmarkObs> T_Obs;
std::cout << "start update"<< std::endl;
	for (int i = 0; i < num_particles; ++i)
	   {

		for (int j = 0; j < observations.size(); ++j)
		   {		
				T_Obs.push_back(LandmarkObs());
				//################################################################
				//x_y Map are the observation in MAP coordinates
				T_Obs[j].id= observations[j].id;
				T_Obs[j].x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
				T_Obs[j].y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);

				//################################################################
				//Associate above new transformed observation to NearEST landmark
				int closest_landmark = 0;
				int min_dist = 999999;
				int curr_dist;
				// Iterate through all landmarks to check which is closest
				for (int k = 0; k < map_landmarks.landmark_list.size() ; ++k)
				{
					  // Calculate Euclidean distance
					  curr_dist = sqrt(pow(T_Obs[j].x - map_landmarks.landmark_list[k].x_f, 2)
								   + pow(T_Obs[j].y - map_landmarks.landmark_list[k].y_f, 2));
					  // Compare to min_dist and update if closest
					  if (curr_dist < min_dist)
					  {
						min_dist = curr_dist;
						closest_landmark = k;
					  }
				}
				particles[i].associations.push_back(map_landmarks.landmark_list[closest_landmark].id_i) ;
				particles[i].sense_x.push_back(T_Obs[j].x);
				particles[i].sense_y.push_back(T_Obs[j].y);
				//################################################################
				// Calculating weight of this observation with respect to the ith particle
				double mean_x = map_landmarks.landmark_list[closest_landmark].x_f;
				double mean_y = map_landmarks.landmark_list[closest_landmark].y_f;
				double distance_x = T_Obs[j].x;
				double distance_y = T_Obs[j].y;
				double mark_std_x = std_landmark[0];
				double mark_std_y = std_landmark[1];

				float exponent = (pow(distance_x - mean_x,2)/(2.0*pow(mark_std_x,2))) +
								(pow(distance_y - mean_y,2)/(2.0*pow(mark_std_y,2)));

				float denominator = 1.0/(2.0*M_PI*mark_std_x*mark_std_y);

				float Prob = denominator*exp(-exponent) ;

				particles[i].weight  *=Prob;
		    }
		//###################################################################
		// storing overall weight of particles after reviewing all observation inorder to use it in resampling
		weights[i] = particles[i].weight;
	   }

std::cout << "end update"<< std::endl;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
std::cout << "start resample"<< std::endl;
	std::vector<Particle> resampled_particles;
	std::default_random_engine gen;
	normal_distribution<double> random_distr(0.5, 0.5);

	int index = std::rand()% num_particles;
	double max_weight = *std::max_element(weights.begin(),weights.end());
while (max_weight == 0)
{
 //do nothing
}
	double beta = 0.0;
	for (int i = 0; i < num_particles; ++i)
	{
std::cout << "Max_Weight = "<<max_weight<< std::endl;
std::cout << "random_distr = "<<random_distr(gen)<< std::endl;
		beta += double(2.0 * random_distr(gen) * max_weight);
std::cout << "Beta = "<<beta<< std::endl;
		while (beta > weights[index])
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}

		resampled_particles.push_back(particles[i]);
std::cout << "number of resampled = "<<resampled_particles.size()<< std::endl;
	}

	particles = resampled_particles;
std::cout << "End resample"<< std::endl;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

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
// #define DEBUGGING_ENABLED_UPDATE_1
#define DEBUGGING_ENABLED_RESAMPLE_2

double weights_sum=0.0;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).s
   *
   */

  num_particles = 250;  // TODO: Set the number of particles

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
   *  http://www.cplusplus.com/reference/random/default_random_engin/e/
   */
//	double yaw_dt = yaw_rate*delta_t;
//	//double init_x;
//	//double init_y;
//	double init_theta;
//
//	for (int i = 0; i < num_particles; ++i)
//	   {
//#ifdef DEBUGGING_ENABLED_PRED
//std::cout << " Speed = " << velocity << " yaw_rate = " << yaw_rate << std::endl;
//std::cout << " init_x = " << particles[i].x << " init_y = " << particles[i].y << " init_Theta = " << particles[i].theta << std::endl;
//#endif
//	      // Sample from these normal distributions
//		 //init_x = particles[i].x;
//		 //init_y = particles[i].y;
//		 init_theta = particles[i].theta;
//
//		 if (fabs(yaw_rate) > 0.0001)
//		 {
//			particles[i].x		+= (velocity/yaw_rate)*(sin(init_theta+yaw_dt)-sin(init_theta));
//			particles[i].y		+= (velocity/yaw_rate)*(cos(init_theta)-cos(init_theta+yaw_dt));
//			particles[i].theta	+= (yaw_dt);
//		 }
//		 else
//		 {
//			 particles[i].x		+= (velocity)*delta_t*cos(init_theta);
//			 particles[i].y		+= (velocity)*delta_t*sin(init_theta);
//		 }
//		//Add noise for all particales around this new measurement
//
//		std::default_random_engine gen;
//		normal_distribution<double> distribution_x(particles[i].x, std_pos[0]);
//		normal_distribution<double> distribution_y(particles[i].y, std_pos[1]);
//		normal_distribution<double> distribution_theta(particles[i].theta, std_pos[2]);
//
//			particles[i].x		= distribution_x(gen);
//			particles[i].y		= distribution_y(gen);
//			particles[i].theta	= distribution_theta(gen);
//
//#ifdef DEBUGGING_ENABLED_PRED
//std::cout << " pred_x = " << particles[i].x << " pred_y = " << particles[i].y << std::endl;
//#endif
//	   }
	std::default_random_engine gen;

	normal_distribution<double> N_x(0, std_pos[0]);
	normal_distribution<double> N_y(0, std_pos[1]);
	normal_distribution<double> N_theta(0, std_pos[2]);

	for(int i = 0 ; i < num_particles ; i++)
	{
		//avoid division by zero
		if (fabs(yaw_rate) > 0.00001) {
			particles[i].x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta = particles[i].theta + yaw_rate * delta_t;
		} else {
			particles[i].x  = particles[i].x  + velocity * delta_t * cos(particles[i].theta);
			particles[i].y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
		}

		particles[i].x += N_x(gen);
		particles[i].y += N_y(gen);
		particles[i].theta += N_theta(gen);
}

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
	double T_Obs_x;
	double T_Obs_y;
	int T_Obs_id;
	weights_sum = 0.0;
	double highest_weight = -1.0;
	int best_particle;

	for (int i = 0; i < num_particles; ++i)
	   {
		particles[i].weight = 1.0;
		particles[i].associations.clear();
		particles[i].sense_x.clear();
		particles[i].sense_y.clear();

		for (int j = 0; j < observations.size(); ++j)
		   {
				//T_Obs.push_back(LandmarkObs());
				//################################################################
				//x_y Map are the observation in MAP coordinates
				T_Obs_id= observations[j].id;
				T_Obs_x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
				T_Obs_y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);

				//################################################################
				//Associate above new transformed observation to NearEST landmark
				int closest_landmark = 0;
				double min_dist = 999999;
				double curr_dist;
				double mark_dist;
				// Iterate through all landmarks to check which is closest
				for (int k = 0; k < map_landmarks.landmark_list.size() ; ++k)
				{
					//check if landmark is within distance of particle
					mark_dist = sqrt(pow(particles[i].x - map_landmarks.landmark_list[k].x_f, 2)
												   + pow(particles[i].y - map_landmarks.landmark_list[k].y_f, 2));

					if (mark_dist <= sensor_range)
					{
					  // Calculate Euclidean distance
					  curr_dist = sqrt(pow(T_Obs_x - map_landmarks.landmark_list[k].x_f, 2)
								   + pow(T_Obs_y - map_landmarks.landmark_list[k].y_f, 2));

					  // Compare to min_dist and update if closest
					  if (curr_dist < min_dist)
					  {
						min_dist = curr_dist;
						closest_landmark = k;
					  }
					}
				}
				particles[i].associations.push_back(map_landmarks.landmark_list[closest_landmark].id_i) ;
				particles[i].sense_x.push_back(T_Obs_x);
				particles[i].sense_y.push_back(T_Obs_y);
				//################################################################
				// Calculating weight of this observation with respect to the ith particle
				double mean_x = map_landmarks.landmark_list[closest_landmark].x_f;
				double mean_y = map_landmarks.landmark_list[closest_landmark].y_f;
				double distance_x = T_Obs_x;
				double distance_y = T_Obs_y;
				double mark_std_x = std_landmark[0];
				double mark_std_y = std_landmark[1];

				double exponent = (pow(distance_x - mean_x,2)/(2.0*pow(mark_std_x,2))) +
								(pow(distance_y - mean_y,2)/(2.0*pow(mark_std_y,2)));

				double denominator = 1.0/(2.0*M_PI*mark_std_x*mark_std_y);

				double Prob = denominator*exp(-exponent) ;

				particles[i].weight  *=Prob;
		    }
		//###################################################################
		// storing overall weight of particles after reviewing all observation inorder to use it in resampling
		weights[i] = particles[i].weight;
		weights_sum += particles[i].weight;

#ifdef DEBUGGING_ENABLED_UPDATE_1
		  if (particles[i].weight > highest_weight) {
		             highest_weight = particles[i].weight;
		              best_particle = i;
		         }
#endif
	   }

#ifdef DEBUGGING_ENABLED_UPDATE_1
std::cout << "ID  = "<< particles[best_particle].id << std::endl;
std::cout << " highest Weight = "<<(weights[best_particle])<< std::endl;
std::cout << "Normalized highest Weight = "<<(weights[best_particle]/weights_sum)<< std::endl;
std::cout << " highest Weight = "<<(weights[best_particle])<< std::endl;
std::cout << "association  = "<< particles[best_particle].associations.size() << std::endl;

#endif


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	std::vector<Particle> resampled_particles;

	double max_weight = *std::max_element(weights.begin(),weights.end());

	std::default_random_engine gen;
	std::uniform_real_distribution<double> random_distr(0.0, max_weight);
	std::uniform_int_distribution<int> index_random_distr(0.0, (num_particles-1));

	int index = index_random_distr(gen);
	double beta = 0.0;


	resampled_particles.clear();

			for (int i = 0; i < num_particles; ++i)
			{
				beta += 2.0 * random_distr(gen);
				while (beta > (weights[index]/*/weights_sum*/))
				{
					beta -= (weights[index]/*/weights_sum*/);
					index = (index + 1) % num_particles;
				}

				resampled_particles.push_back(particles[index]);

			}

#ifdef DEBUGGING_ENABLED_RESAMPLE_3
std::cout << "End resample"<< std::endl;

    double highest_old_weight = -1.0;
    double weight_resampled_sum = 0.0;
    double highest_resampled_weight = -1.0;
    double weight_old_sum = 0.0;
   for (int i = 0; i < num_particles; ++i) {
     if (particles[i].weight > highest_old_weight) {
         highest_old_weight = particles[i].weight;
         }
     if (resampled_particles[i].weight > highest_resampled_weight) {
         highest_resampled_weight = resampled_particles[i].weight;

         }
     weight_resampled_sum += resampled_particles[i].weight;
     weight_old_sum += particles[i].weight;
     }
   if ((weight_resampled_sum/num_particles) <= (weight_old_sum/num_particles))
   {
	   while(1){}
   }

          std::cout << "highest old w " << highest_old_weight << "...... average old w " << weight_old_sum/num_particles << std::endl;
          std::cout << "highest res w " << highest_resampled_weight << "...... average res w " << weight_resampled_sum/num_particles << std::endl;
#endif

          particles = resampled_particles;

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

/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *
 */

#include "./particle_filter.h"

#include <math.h>

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <iterator>



using std::normal_distribution;
using std::numeric_limits;
using std::vector;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::string;
using std::stringstream;
using std::ostream_iterator;

#define ALMOST_ZERO 0.00001

// @param x     GPS provided x position
// @param y     GPS provided y position
// @param theta GPS provided yaw
// inspired by
// Lesson 14: Implementation of Particle Filter,
// 6. Program Gassian Sampling: Code Solution
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  if (is_initialized) {
    return;
  }

  // Extracting standard deviations
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // get the normal distributions
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // initialize the particles
  for (int i = 0; i < num_particles; ++i) {
    Particle particle;
    particle.x = dist_x(gen);
    particle.y  = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    particles.push_back(particle);
  }

  is_initialized = true;
}

// inspired by
// Lesson 14: Implementation of Particle Filter,
// 8. Calculate Prediction Step: Quiz
void ParticleFilter::prediction(double delta_t,
                                double std_pos[],
                                double velocity,
                                double yaw_rate) {
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // Extracting standard deviations
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  // Creating normal distributions
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  // Calculate new state.
  for (int i = 0; i < num_particles; i++) {
    double theta = particles[i].theta;

    if ( fabs(yaw_rate) < ALMOST_ZERO ) {  // When yaw is not changing.
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
      // yaw continue to be the same.
    } else {
      particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // Adding noise.
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}


// side effect: updated id of nearest landmark for each observation
// inspired by
// Lesson 14: Implementation of Particle Filter,
// 11. Nearest Neighbor Advantages and Disadvantages
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  unsigned int nObservations = observations.size();
  unsigned int nPredictions = predicted.size();

  for (unsigned int i = 0; i < nObservations; i++) {
    // Initialize min distance as a really big number.
    double minDistance = numeric_limits<double>::max();

    // Initialize the found map in something not possible.
    int mapId = -1;

    for (unsigned j = 0; j < nPredictions; j++) {
      double xDistance = observations[i].x - predicted[j].x;
      double yDistance = observations[i].y - predicted[j].y;

      // we skip the sqrt calculation
      double distance = xDistance * xDistance + yDistance * yDistance;

      // If the "distance" is less than min, stored the id and update min.
      if ( distance < minDistance ) {
        minDistance = distance;
        mapId = predicted[j].id;
      }
    }

    // Update the observation identifier.
    observations[i].id = mapId;
  }
}


// inspired by
// Lesson 14: Implementation of Particle Filter,
// 12. Update Step - 19. Particle Weights
void ParticleFilter::updateWeights(double sensor_range,
                                   double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html

  // Extracting standard deviations of landmark positions
  double stdLandmarkX = std_landmark[0];
  double stdLandmarkY = std_landmark[1];

  // for each particle...
  for (int i = 0; i < num_particles; i++) {
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    // Find landmarks in in the map that are in the particle's sensor range.
    double sensor_range_2 = sensor_range * sensor_range;
    vector<LandmarkObs> inRangeLandmarks;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      float landmarkX = map_landmarks.landmark_list[j].x_f;
      float landmarkY = map_landmarks.landmark_list[j].y_f;
      int id = map_landmarks.landmark_list[j].id_i;
      double dX = x - landmarkX;
      double dY = y - landmarkY;
      if ( dX*dX + dY*dY <= sensor_range_2 ) {
        inRangeLandmarks.push_back(LandmarkObs{ id, landmarkX, landmarkY });
      }
    }

    // Transform observation coordinates (measured in particle's coordinate system) to map coordinates.
    // for formula see: https://www.miniphysics.com/coordinate-transformation-under-rotation.html
    vector<LandmarkObs> mappedObservations;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double mappedX = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double mappedY = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      mappedObservations.push_back(LandmarkObs{ observations[j].id, mappedX, mappedY });
    }

    // updates id of nearest landmark for each observation
    dataAssociation(inRangeLandmarks, mappedObservations);

    // Reseting weight.
    particles[i].weight = 1.0;
    // Calculate weights.
    for (unsigned int j = 0; j < mappedObservations.size(); j++) {
      double observationX = mappedObservations[j].x;
      double observationY = mappedObservations[j].y;

      // get x,y of the nearest landmark
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

      // Multivariate-Gaussian Probability
      double weight = (1/(2*M_PI*stdLandmarkX*stdLandmarkY)) * exp(-(dX*dX/(2*stdLandmarkX*stdLandmarkX) + (dY*dY/(2*stdLandmarkY*stdLandmarkY))));
      if (weight == 0) {
        // avoid weight of zero
        particles[i].weight *= ALMOST_ZERO;
      } else {
        particles[i].weight *= weight;
      }
    }
  }
}


// inspired by
// Lesson 13: Particle Filter,
// 20. Quiz: Resample Wheel
void ParticleFilter::resample() {
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // initial index is retrieved by random.
  uniform_int_distribution<int> distInt(0, num_particles - 1);
  int index = distInt(gen);

  double beta = 0.0;



  // Get weights and max weight.
  vector<double> weights;
  double maxWeight = numeric_limits<double>::min();
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if (particles[i].weight > maxWeight) {
      maxWeight = particles[i].weight;
    }
  }

  // Creating distributions.
  uniform_real_distribution<double> distDouble(0.0, maxWeight);

  // the wheel
  vector<Particle> resampledParticles;
  for (int i = 0; i < num_particles; i++) {
    beta += distDouble(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
  }

  particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle,
                                         const std::vector<int>& associations,
                                         const std::vector<double>& sense_x,
                                         const std::vector<double>& sense_y) {
  // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  // Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

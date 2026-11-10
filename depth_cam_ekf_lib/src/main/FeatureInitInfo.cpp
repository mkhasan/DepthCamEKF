//
// Created by mecasys on 11/10/26.
//

#include "FeatureInitInfo.h"

depth_cam_ekf::Particle::Particle(const Eigen::VectorXd &l, double p, const int measurement_size)
{
    lambda_ = l;
    probability_ = p;

    m_z_.resize(measurement_size);
    m_h_.resize(measurement_size);
    m_SInv_.resize(measurement_size, measurement_size);
}

depth_cam_ekf::Particle::~Particle()
{
}

// Set the current innovation covariance \f$ S_i^{-1} \f$ for this
// particle. Called from
// Scene_Single::predict_partially_initialised_feature_measurements()
void depth_cam_ekf::Particle::set_S(const Eigen::MatrixXd &Si)
{
    Eigen::LLT<Eigen::MatrixXd>  local_Si_cholesky(Si);
    Eigen::MatrixXd S_L = local_Si_cholesky.matrixL();
    Eigen::MatrixXd S_Linv = S_L.inverse();

    m_SInv_ = S_Linv.transpose() * S_Linv;
    m_detS_ = Si.determinant();
}

depth_cam_ekf::FeatureInitInfo::FeatureInitInfo(Feature *fp, const int particle_dimension,
                                                const int measurement_size) : fp_(fp)
{
    kParticleDimension_ = particle_dimension;
    kMeasurementSize_ = measurement_size;
    number_of_match_attempts_ = 0;

    mean_.resize(particle_dimension);
    covariance_.resize(particle_dimension, particle_dimension);
}

depth_cam_ekf::FeatureInitInfo::~FeatureInitInfo()
{
}

// Create a particle and add it to this partially-initialised feature's list of
// particles.
// @param lambda The value(s) of the free parameters represented by this particle.
// @param probability The initial probability to assign to the particle.
void depth_cam_ekf::FeatureInitInfo::add_particle(const Eigen::VectorXd &lambda, double probability)
{
    Particle  p(lambda, probability, kMeasurementSize_);
    particle_vector_.push_back(p);
}

// Normalise the probabilities of the particles (divide them by the
// total probability so that they sum to one). This function sets each
// Particle::probability to its normalised value, and also sets
// Particle::cumulative_probability to the total probability of all the particles
// up to and including the current one (after normalisation).
// @returns <code>false</code> if the total probability is zero, <code>true</code>
// otherwise.
bool depth_cam_ekf::FeatureInitInfo::normalise_particle_vector_and_calculate_cumulative()
{
    double  total = 0.0;

    for (std::vector<Particle>::iterator it = particle_vector_.begin();
        it != particle_vector_.end(); ++it) {
        total += it->probability_;
    }

    if (total == 0.0)
        return  false;

    double  cumulative_total = 0.0;

    for (std::vector<Particle>::iterator it = particle_vector_.begin();
        it != particle_vector_.end(); ++it) {
        it->probability_ = it->probability_ / total;
        it->cumulative_probability_ = cumulative_total + it->probability_;
        cumulative_total += it->probability_;
    }

    return  true;
}

// Prune the vector of particles.
// It is assumed that the particle probabilities are normalised before we do this.
// This function remove particles with probability below
// prune_probability_threshold * (1/N) and then normalises again.
// @param prune_probability_threshold The threshold to use.
void depth_cam_ekf::FeatureInitInfo::prune_particle_vector(const double prune_probability_threshold)
{
    double  prune_threshold = prune_probability_threshold / double(particle_vector_.size());

    std::vector<Particle>::iterator it = particle_vector_.begin();

    while (it != particle_vector_.end()) {
        if (it->probability_ < prune_threshold) {
        it = particle_vector_.erase(it);
        }
        else {
        ++it;
        }
    }

    normalise_particle_vector_and_calculate_cumulative();
}

// Calculate the mean and covariance of \f$ \lambda \f$ over all the particles,
// i.e.
// \f{align}
//   \text{mean} &= \mu = \sum_i \lambda_i p(i) \nonumber
//   \text{mean} &= \sum_i \lambda_i\lambda_i^T p(i) - \mu\mu^T \nonumber
// \f}
// The result is not returned, but is instead stored in the class to be read using
// get_mean() and get_covariance().
void depth_cam_ekf::FeatureInitInfo::calculate_mean_and_covariance()
{
    // Vector which will store expected value of lambda * lambda^T
    // (allows us to do this calculation with one pass through particles)
    Eigen::MatrixXd ExpectedSquared(kParticleDimension_, kParticleDimension_);
    ExpectedSquared.setZero();

    // Zero mean vector before filling it up
    mean_.setZero();

    for (std::vector<Particle>::iterator it = particle_vector_.begin();
        it != particle_vector_.end(); ++it) {
        mean_ += it->probability_ * it->lambda_;

        // (it->lambda_ * it->lambda_) is outer product in Eigen3?
        ExpectedSquared += it->probability_ * (it->lambda_ * it->lambda_);
    }

    // (mean_ * mean_) is outer product in Eigen3?
    covariance_ = ExpectedSquared - (mean_ * mean_);
}

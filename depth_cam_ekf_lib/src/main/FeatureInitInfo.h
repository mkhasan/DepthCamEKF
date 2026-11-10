//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_FEATUREINITINFO_H
#define DEPTHCAMEKF_FEATUREINITINFO_H

#include "Feature.h"
namespace depth_cam_ekf {
    class Particle {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Particle(const Eigen::VectorXd &l, double p, const int measurement_size);
        ~Particle();

        void set_S(const Eigen::MatrixXd &Si);

        // The value(s) of the free parameters represented by this particle
        Eigen::VectorXd lambda_;
        // The probability of this particle
        double  probability_;
        // Cumulative probability of all particles up to and including this one
        double  cumulative_probability_;

        // Measurement information used for updating particle
        // The measurement of the feature state \f$ z_i \f$. (e.g. the image location)
        Eigen::VectorXd m_z_;
        // The current predicted measurement state \f$ h_i \f$ for this particle. (e.g. image location)
        Eigen::VectorXd m_h_;
        // The current inverse innovation covariance \f$ S_i^{-1} \f$ for this particle.
        Eigen::MatrixXd m_SInv_;
        // The determinant of the current innovation covariance \f$ |S_i| \f$
        double  m_detS_;
        // Was the last measurement of this particle successful?
        bool  m_successful_measurement_flag_;
    };

    // Class to hold information about a partially-initialised feature. This maintains
    // a list of particle representing the free parameters \f$ \lambda \f$ in
    // the feature.
    //
    // Particles are added by the user of this class by calling add_particle() (no
    // particles are initially present). Their probabilities are updated externally in
    // Scene_Single, and then the current estimate for \f$ \lambda \f$ can be found by
    // calling normalise_particle_vector_and_calculate_cumulative() followed by
    // calculate_mean_and_covariance(). Optionally (but recommended),
    // prune_particle_vector() can also be called to remove particles with small
    // probabilities.
    // @ingroup Scene
    class FeatureInitInfo {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::vector<Particle>  ParticleVector;

        FeatureInitInfo(Feature *fp, const int particle_dimension, const int measurement_size);
        ~FeatureInitInfo();

        void add_particle(const Eigen::VectorXd &lambda, double probability);
        bool normalise_particle_vector_and_calculate_cumulative();
        void prune_particle_vector(const double prune_probability_threshold);
        void calculate_mean_and_covariance();

        Feature *fp_;

        // Since particles can live in PARTICLE_DIMENSION dimensions,
        // mean of distribution is a vector and covariance is a matrix
        Eigen::VectorXd mean_;
        Eigen::MatrixXd covariance_;

        // Vector of particles representing probability distribution
        ParticleVector  particle_vector_;

        int  number_of_match_attempts_;
        bool  making_measurement_on_this_step_flag_;

        // The dimension the particle distribution will live in
        int kParticleDimension_;
        int kMeasurementSize_;
    };

}


#endif //DEPTHCAMEKF_FEATUREINITINFO_H
//
// Created by mecasys on 11/9/26.
//

#ifndef DEPTHCAMEKF_DEPTHCAMSLAM_H
#define DEPTHCAMEKF_DEPTHCAMSLAM_H

#include "Camera.h"
#include "MotionModel.h"
#include "FeatureModel.h"
#include "Feature.h"
#include "PartFeatureModel.h"
#include "FullFeatureModel.h"
#include "FeatureInitInfo.h"
#include "framegrabber/FrameGrabber.h"

#include <vector>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>


namespace depth_cam_ekf {
    class DepthCamSLAM {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DepthCamSLAM();
        ~DepthCamSLAM();
        static void Init(const std::string &config_path);
        bool GoOneStep(cv::Mat frame, bool save_trajectory, bool enable_mapping);

        Camera            *camera_;
        MotionModel       *motion_model_;
        FullFeatureModel  *full_feature_model_;
        PartFeatureModel  *part_feature_model_;

        FrameGrabber      *frame_grabber_;
        // GraphicTool       *graphic_tool_;
        //
        // Kalman            *kalman_;

        Eigen::VectorXd xv_;
        Eigen::MatrixXd Pxx_;

        // Lists of pointers to feature models, fm_list_ contains all features
        std::vector<Feature *>   feature_list_;
        // selected_fm_list just lists those currently selected for measurement
        std::vector<Feature *>   selected_feature_list_;
        // A separate list here of partially-initialised features with additional
        // probabilistic information. These features are also in the main feature_list.
        std::vector<FeatureInitInfo> feature_init_info_vector_;

        // For saving and drawing the camera trajectory
        std::vector<Eigen::Vector3d> trajectory_store_;

        int           number_of_visible_features_{};
        int           next_free_label_{};
        int           marked_feature_label_{};
        int           total_state_size_{};
        // The size of the most recent vector of measurements
        int           successful_measurement_vector_size_{};

        // Constant-likes
        double        kDeltaT_{};
        int           kNumberOfFeaturesToSelect_{};
        int           kNumberOfFeaturesToKeepVisible_{};
        int           kMaxFeaturesToInitAtOnce_{};
        double        kMinLambda_{};
        double        kMaxLambda_{};
        int           kNumberOfParticles_{};
        double        kStandardDeviationDepthRatio_{};
        int           kMinNumberOfParticles_{};
        double        kPruneProbabilityThreshold_{};
        int           kErasePartiallyInitFeatureAfterThisManyAttempts_{};

        // Corners of box we search for new features: save so we can display it
        int           init_feature_search_ustart_{};
        int           init_feature_search_vstart_{};
        int           init_feature_search_ufinish_{};
        int           init_feature_search_vfinish_{};
        bool          init_feature_search_region_defined_flag_{};

        int           minimum_attempted_measurements_of_feature_{};
        double        successful_match_fraction_{};

        int           uu_{}, vv_{};
        bool          location_selected_flag_{};

        const int     kBoxSize_;
        const double  kNoSigma_;
        const double  kCorrThresh2_;
        const double  kCorrelationSigmaThreshold_;
    };
}
#endif //DEPTHCAMEKF_DEPTHCAMSLAM_H
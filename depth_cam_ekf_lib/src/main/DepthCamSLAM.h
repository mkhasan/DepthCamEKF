//
// Created by mecasys on 11/9/26.
//

#ifndef DEPTHCAMEKF_DEPTHCAMSLAM_H
#define DEPTHCAMEKF_DEPTHCAMSLAM_H

#include "Camera.h"
#include "MotionModel.h"
#include "FeatureModel.h"
#include "Feature.h"
#include "DepthCamFeatureModel.h"
#include "FeatureInitInfo.h"
#include "framegrabber/FrameGrabber.h"
#include  "graphic/GraphicTool.h"
#include "Kalman.h"


#include <vector>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>


namespace depth_cam_ekf {
    class Kalman;
    // Array of the scores we've found for local use
    class FeatureAndScore {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FeatureAndScore() {score = 0; fp = NULL;}
        FeatureAndScore(double s, Feature *f) {score = s; fp = f;}
        double  score;
        Feature *fp;
    };
    class DepthCamSLAM {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DepthCamSLAM();
        ~DepthCamSLAM();
        void Init(const std::string &config_path);
        bool GoOneStep(cv::Mat frame, const std::vector<float>& depths, bool save_trajectory, bool enable_mapping);
        void normalise_state();
        void delete_bad_features();
        void exterminate_features();
        bool delete_feature();
        void construct_total_covariance(Eigen::MatrixXd &M);
        void fill_covariances(const Eigen::MatrixXd &M);
        bool toggle_feature_lab(int lab) {
            return false;
        }

        void mark_feature_by_lab(int lab) {

        }

        int auto_select_n_features(int n);
        bool select_feature(Feature *fp);
        bool deselect_feature(Feature *fp);
        void predict_single_feature_measurements(Feature *sfp);

        bool AutoInitialiseFeature(cv::Mat frame, const std::vector<float>& depths, const Eigen::Vector3d &u);
        bool FindNonOverlappingRegion(Eigen::VectorXd local_u,
                                        int &init_feature_search_ustart,
                                        int &init_feature_search_vstart,
                                        int &init_feature_search_ufinish,
                                        int &init_feature_search_vfinish);
        bool FindNonOverlappingRegionNoPredict(int safe_feature_search_ustart,
                                                int safe_feature_search_vstart,
                                                int safe_feature_search_ufinish,
                                                int safe_feature_search_vfinish,
                                                int &init_feature_search_ustart,
                                                int &init_feature_search_vstart,
                                                int &init_feature_search_ufinish,
                                                int &init_feature_search_vfinish);

        double set_image_selection_automatically(cv::Mat frame, const std::vector<float>& depths,
                                                int ustart,
                                                int vstart,
                                                int ufinish,
                                                int vfinish);

        void find_best_patch_inside_region(const cv::Mat &image,
                                            int *ubest,
                                            int *vbest,
                                            double *evbest,
                                            const int BOXSIZE,
                                            int ustart,
                                            int vstart,
                                            int ufinish,
                                            int vfinish);
        void get_median_depth_around_a_pixel(const std::vector<float>& depths, int width, int height, int uu, int vv, double& med_depth);

        void find_eigenvalues(double A, double B, double C, double *eval1ptr,
                        double *eval2ptr);

        void InitialiseFeature(cv::Mat frame);

        void copy_into_patch(const cv::Mat frame, cv::Mat patch);

        void add_new_feature(cv::Mat patch, const Eigen::VectorXd &y);

        int make_measurements(cv::Mat image, const std::vector<float>& depths);
        bool elliptical_search(const cv::Mat &image,
                                const cv::Mat &patch,
                                const Eigen::Vector2d centre,
                                const Eigen::Matrix2d &PuInv,
                                int *u,
                                int *v,
                                const int uBOXSIZE);
        void successful_measurement_of_feature(Feature *sfp);
        void failed_measurement_of_feature(Feature *sfp);
        bool measure_feature(cv::Mat image, const std::vector<float>& depths, cv::Mat patch, Eigen::VectorXd &z,
                               const Eigen::VectorXd &h, const Eigen::MatrixXd &S);
        void construct_total_state(Eigen::VectorXd &V);
        void construct_total_measurement_stuff(Eigen::VectorXd &nu_tot, Eigen::MatrixXd &dh_by_dx_tot, Eigen::MatrixXd &R_tot);
        void fill_states(const Eigen::VectorXd &V);

        Camera            *camera_;
        MotionModel       *motion_model_;
        DepthCamFeatureModel  *feature_model_;

        FrameGrabber      *frame_grabber_;
        GraphicTool       *graphic_tool_;
        //
        Kalman            *kalman_;

        Eigen::VectorXd xv_;
        Eigen::MatrixXd Pxx_;

        // Lists of pointers to feature models, fm_list_ contains all features
        std::vector<Feature *>   feature_list_;
        // selected_fm_list just lists those currently selected for measurement
        std::vector<Feature *>   selected_feature_list_;


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
        double dd_;
        bool          location_selected_flag_{};

        const int     kBoxSize_;
        const double  kNoSigma_;
        const double  kCorrThresh2_;
        const double  kCorrelationSigmaThreshold_;

    };
}
#endif //DEPTHCAMEKF_DEPTHCAMSLAM_H
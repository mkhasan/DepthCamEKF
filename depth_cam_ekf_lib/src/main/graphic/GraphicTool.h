//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_GRAPHICTOOL_H
#define DEPTHCAMEKF_GRAPHICTOOL_H

#include "Feature.h"


#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

#include <GL/freeglut.h>

namespace depth_cam_ekf {
    class DepthCamSLAM;
    class GraphicTool {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GraphicTool(DepthCamSLAM *depth_cam_slam);
        ~GraphicTool();

        void Draw3dScene(const bool &chk_display_trajectory, const bool &chk_display_3d_features,
                        const bool &chk_display_3d_uncertainties);

        void DrawAR(cv::Mat frame,
                    const bool &chk_rectify_image_display, const bool &chk_display_trajectory,
                    const bool &chk_display_3d_features, const bool &chk_display_3d_uncertainties,
                    const bool &chk_display_2d_descriptors, const bool &chk_display_2d_search_regions,
                    const bool &chk_display_initialisation);

        int Picker(int x, int y, bool threed);

        private:
        void Init();
        void DrawRectifiedAR(cv::Mat frame,
                            const bool &chk_display_trajectory,
                            const bool &chk_display_3d_features,
                            const bool &chk_display_3d_uncertainties);
        void DrawRawAR(cv::Mat frame,
                        const bool &chk_display_2d_descriptors,
                        const bool &chk_display_2d_search_regions,
                        const bool &chk_display_initialisation);

        void DrawCamera(const Eigen::Vector3d &r_w, const Eigen::Quaterniond &q_wr);
        void DrawCameraTrajectory(const std::vector<Eigen::Vector3d> &trajectory_store);
        void DrawEstimatedSemiInfiniteLine(const Eigen::VectorXd &yigraphics,
                                            const double line_length, const int name_to_draw);
        void DrawAxes();
        void DrawDescriptors(double draw_patch_x, double draw_patch_y,
                            const cv::Mat &patch, const int name_to_draw);
        void DrawPatch(const double u, const double v, const cv::Mat &patch);
        void Draw2DPartiallyInitialisedLineEllipses(Feature *fp);
        void Draw2DInitialisationBoxes(const bool location_selected_flag,
                                        const bool init_feature_search_region_defined_flag,
                                        const unsigned int uu, unsigned int vv,
                                        const unsigned int init_feature_search_ustart,
                                        const unsigned int init_feature_search_vstart,
                                        const unsigned int init_feature_search_ufinish,
                                        const unsigned int init_feature_search_vfinish,
                                        const unsigned int BOXSIZE);

        void DrawFrame(const cv::Mat &frame, const double zC, const bool rectified);
        void DrawTexturedRectangle(const Eigen::Vector3d rW,
                                    const Eigen::Quaterniond qWO,
                                    const double rectangle_width,
                                    const double rectangle_height,
                                    const cv::Mat &imageref);
        void DrawDistortedTexturedRectangle(const Eigen::Vector3d rW,
                                            const Eigen::Quaterniond qWO,
                                            const double rectangle_width,
                                            const double rectangle_height,
                                            const cv::Mat &imageref,
                                            double kappa,
                                            const Eigen::Vector2d& distortion_centre);
        unsigned int GetNextPowerOf2(const unsigned int number);
        Eigen::Vector2d UndistortPoint(const double kappa,
                                        const Eigen::Vector2d& source_point,
                                        const Eigen::Vector2d& distortion_centre);

        void DrawCylinder(const double radius, const double height, const bool drawends);
        void DrawCone(const double bottomradius, const double topradius, const double height);
        void DrawCurve(float width, float curveheight, int m, int n);
        void DrawCurveLid(float width, float curveheight, float depth, int m);
        void DrawLine(const Eigen::Vector3d &r0, Eigen::Vector3d &r1);
        void DrawPoint(const Eigen::Vector3d &r0, const int name);
        void DrawCovariance(const Eigen::VectorXd rW, const Eigen::MatrixXd Prr,
                            const double number_of_sigma, const int name);
        void Draw2DCovariance(const double u, const double v, const Eigen::Matrix2d &P,
                                const double number_of_sigma, const double ring_thickness);
        void DrawNumber(const double number);
        void Draw2DRectangle(const double u, const double v, const double width, const double height);

        void RotateDrawingPosition(const Eigen::Quaterniond &qON);
        void SetFeatureColour(const bool selected_flag, const bool successful_measurement_flag, const bool marked_flag);
        void Draw2DBegin(double u, double v);
        void Draw2DEnd();
        void Set2DDrawingPosition(const double u, const double v);
        void SetProjectionMatrix();
        void UpdateDrawingDimension(const Eigen::Vector3d& v);

        bool selection_mode_;
        bool bInitialised;

        cv::Mat *frame_;
        bool chk_rectify_image_display_;
        bool chk_display_trajectory_;
        bool chk_display_3d_features_;
        bool chk_display_3d_uncertainties_;
        bool chk_display_2d_descriptors_;
        bool chk_display_2d_search_regions_;
        bool chk_display_initialisation_;

        GLUquadricObj *sphere_quad_;
        GLUquadricObj *cylinder_quad_;
        GLUquadricObj *circle_quad_;

        DepthCamSLAM  *depth_cam_slam_ptr_;

        double  xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;

        GLuint  texName_;
        int     texWidth_, texHeight_;

        int     clicked_x_, clicked_y_;

        // q in state vector is qWR between world frame and Scene robot frame
        // What we need to plot though uses GL object frame O
        // Know qRO: pi rotation about y axis
        // qWO = qWR * qRO
        const Eigen::Quaterniond  kQR0_;    // (0.0, 0.0, 1.0, 0.0)

        const double  kMoveClippingPlaneFactor_;

        const double  kSemiInfiniteLineLength_;
        const double  kCovariancesNumberOfSigma_;
        const int     kDrawNOverlappingEllipses_;
    };

}


#endif //DEPTHCAMEKF_GRAPHICTOOL_H
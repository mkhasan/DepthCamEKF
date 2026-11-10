//
// Created by mecasys on 11/10/26.
//

#include "SearchMultipleOverlappingEllipses.h"
#include "improc.h"

depth_cam_ekf::SearchMultipleOverlappingEllipses::SearchDatum::SearchDatum(
    const Eigen::Matrix2d &PuInv, const Eigen::Vector2d &search_centre) :
    PuInv_(PuInv), search_centre_(search_centre), result_flag_(false),
    result_u_(0), result_v_(0)
{
    halfwidth_ = depth_cam_ekf::NO_SIGMA / sqrt(PuInv(0,0) - PuInv(0,1) * PuInv(0,1) / PuInv(1,1));
    halfheight_ = NO_SIGMA / sqrt(PuInv(1,1) - PuInv(0,1) * PuInv(0,1) / PuInv(0,0));
}

// Checks whether the point \f$(u,v)\f$ is inside the bounding box of the
// (three standard deviation) ellipse.
// @param u The x-co-ordinate to test
// @param v The x-co-ordinate to test
bool depth_cam_ekf::SearchMultipleOverlappingEllipses::SearchDatum::inside_fast(int u, int v)
{
    u = abs(u - int(search_centre_(0) + 0.5));

    if(u > int(halfwidth_))
        return  false;

    v = abs(v - int(search_centre_(1) + 0.5));

    if(v > int(halfheight_))
        return  false;

    return  true;
}

// Checks whether the point \f$(u,v)\f$ is inside the ellipse (within three
// standard deviations).
// @param u The x-co-ordinate to test
// @param v The x-co-ordinate to test
bool depth_cam_ekf::SearchMultipleOverlappingEllipses::SearchDatum::inside(int u, int v)
{
    u = abs(u - int(search_centre_(0) + 0.5));
    v = abs(v - int(search_centre_(1) + 0.5));

    return  inside_relative(u,v);
}

// Checks whether the point \f$(u,v)\f$ is inside the ellipse (within three
// standard deviations). This version assume that distances are relative to the
// centre of the ellipse, not absolute co-ordinates.
// @param u The horizontal distance from the search centre
// @param v The vertical distance from the search centre
bool depth_cam_ekf::SearchMultipleOverlappingEllipses::SearchDatum::inside_relative(int u, int v)
{
    return  (PuInv_(0,0) * u * u + 2 * PuInv_(0,1) * u * v +
            PuInv_(1,1) * v * v <  NO_SIGMA * NO_SIGMA);
}

// Add a search ellipse to the class.
// @param PuInv The inverse covariance of the feature location. The search will be
// over the ellipse defined by NO_SIGMA standard deviations from the centre of the
// search.
// @param search_centre The centre of the search.
void depth_cam_ekf::SearchMultipleOverlappingEllipses::add_ellipse(const Eigen::Matrix2d &PuInv,
                                                                   const Eigen::Vector2d &search_centre)
{
    m_searchdata_.push_back(SearchDatum(PuInv, search_centre));
}

// Search for the image patch over all of the ellipses registered with the class.
// Since correlation is expensive we locally cache the correlation results so that
// we only do it once at each image location.
void depth_cam_ekf::SearchMultipleOverlappingEllipses::search()
{
    // Rather than working out an overall bounding box, we'll be slightly
    // lazy and make an array for correlation results which is the same size
    // as the image
    // Pixels in this array in raster order

    // Set all positions to impossible correlation value
    cv::Mat correlation_image(m_image_->size(), CV_64FC1, -1.0);

    // Now, we loop over ellipses
    for (SearchData::iterator i = m_searchdata_.begin();
        i != m_searchdata_.end(); ++i) {
        // Limits of search
        int urelstart = -i->halfwidth_;
        int urelfinish = i->halfwidth_;
        int vrelstart = -i->halfheight_;
        int vrelfinish = i->halfheight_;

        int ucentre = int(i->search_centre_(0));
        int vcentre = int(i->search_centre_(1));

        // Check these limits aren't outside the image
        if(ucentre + urelstart - int(m_boxsize_ - 1) / 2 < 0) {
            urelstart = int(m_boxsize_ - 1) / 2 - ucentre;
        }

        if(ucentre + urelfinish - int(m_boxsize_ - 1) / 2 >
            int(m_image_->size().width) - int(m_boxsize_)) {
            urelfinish = int(m_image_->size().width) - int(m_boxsize_) - ucentre +
                        int(m_boxsize_-1) / 2;
        }

        if(vcentre + vrelstart - int(m_boxsize_ - 1) / 2 < 0) {
            vrelstart = int(m_boxsize_ - 1) / 2 - vcentre;
        }

        if(vcentre + vrelfinish - int(m_boxsize_ - 1) / 2 >
        int(m_image_->size().height) - int(m_boxsize_)) {
            vrelfinish = int(m_image_->size().height) - int(m_boxsize_) - vcentre +
                        int(m_boxsize_ - 1) / 2;
        }

        // Search counters
        int urel, vrel;

        double corrmax = 1000000.0;
        double corr;

        // For passing to and_correlate2_warning
        double sdpatch, sdimage;

        // Do the search
        for (urel = urelstart; urel <= urelfinish; ++urel) {
            for (vrel = vrelstart; vrel <= vrelfinish; ++vrel) {
                if(i->inside_relative(urel, vrel)) {
                    // We are inside ellipse
                    // Has this place been searched before?
                    if (correlation_image.at<double>(vcentre + vrel, ucentre + urel) != -1.0) {
                        corr = correlation_image.at<double>(vcentre + vrel, ucentre + urel);
                    }
                    else {
                        corr = correlate2_warning(0, 0, m_boxsize_, m_boxsize_,
                                                ucentre + urel - (int)(m_boxsize_ - 1) / 2,
                                                vcentre + vrel - (int)(m_boxsize_ - 1) / 2,
                                                *m_patch_, *m_image_, &sdpatch, &sdimage);

                        if (sdimage < CORRELATION_SIGMA_THRESHOLD) {
                        corr += LOW_SIGMA_PENALTY;
                        }

                        correlation_image.at<double>(vcentre + vrel, ucentre + urel) = corr;
                    }

                    if (corr <= corrmax) {
                        corrmax = corr;
                        i->result_u_ = urel + ucentre;
                        i->result_v_ = vrel + vcentre;
                    }
                }
            }
        }

        // Threshold correlation score: check if good enough
        if(corrmax > CORRTHRESH2) {
            i->result_flag_ = false;
        }
        else
            i->result_flag_ = true;
    }
}

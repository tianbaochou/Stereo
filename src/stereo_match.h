#ifndef STEREO_STEREO_VISION_STEREO_MATCH_H
#define STEREO_STEREO_VISION_STEREO_MATCH_H
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
class StereoMatch
{
public:
    StereoMatch(){}
    
    void StereoMatching(cv::InputArray rec_image1, cv::InputArray rec_image2,
        cv::OutputArray disparity_map, int min_disparity, int num_disparities, int SAD_window_size,
        int P1, int P2);
};


#endif
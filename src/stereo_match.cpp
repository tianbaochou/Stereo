#include "stereo_match.h"

void StereoMatch::StereoMatching(cv::InputArray rec_image1, cv::InputArray rec_image2,
    cv::OutputArray disparity_map, int min_disparity, int num_disparities, int SAD_window_size,
    int P1, int P2)
{
    cv::Mat img1 = rec_image1.getMat();
    cv::Mat img2 = rec_image2.getMat();
    disparity_map.create(img1.size(), CV_16S);
    cv::Mat dis = disparity_map.getMat();
    
    cv::StereoSGBM matcher(min_disparity, num_disparities, SAD_window_size, P1, P2);
    matcher(img1, img2, dis);
    dis = dis / 16.0;
}


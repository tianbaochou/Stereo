#ifndef STEREO_STEREO_VISION_DISTORTION_RECIFITION_H
#define STEREO_STEREO_VISION_DISTORTION_RECIFITION_H

#include "camera.h"
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>
class DistortionRecifition
{
public:

    DistortionRecifition(){}

    /*
    Set some paramater
    */
    void SetParamter( cv::Mat &left_image, cv::Mat &right_image, int flags,
        double alpha, cv::Size old_image_size, cv::Size new_image_size);

    void Recifition(CameraParam & cp, cv::Rect &left_roi, cv::Rect &right_roi);

    void GetRecifyImage(cv::Mat &left_recify_image, cv::Mat &right_recify_image, int inter_polation = cv::INTER_LINEAR);
private:
    int _flags;
    double _alpha;
    cv::Size _new_image_size;
    cv::Size _old_image_size;
    cv::Mat _left_image;
    cv::Mat _right_image;
    CameraParam _camera_param;
    cv::Mat _map_x1;
    cv::Mat _map_y1;
    cv::Mat _map_x2;
    cv::Mat _map_y2;
};


#endif
#ifndef STEREO_STEREO_VISION_DISTORTION_RECIFITION_H
#define STEREO_STEREO_VISION_DISTORTION_RECIFITION_H

#include "camera.h"
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>

#define PI 3.1415926
enum CameraType{
    CT_NORMAL,
    CT_FISHEYE
};

enum RectifyType{
    RT_NORMAL,
    RT_LONGITUDE_LATITUDE
};

class DistortionRecifition
{
public:

    DistortionRecifition( CameraType camera_type, RectifyType rectify_type ){
        _cameraType = camera_type;
        _rectifyType = rectify_type;
    }

    /*
    Set some paramater
    */
    void SetParamter( cv::Mat &left_image, cv::Mat &right_image, int flags,
        double alpha, cv::Size old_image_size, cv::Size new_image_size);


    void Recifition(CameraParam & cp, cv::Rect &left_roi, cv::Rect &right_roi);

    /*
    Computes the undistortion and rectification transformation map.
    \input camera_matrix:  Camera matrix for device
    \input discoeffs: Camera distortion coeffcients
    \input R:  Rectification rotation matrix
    \input new_camera_matrix:  New camera matrix, unused if rectifytype = RT_LONGITUDE_LATITUDE
    \input undis_size:  Undistorted image size,  which is releated to new_camera_matrix and is unused if RT_LONGITUDE_LATITUDE
    \input mltype:  Input type of map, now can only be CV_32FC1
    \output map1: Output map of the u coordinate of undistorted image to distorted
    \output map2: Output map of the v coordinate of undistorted image to distorted
    */
    void InitUndistortRectifyMap(cv::InputArray camera_matrix, cv::InputArray discoeffs, cv::InputArray R,
        cv::InputArray _new_camera_matrix, cv::Size undis_size, int mltype, cv::OutputArray map1,
        cv::OutputArray map2, cv::Point2f range_longitude = cv::Point2f(0.0, PI), cv::Point2f range_latitude = cv::Point2f(0.0, PI));

    void GetRecifyImage(cv::Mat &left_recify_image, cv::Mat &right_recify_image, int inter_polation = cv::INTER_LINEAR);

    void RectifyImage(cv::InputArray image1, cv::InputArray image2, cv::InputArray map11, cv::InputArray map12,
        cv::InputArray map21, cv::InputArray map22, cv::OutputArray rec_image1, cv::OutputArray rec_image2);

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
    CameraType _cameraType;
    RectifyType _rectifyType;
};


#endif
#include "distortion_recifition.h"

void DistortionRecifition::Recifition(CameraParam & cp, cv::Rect &left_roi, cv::Rect &right_roi)
{

    _camera_param = cp;
// Use opencv stereo module
    cv::stereoRectify(cp._left_camera_matrix, cp._Distortion_left, cp._right_camera_matrix,
        cp._Distortion_right, _old_image_size, cp._R, cp._T, cp._RectificationR_left, cp._RectificationR_right,
        cp._P1, cp._P2, cp._Q, _flags, _alpha, _new_image_size, &left_roi, &right_roi);

}

void DistortionRecifition::SetParamter(cv::Mat &left_image, cv::Mat &right_image, int flags, double alpha, cv::Size old_image_size, cv::Size new_image_size)
{
    _left_image = left_image;
    _right_image = right_image;
    _flags = flags;
    _alpha = alpha;
    _new_image_size = new_image_size;
    _old_image_size = old_image_size;
}

void DistortionRecifition::GetRecifyImage(cv::Mat &left_recify_image, cv::Mat &right_recify_image, 
    int inter_polation)
{
    cv::initUndistortRectifyMap(_left_image, _camera_param._Distortion_left,
        _camera_param._RectificationR_left, _camera_param._P1, _new_image_size,
        CV_32FC1, _map_x1, _map_y1);

    cv::initUndistortRectifyMap(_right_image, _camera_param._Distortion_right,
        _camera_param._RectificationR_left, _camera_param._P2, _new_image_size,
        CV_32FC1, _map_x2, _map_y2);

    cv::remap(_left_image, left_recify_image, _map_x1, _map_y1, inter_polation);
    cv::remap(_right_image, right_recify_image, _map_x2, _map_y2, inter_polation);
}
#include "stereo.h"
#include "stereo_error.h"
#include <iostream>
using std::thread;
using std::mutex;
using std::cout;
bool Stereo::Init()
{
    cv::FileStorage fs("camera_paramter.yml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        throw StereoException("Load the file camera_paramter.yml failed\n");
        return false;
    }
    cv::Mat left_camera_matrix, right_camera_matrix, om, t, left_distortion, right_distortion;
    fs["left_camera_matrix"] >> left_camera_matrix;
    _camera_param._left_camera_matrix = left_camera_matrix;
    fs["right_camera_matrix"] >> right_camera_matrix;
    _camera_param._right_camera_matrix = right_camera_matrix;
    fs["om"] >> om;
    cv::Rodrigues(om, _camera_param._R);
    fs["t"] >> t;
    _camera_param._T = t;
    fs["left_distortion"] >> left_distortion;
    _camera_param._Distortion_left = left_distortion;
    fs["right_distortion"] >> right_distortion;
    _camera_param._Distortion_right = right_distortion;
    _running = false;
    return true;
}

void Stereo::Start()
{
    if (_running)
    {
        std::cout << "The Stereo Thread Already Start\n";
        return;
    }
    _running = true;
    _process_thread = std::make_shared<thread>(&Stereo::Process, this);
}

void Stereo::Process()
{
    while (_running)
    {
        std::lock_guard<mutex> lock(_read_image);
        {
            _left_image_cache.copyTo(_left_image);
            _right_image_cache.copyTo(_right_image);
            if (_left_image.empty() || _right_image.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); //TODO: only for once
                continue;
            }
        }

        // First rectify distortion
        cv::Rect left_roi, right_roi;
        _distor_recifition.SetParamter(_left_image, _right_image, CV_CALIB_ZERO_DISPARITY, 0, cv::Size(_left_image.cols, _left_image.rows), cv::Size(_left_image.cols, _left_image.rows));
        _distor_recifition.Recifition(_camera_param, left_roi, right_roi);
        _distor_recifition.GetRecifyImage(_left_recifition_image, _right_recifition_image);
        //Test
        cv::imshow("left", _left_recifition_image);
        cv::imshow("right", _right_recifition_image);
        char c = cv::waitKey(10);
        if (c == 27)
            break;
    }
}

void Stereo::Stop()
{
    if (_running )
    _running = false;
    _process_thread->join();
}

void Stereo::GrabImage(cv::Mat &left_image, cv::Mat &right_image)
{
    std::lock_guard<mutex> lock(_read_image);
    {
        left_image.copyTo(_left_image_cache);
        right_image.copyTo(_right_image_cache);
    }
}
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
    fs["left_camera_matrix"] >> _camera_param._left_camera_matrix;
    fs["right_camera_matrix"] >> _camera_param._right_camera_matrix;
    fs["om"] >> om;
    cv::Rodrigues(om, _camera_param._R);
    fs["t"] >> _camera_param._T;
    fs["left_distortion"] >> _camera_param._Distortion_left;
    fs["right_distortion"] >> _camera_param._Distortion_right;
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
    // Alter from ...
    _distor_recifition = new DistortionRecifition(CameraType::CT_FISHEYE, RectifyType::RT_LONGITUDE_LATITUDE);
    cv::Mat R1, R2, P1, P2, Q;
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

        cv::Rect left_roi, right_roi;
        _distor_recifition->Recifition(_camera_param, left_roi, right_roi);

        cv::Mat map11, map12, map21, map22;
        cv::Point2f log_range(0 * PI, 1 * PI);
        cv::Point2f lat_range(0 * PI, 1 * PI);
        cv::Mat camera_matrix_new = _camera_param._left_camera_matrix.clone();
        if (camera_matrix_new.type() != CV_32F)
            camera_matrix_new.convertTo(camera_matrix_new, CV_32F);
       
        camera_matrix_new.at<float>(0, 0) = camera_matrix_new.at<float>(0, 0) / 4;
        camera_matrix_new.at<float>(1, 1) = camera_matrix_new.at<float>(1, 1) / 4;
        
        _distor_recifition->InitUndistortRectifyMap(_camera_param._left_camera_matrix, _camera_param._Distortion_left,
            _camera_param._RectificationR_left, camera_matrix_new, _left_image.size(), CV_32F,
            map11, map12, log_range, lat_range);
        
        _distor_recifition->InitUndistortRectifyMap(_camera_param._right_camera_matrix, _camera_param._Distortion_right,
            _camera_param._RectificationR_right, camera_matrix_new, _left_image.size(),
            CV_32F, map21, map22, log_range, lat_range);

        cv::Mat rec_image1, rec_image2;
        _distor_recifition->RectifyImage(_left_image, _right_image, map11, map12, map21, map22,
            rec_image1, rec_image2);
        cv::imshow("rectify image1", rec_image1);
        cv::imshow("rectify image2", rec_image2);

        cv::Mat disparity_map;
        StereoMatch stereo_match;
        stereo_match.StereoMatching(rec_image1, rec_image2, disparity_map,
            0, 100 / 16 * 16, 8, 8 * 5 * 5, 32 * 5 * 5);
        disparity_map.convertTo(disparity_map, CV_8U);
        cv::imshow("disparity", disparity_map);


        // Opencv default 
        //// First   distortion
        //cv::Rect left_roi, right_roi;
        //_distor_recifition->SetParamter(_left_image, _right_image, CV_CALIB_ZERO_DISPARITY, 0, cv::Size(_left_image.cols, _left_image.rows), cv::Size(_left_image.cols, _left_image.rows));
        //_distor_recifition->Recifition(_camera_param, left_roi, right_roi);
        //_distor_recifition->GetRecifyImage(_left_recifition_image, _right_recifition_image);

        ////Test
        //cv::imshow("left", _left_recifition_image);
        //cv::imshow("right", _right_recifition_image);
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
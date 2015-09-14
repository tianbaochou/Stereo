#ifndef STEREO_STEREO_VISION_STEREO_H
#define STEREO_STEREO_VISION_STEREO_H
#include "camera.h"
#include "stereo_match.h"
#include "distortion_recifition.h"
#include <chrono>
#include <mutex>
#include <thread>
#include <memory>
#include <atomic>

class Stereo
{
public:
    Stereo(){}
    bool Init();

    void Start();
    void Stop();

    void GrabImage(cv::Mat &left_image, cv::Mat &rigth_image);

private:
    void Process();
    CameraParam _camera_param;
    DistortionRecifition _distor_recifition;
    std::mutex _read_image;
    std::atomic_bool _running;
    std::shared_ptr<std::thread> _process_thread;
    cv::Mat _left_image;
    cv::Mat _right_image;
    cv::Mat _left_recifition_image;
    cv::Mat _right_recifition_image;
    cv::Mat _left_image_cache;
    cv::Mat _right_image_cache;

};



#endif
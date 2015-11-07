#include "distortion_recifition.h"
#include <iostream>

using cv::Mat;
using std::cout;
using std::endl;
void DistortionRecifition::Recifition(CameraParam & cp, cv::Rect &left_roi, cv::Rect &right_roi)
{
#if 0
    _camera_param = cp;
    // Use opencv stereo module
    cv::stereoRectify(cp._left_camera_matrix, cp._Distortion_left, cp._right_camera_matrix,
        cp._Distortion_right, _old_image_size, cp._R, cp._T, cp._RectificationR_left, cp._RectificationR_right,
        cp._P1, cp._P2, cp._Q, _flags, _alpha, _new_image_size, &left_roi, &right_roi);
#else
    _camera_param = cp;

    Mat K1 = cp._left_camera_matrix, K2 = cp._right_camera_matrix, R = cp._R;
    Mat T = cp._T;
    cp._RectificationR_left.create(3, 3, CV_32F);
    cp._RectificationR_right.create(3, 3, CV_32F);
    Mat R1 = cp._RectificationR_left;
    Mat R2 = cp._RectificationR_right;
    cp._P1.create(3, 4, CV_32F);
    cp._P2.create(3, 4, CV_32F);
    Mat P1 = cp._P1;
    Mat P2 = cp._P2;

    // Convert to float 
    if (K1.type() != CV_32F)
    {
        K1.convertTo(K1, CV_32F);
        cp._left_camera_matrix.convertTo(cp._left_camera_matrix, CV_32F);
    }
    if (K2.type() != CV_32F)
    {
        K2.convertTo(K2, CV_32F);
        cp._right_camera_matrix.convertTo(cp._right_camera_matrix, CV_32F);
    }
    if (R.type() != CV_32F)
    {
        R.convertTo(R, CV_32F);
        cp._RectificationR_left.convertTo(cp._RectificationR_left, CV_32F);
    }
    if (T.type() != CV_32F)
    {
        T.convertTo(T, CV_32F);
        cp._T.convertTo(cp._T, CV_32F);
    }
    if (T.rows != 3)
        T = T.t();

    // 获取第二个相机到第一个相机间的转换
    Mat R_inv = R.inv();
    Mat T_inv = -R.t()*T;
    Mat e1, e2, e3;
    e1 = T_inv.t() / cv::norm(T_inv);
    e2 = (cv::Mat_<float>(1, 3) << -1*T_inv.at<float>(1), T_inv.at<float>(0), 0.0);
    e2 = e2 / (sqrt(e2.at<float>(0)*e2.at<float>(0) + e2.at<float>(1)*e2.at<float>(1)));
    e3 = e1.cross(e2);
    e3 = e3 / cv::norm(e3);
    e1.copyTo(R1.row(0));
    e2.copyTo(R1.row(1));
    e3.copyTo(R1.row(2));
    R2 = R_inv *R1;
   
    
    P1.setTo(cv::Scalar(0));
    R1.copyTo(P1.colRange(0, 3));
    P1 = K1*P1;

    P2.setTo(cv::Scalar(0));
    R2.copyTo(P2.colRange(0, 3));
    P2 = K2*P2;
#endif
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

void DistortionRecifition::RectifyImage(cv::InputArray image1, cv::InputArray image2, cv::InputArray map11, cv::InputArray map12,
    cv::InputArray map21, cv::InputArray map22, cv::OutputArray rec_image1, cv::OutputArray rec_image2)
{
    cv::Mat _img1 = image1.getMat();
    cv::Mat _img2 = image2.getMat();
    cv::Mat _map11 = map11.getMat();
    cv::Mat _map12 = map12.getMat();
    cv::Mat _map21 = map21.getMat();
    cv::Mat _map22 = map22.getMat();

    rec_image1.create(_map11.size(), _img1.type());
    Mat _rec_img1 = rec_image1.getMat();
    rec_image2.create(_map21.size(), _img2.type());
    Mat _rec_img2 = rec_image2.getMat();

    cv::remap(_img1, _rec_img1, _map11, _map12, cv::INTER_LINEAR);
    cv::remap(_img2, _rec_img2, _map21, _map22, cv::INTER_LINEAR);
}

void DistortionRecifition::GetRecifyImage(cv::Mat &left_recify_image, cv::Mat &right_recify_image, 
    int inter_polation)
{
    cv::initUndistortRectifyMap(_camera_param._left_camera_matrix, _camera_param._Distortion_left,
        _camera_param._RectificationR_left, _camera_param._P1, _new_image_size,
        CV_32FC1, _map_x1, _map_y1);

    cv::initUndistortRectifyMap(_camera_param._right_camera_matrix, _camera_param._Distortion_right,
        _camera_param._RectificationR_right, _camera_param._P2, _new_image_size,
        CV_32FC1, _map_x2, _map_y2);

    cv::remap(_left_image, left_recify_image, _map_x1, _map_y1, inter_polation);
    cv::remap(_right_image, right_recify_image, _map_x2, _map_y2, inter_polation);
} 

void DistortionRecifition::InitUndistortRectifyMap(cv::InputArray camera_matrix, cv::InputArray discoeffs, cv::InputArray R,
    cv::InputArray new_camera_matrix, cv::Size undis_size, int mltype, cv::OutputArray map1,
    cv::OutputArray map2, cv::Point2f range_longitude , cv::Point2f range_latitude )
{
    Mat _K = camera_matrix.getMat(), _discoeffs = discoeffs.getMat(), _R = R.getMat(),
        _newK = new_camera_matrix.getMat();

    map1.create(undis_size, mltype);
    map2.create(undis_size, mltype);
    Mat _map1 = map1.getMat();
    Mat _map2 = map2.getMat();

    // [x_u;y_u] is the undistorted point in image plane, r2 = x_u^2 + y_u^2
    // [x_d;y_d] is the distorted point in image plane, then
    // x_d = x_u*(1+k1*r2+k2*r4+k3*r6)/(1+k4*r2+k5*r4+k6*r6)+2*p1*x_u*y_u+p2*(r2+2*x_u*x_u)
    // the proceeding of y_d is similar to x_d

    // Convert to float 
    if (_discoeffs.type() != CV_32F)
    {
        _discoeffs.convertTo(_discoeffs, CV_32F);
    }

    float k1 = _discoeffs.at<float>(0);
    float k2 = _discoeffs.at<float>(1);
    float p1 = _discoeffs.at<float>(2);
    float p2 = _discoeffs.at<float>(3);
    float k3 = _discoeffs.total() >= 5 ? _discoeffs.at<float>(4) : 0;
    float k4 = _discoeffs.total() >= 8 ? _discoeffs.at<float>(5) : 0;
    float k5 = _discoeffs.total() >= 8 ? _discoeffs.at<float>(6) : 0;
    float k6 = _discoeffs.total() >= 8 ? _discoeffs.at<float>(7) : 0;
    
    float fx = _K.at<float>(0, 0);
    float fy = _K.at<float>(1, 1);
    float u0 = _K.at<float>(0, 2);
    float v0 = _K.at<float>(1, 2);

    // inv(R)*inv(newK),which transform a pixel in undistorted image to undistorted coordinate
    // useless if rectifyType == RT_LONGITUDE_LATITUDE
    Mat newKR_inv = (_newK*_R).inv();
    const float * nkri = &newKR_inv.at<float>(0, 0);
    for (int i = 0; i < undis_size.height; i++)
    {
        float *m1f = (float*)(_map1.data + _map1.step*i);
        float *m2f = (float*)(_map2.data + _map2.step*i);
        if (_rectifyType == RT_NORMAL)
        {
            // [_x; _y;  _z] = inv(R)*inv(K)*[u;v;1]
            float _x, _y, _z;
            // [u;v] = [j,i]
            _x = i*nkri[1] + nkri[2];
            _y = i*nkri[4] + nkri[5];
            _z = i*nkri[7] + nkri[8];
            for (int j = 0; j < undis_size.width; j++, _x += nkri[0], _y += nkri[3], _z += nkri[6])
            {
                float x, y;
                if (_cameraType == CT_NORMAL)
                {
                    // Project to image plane [x;y;1] using perspective model
                    x = _x / _z;
                    y = _y / _z; 
                }
                else if (_cameraType == CT_FISHEYE)
                {
                    // Project to image plane[x;y;1] using equal distance model
                    // First project to sphere
                    float x_s = _x / sqrt(_x*_x + _y*_y + _z*_z);
                    float y_s = _y / sqrt(_x*_x + _y*_y + _z*_z);
                    float z_s = _z / sqrt(_x*_x + _y*_y + _z*_z);

                    // On a sphere, x = sin(theta)cos(alpha), y = sin(theta)cos(alpha), z = cos(theta)
                    float theta = acos(z_s);
                    float calpha = x_s / sin(theta);
                    float salpha = y_s / sin(theta);
                    float r = theta;
                    x = r*calpha;
                    y = r*salpha;
                }

                float x2 = x*x, y2 = y*y, _2xy = 2 * x*y;
                float r2 = x*x + y*y;
                float kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2) / (1 + ((k6*r2 + k5)*r2 + k4) * r2);
                float x_d = x*kr + p1*_2xy + p2*(r2 + 2 * x2);
                float y_d = y*kr + p1*(r2 + 2 * y2) + p2*_2xy;

                // Convert to pixel
                float u_d = fx*x_d + u0;
                float v_d = fy*y_d + v0;

                m1f[j] = float(u_d);
                m2f[j] = float(v_d);
            }
        }
        else if (_rectifyType == RT_LONGITUDE_LATITUDE)
        {
            // Only fish eye camera is allowed
            // [j,i] represent one [longitude, latitude] pair
            for (int j = 0; j < undis_size.width; j++)
            {
                float m = (float)undis_size.width;
                float n = (float)undis_size.height;

                float longitude = range_longitude.y * j / m + range_longitude.x,
                    latitude = range_latitude.y*i / n + range_latitude.x;

                // Geometric coordinate
                float x_s = -1*cos(longitude);
                float y_s = -1*sin(longitude)*cos(latitude);
                float z_s = sin(latitude)*sin(longitude);

                // Rotate by R.inv()
                cv::Mat_<float>R_inv = cv::Mat_<float>(_R.inv());
                float _x = R_inv(0, 0)*x_s + R_inv(0, 1)*y_s + R_inv(0, 2)*z_s;
                float _y = R_inv(1, 0)*x_s + R_inv(1, 1)*y_s + R_inv(1, 2)*z_s;
                float _z = R_inv(2, 0)*x_s + R_inv(2, 1)*y_s + R_inv(2, 2)*z_s;

                // Project back to sphere
                x_s = _x / sqrt(_x*_x + _y*_y + _z*_z);
                y_s = _y / sqrt(_x*_x + _y*_y + _z*_z);
                z_s = _z / sqrt(_x*_x + _y*_y + _z*_z);

                //// On a sphere, x = sin(theta)cos(alpha), y = sin(theta)cos(alpha), z = cos(theta)
                //float theta = atan(sqrt(x_s*x_s + y_s*y_s)/z_s);
                //// For equal distance model r = theta, r = x*x + y*y
                //float r = theta;
                //// x = r*cos(alpha), y = r*sin(alpha)
                //float x = r * x_s / sqrt(x_s*x_s + y_s*y_s);
                //float y = r * y_s / sqrt(x_s*x_s + y_s*y_s);
                float theta = acos(z_s);
                float calpha = x_s / sin(theta);
                float salpha = y_s / sin(theta);
                float r = theta;
                float x = r * calpha;
                float y = r * salpha;

                // Distortion
                float x2 = x*x;
                float y2 = y*y;
                float _2xy = 2 * x*y;
                float r2 = x*x + y*y;
                float kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2) / (1 + ((k6*r2 + k5)*r2 + k4)*r2);
                float x_d = x*kr + p1*_2xy + p2*(r2 + 2 * x2);
                float y_d = y*kr + p1*(r2 + 2 * y2) + p2*_2xy;

                // Convert to pixel
                float u_d = fx*x_d + u0;
                float v_d = fy*y_d + v0;
                m1f[j] = float(u_d);
                m2f[j] = float(v_d);
            }
        }
    }

}
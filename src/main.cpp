#include "camera.h"
#include "stereo.h"
#include <iostream>
using namespace std;
int main()
{
    shared_ptr<Stereo> stereo = shared_ptr<Stereo>(new Stereo());
    try
    {
        stereo->Init();
        stereo->Start();
        Camera left_camera, right_camera;
        if (!left_camera.OpenCamera(2, false, 320, 240)) {
            cout << "Can't open camera 1" << endl; return -1;
        }
        if (!right_camera.OpenCamera(1, false, 320, 240)) {
            cout << "Can't open camera 2" << endl; return -1;
        }
        cv::Mat left_image, right_image;
        while (true)
        {
            left_camera.ReadFrame(left_image);
            right_camera.ReadFrame(right_image);

            stereo->GrabImage(left_image, right_image);
            if (left_image.empty() || right_image.empty()) continue;
            cv::imshow("o_left_image", left_image);
            cv::imshow("o_right_image", right_image);
            char c = cv::waitKey(2);
            if (c == 27)
                break;
        }
    }
    catch (exception e)
    {
        cout << e.what() << endl;
        exit(0);
    }
    return 0;
}
#ifndef STEREO_STEREO_VISION_H
#define STEREO_STEREO_VISION_H

// Windows header or mfc atl 
#include <atlbase.h>
#include "qedit.h"
#include "dshow.h"
#include <windows.h>

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>


class CameraParam
{
public:
    cv::Matx33d _left_camera_matrix;
    cv::Matx33d _right_camera_matrix;
    cv::Matx33d _R;
    cv::Matx31d _T;
    cv::Matx41d _Distortion_left;
    cv::Matx41d _Distortion_right;
    cv::Matx33d _RectificationR_left;
    cv::Matx33d _RectificationR_right;
    cv::Matx34d _P1;
    cv::Matx34d _P2;
    cv::Matx44d _Q;
   
    CameraParam& operator=(CameraParam &cp) 
    {
        _left_camera_matrix = cp._left_camera_matrix;
        _right_camera_matrix = cp._right_camera_matrix;
        _R = cp._R;
        _T = cp._T;
        _Distortion_left = cp._Distortion_left;
        _Distortion_right = cp._Distortion_right;
        _RectificationR_left = cp._RectificationR_left;
        _RectificationR_right = cp._RectificationR_right;
        _P1 = cp._P1;
        _P2 = cp._P2;
        _Q = cp._Q;
    }
};

class Camera
{
private:

	bool m_bConnected, m_bLock, m_bChanged;

	int m_nWidth, m_nHeight;

	long m_nBufferSize;

	CComPtr<IGraphBuilder> m_pGraph;

	CComPtr<ISampleGrabber> m_pSampleGrabber;

	CComPtr<IMediaControl> m_pMediaControl;

	CComPtr<IMediaEvent> m_pMediaEvent;

	CComPtr<IBaseFilter> m_pSampleGrabberFilter;
	CComPtr<IBaseFilter> m_pDeviceFilter;
	CComPtr<IBaseFilter> m_pNullFilter;

	CComPtr<IPin> m_pGrabberInput;
	CComPtr<IPin> m_pGrabberOutput;
	CComPtr<IPin> m_pCameraOutput;
	CComPtr<IPin> m_pNullInputPin;

	bool BindFilter(int nCamIDX, IBaseFilter **pFilter);

    void FreeMediaType(AM_MEDIA_TYPE &pmt );

public:

	Camera();

	virtual ~Camera();

    /*
    Open camera
    /param nCamID : camera id number
    /param bDisplayProperties : the flag indicates whether show properties windows or not
    /param nWidth : the x resolution
    /param nHeight : the y resolution
    */
	bool OpenCamera(int nCamID, bool bDisplayProperties = true, int nWidth = 320, int nHeight = 240);

	void CloseCamera();

	static int CameraCount(); 

	static int CameraName(int nCamID, char* sName, int nBufferSize);

	int GetWidth(){return m_nWidth;} 

	int GetHeight(){return m_nHeight;}

	void  ReadFrame( cv::Mat & out);
};

#endif 

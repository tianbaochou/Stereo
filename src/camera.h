#ifndef STEREO_STEREO_VISION_H
#define STEREO_STEREO_VISION_H

// Windows header or mfc atl 
#include <atlbase.h>
#include "qedit.h"
#include "dshow.h"
#include <windows.h>
#include <opencv2\core\core.hpp>

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

#include "camera.h"
#include <iostream>
#pragma comment(lib,"Strmiids.lib") 

Camera::Camera()
{
    m_bConnected = m_bLock = m_bChanged = false;
    m_nWidth = m_nHeight = 0;
    m_nBufferSize = 0;

    m_pNullFilter = NULL;
    m_pMediaEvent = NULL;
    m_pSampleGrabberFilter = NULL;
    m_pGraph = NULL;

    CoInitialize(NULL);
}

Camera::~Camera()
{
    CloseCamera();
    CoUninitialize();
}

void Camera::CloseCamera()
{
    if (m_bConnected)
    {
        m_pMediaControl->Stop();
    }

    m_pGraph = NULL;
    m_pDeviceFilter = NULL;
    m_pMediaControl = NULL;
    m_pSampleGrabberFilter = NULL;
    m_pSampleGrabber = NULL;
    m_pGrabberInput = NULL;
    m_pGrabberOutput = NULL;
    m_pCameraOutput = NULL;
    m_pMediaEvent = NULL;
    m_pNullFilter = NULL;
    m_pNullInputPin = NULL;

    m_bConnected = m_bLock = m_bChanged = false;
    m_nWidth = m_nHeight = 0;
    m_nBufferSize = 0;
}

bool Camera::OpenCamera(int nCamID, bool bDisplayProperties, int nWidth, int nHeight)
{
    HRESULT hr = S_OK;

    CoInitialize(NULL);

    // Create the Filter Graph Manager.
    hr = CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC, IID_IGraphBuilder, (void **)&m_pGraph);

    hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER, IID_IBaseFilter, (LPVOID *)&m_pSampleGrabberFilter);

    hr = m_pGraph->QueryInterface(IID_IMediaControl, (void **)&m_pMediaControl);
    hr = m_pGraph->QueryInterface(IID_IMediaEvent, (void **)&m_pMediaEvent);

    hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER, IID_IBaseFilter, (LPVOID*)&m_pNullFilter);

    hr = m_pGraph->AddFilter(m_pNullFilter, L"NullRenderer");

    hr = m_pSampleGrabberFilter->QueryInterface(IID_ISampleGrabber, (void**)&m_pSampleGrabber);

    // Set the camera properties
    AM_MEDIA_TYPE   mt;
    ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
    mt.majortype = MEDIATYPE_Video;
    mt.subtype = MEDIASUBTYPE_RGB24;
    mt.formattype = FORMAT_VideoInfo;

    hr = m_pSampleGrabber->SetMediaType(&mt);
    FreeMediaType(mt);

    m_pGraph->AddFilter(m_pSampleGrabberFilter, L"Grabber");

    // Bind Device Filter.  We know the device because the id was passed in
    BindFilter(nCamID, &m_pDeviceFilter);
    m_pGraph->AddFilter(m_pDeviceFilter, NULL);

    CComPtr<IEnumPins> pEnum;
    m_pDeviceFilter->EnumPins(&pEnum);

    hr = pEnum->Reset();
    hr = pEnum->Next(1, &m_pCameraOutput, NULL);

    pEnum = NULL;
    m_pSampleGrabberFilter->EnumPins(&pEnum);
    pEnum->Reset();
    hr = pEnum->Next(1, &m_pGrabberInput, NULL);

    pEnum = NULL;
    m_pSampleGrabberFilter->EnumPins(&pEnum);
    pEnum->Reset();
    pEnum->Skip(1);
    hr = pEnum->Next(1, &m_pGrabberOutput, NULL);

    pEnum = NULL;
    m_pNullFilter->EnumPins(&pEnum);
    pEnum->Reset();
    hr = pEnum->Next(1, &m_pNullInputPin, NULL);

    if (bDisplayProperties) // If show the set windows ? ( Todo: the default by direct show )
    {
        CComPtr<ISpecifyPropertyPages> pPages;

        HRESULT hr = m_pCameraOutput->QueryInterface(IID_ISpecifyPropertyPages, (void**)&pPages);
        if (SUCCEEDED(hr))
        {
            PIN_INFO PinInfo;
            m_pCameraOutput->QueryPinInfo(&PinInfo);

            CAUUID caGUID;
            pPages->GetPages(&caGUID);

            OleCreatePropertyFrame(NULL, 0, 0,
                L"Property Sheet", 1,
                (IUnknown **)&(m_pCameraOutput.p),
                caGUID.cElems, caGUID.pElems,
                0, 0, NULL);

            CoTaskMemFree(caGUID.pElems);
            PinInfo.pFilter->Release();
        }
        pPages = NULL;
    }
    else
    {
        IAMStreamConfig *iconfig = NULL;
        hr = m_pCameraOutput->QueryInterface(IID_IAMStreamConfig, (void**)&iconfig);

        AM_MEDIA_TYPE *pmt;
        if (iconfig->GetFormat(&pmt) != S_OK)
        {
            return false;
        }

        if ((pmt->lSampleSize != (nWidth * nHeight * 3)) && (pmt->formattype == FORMAT_VideoInfo))
        {
            VIDEOINFOHEADER *phead = (VIDEOINFOHEADER*)(pmt->pbFormat);
            phead->bmiHeader.biWidth = nWidth;
            phead->bmiHeader.biHeight = nHeight;
            if ((hr = iconfig->SetFormat(pmt)) != S_OK)
            {
                return false;
            }
        }

        iconfig->Release();
        iconfig = NULL;
        FreeMediaType(*pmt);
    }

    hr = m_pGraph->Connect(m_pCameraOutput, m_pGrabberInput);
    hr = m_pGraph->Connect(m_pGrabberOutput, m_pNullInputPin);

    if (FAILED(hr))
    {
        std::cout << "Can't connect the pin\n";
        return false;
    }

    m_pSampleGrabber->SetBufferSamples(TRUE);
    m_pSampleGrabber->SetOneShot(TRUE);

    hr = m_pSampleGrabber->GetConnectedMediaType(&mt);
    if (FAILED(hr))
    {
        return false;
    }

    VIDEOINFOHEADER *videoHeader;
    videoHeader = reinterpret_cast<VIDEOINFOHEADER*>(mt.pbFormat);
    m_nWidth = videoHeader->bmiHeader.biWidth;
    m_nHeight = videoHeader->bmiHeader.biHeight;
    m_bConnected = true;

    pEnum = NULL;
    return true;
}


bool Camera::BindFilter(int nCamID, IBaseFilter **pFilter)
{
    if (nCamID < 0)
    {
        return false;
    }

    // enumerate all video capture devices
    CComPtr<ICreateDevEnum> pCreateDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (void**)&pCreateDevEnum);
    if (hr != NOERROR)
    {
        return false;
    }

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEm, 0);
    if (hr != NOERROR)
    {
        return false;
    }

    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
    int index = 0;
    while (hr = pEm->Next(1, &pM, &cFetched), hr == S_OK, index <= nCamID)
    {
        IPropertyBag *pBag;
        hr = pM->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pBag);
        if (SUCCEEDED(hr))
        {
            VARIANT var;
            var.vt = VT_BSTR;
            hr = pBag->Read(L"FriendlyName", &var, NULL);
            if (hr == NOERROR)
            {
                if (index == nCamID)
                {
                    pM->BindToObject(0, 0, IID_IBaseFilter, (void**)pFilter);
                }
                SysFreeString(var.bstrVal);
            }
            pBag->Release();
        }
        pM->Release();
        index++;
    }

    pCreateDevEnum = NULL;
    return true;
}

void Camera::ReadFrame(cv::Mat &out)
{
    long evCode, size = 0;
    if (m_pMediaControl->Run() != S_OK)
    {
        out = cv::Mat();
        return;
    }

    m_pMediaEvent->WaitForCompletion(INFINITE, &evCode);
    m_pSampleGrabber->GetCurrentBuffer(&size, NULL);

    if (size != m_nBufferSize)
    {
        m_nBufferSize = size;
        out = cv::Mat(m_nHeight, m_nWidth, CV_8UC3);
    }
    m_pSampleGrabber->GetCurrentBuffer(&m_nBufferSize, (long*)out.data);
    cv::flip(out, out, 0);
}

int Camera::CameraCount()
{
    int count = 0;
    CoInitialize(NULL);

    // Enumerate all video capture devices
    CComPtr<ICreateDevEnum> pCreateDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (void**)&pCreateDevEnum);

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEm, 0);
    if (hr != NOERROR)
    {
        return count;
    }

    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
    while (hr = pEm->Next(1, &pM, &cFetched), hr == S_OK)
    {
        count++;
    }

    pCreateDevEnum = NULL;
    pEm = NULL;
    return count;
}

void Camera::FreeMediaType(AM_MEDIA_TYPE &mt)
{
    if ((mt).cbFormat != 0)
    {
        CoTaskMemFree((PVOID)(mt).pbFormat);
        (mt).cbFormat = 0;
        (mt).pbFormat = NULL;
    }
    if ((mt).pUnk != NULL)
    {
        (mt).pUnk->Release();
        (mt).pUnk = NULL;
    }
}

int Camera::CameraName(int nCamID, char* sName, int nBufferSize)
{
    int count = 0;
    CoInitialize(NULL);

    // enumerate all video capture devices
    CComPtr<ICreateDevEnum> pCreateDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (void**)&pCreateDevEnum);

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEm, 0);
    if (hr != NOERROR) return 0;

    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
    while (hr = pEm->Next(1, &pM, &cFetched), hr == S_OK)
    {
        if (count == nCamID)
        {
            IPropertyBag *pBag = 0;
            hr = pM->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pBag);
            if (SUCCEEDED(hr))
            {
                VARIANT var;
                var.vt = VT_BSTR;
                hr = pBag->Read(L"FriendlyName", &var, NULL);
                if (hr == NOERROR)
                {

                    WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, sName, nBufferSize, "", NULL);

                    SysFreeString(var.bstrVal);
                }
                pBag->Release();
            }
            pM->Release();

            break;
        }
        count++;
    }

    pCreateDevEnum = NULL;
    pEm = NULL;

    return 1;
}

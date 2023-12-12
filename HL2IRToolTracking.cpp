#include "pch.h"
#include "HL2IRToolTracking.h"
#include "HL2IRTracking.g.cpp"

#include "IRToolTrack.h"

#include <winrt/Windows.Foundation.Collections.h>


extern "C"
HMODULE LoadLibraryA(
    LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;
static ResearchModeSensorConsent imuAccessCheck;
static HANDLE imuConsentGiven;

using namespace DirectX;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Graphics::Imaging;

//typedef std::chrono::duration<int64_t, std::ratio<1, 10'000'000>> HundredsOfNanoseconds;
//static constexpr UINT64 kMaxLongLong = static_cast<UINT64>(std::numeric_limits<long long>::max());

namespace winrt::HL2IRToolTracking::implementation
{
    HL2IRTracking::HL2IRTracking()
    {
        // Load Research Mode library
        camConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        imuConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
        HRESULT hr = S_OK;

        if (hrResearchMode)
        {
            typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
            PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
            if (pfnCreate)
            {
                winrt::check_hresult(pfnCreate(&m_pSensorDevice));
            }
            else
            {
                winrt::check_hresult(E_INVALIDARG);
            }
        }

        // get spatial locator of rigNode
        GUID guid;
        IResearchModeSensorDevicePerception* pSensorDevicePerception;
        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
        winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&guid));
        pSensorDevicePerception->Release();
        m_locator = SpatialGraphInteropPreview::CreateLocatorForNode(guid);

        size_t sensorCount = 0;

        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(HL2IRTracking::CamAccessOnComplete));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestIMUAccessAsync(HL2IRTracking::ImuAccessOnComplete));

        m_pSensorDevice->DisableEyeSelection();

        winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
        m_sensorDescriptors.resize(sensorCount);
        winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(), m_sensorDescriptors.size(), &sensorCount));
    }

    HRESULT HL2IRTracking::CheckCamConsent()
    {
        HRESULT hr = S_OK;
        DWORD waitResult = WaitForSingleObject(camConsentGiven, INFINITE);
        if (waitResult == WAIT_OBJECT_0)
        {
            switch (camAccessCheck)
            {
            case ResearchModeSensorConsent::Allowed:
                OutputDebugString(L"Access is granted\n");
                break;
            case ResearchModeSensorConsent::DeniedBySystem:
                OutputDebugString(L"Access is denied by the system\n");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::DeniedByUser:
                OutputDebugString(L"Access is denied by the user\n");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::NotDeclaredByApp:
                OutputDebugString(L"Capability is not declared in the app manifest\n");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::UserPromptRequired:
                OutputDebugString(L"Capability user prompt required\n");
                hr = E_ACCESSDENIED;
                break;
            default:
                OutputDebugString(L"Access is denied by the system\n");
                hr = E_ACCESSDENIED;
                break;
            }
        }
        else
        {
            hr = E_UNEXPECTED;
        }
        return hr;
    }

    HRESULT HL2IRTracking::CheckImuConsent()
    {
        HRESULT hr = S_OK;
        DWORD waitResult = WaitForSingleObject(imuConsentGiven, INFINITE);
        if (waitResult == WAIT_OBJECT_0)
        {
            switch (imuAccessCheck)
            {
            case ResearchModeSensorConsent::Allowed:
                OutputDebugString(L"Access is granted\n");
                break;
            case ResearchModeSensorConsent::DeniedBySystem:
                OutputDebugString(L"Access is denied by the system\n");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::DeniedByUser:
                OutputDebugString(L"Access is denied by the user\n");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::NotDeclaredByApp:
                OutputDebugString(L"Capability is not declared in the app manifest\n");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::UserPromptRequired:
                OutputDebugString(L"Capability user prompt required\n");
                hr = E_ACCESSDENIED;
                break;
            default:
                OutputDebugString(L"Access is denied by the system\n");
                hr = E_ACCESSDENIED;
                break;
            }
        }
        else
        {
            hr = E_UNEXPECTED;
        }
        return hr;
    }
    void HL2IRTracking::InitializeDepthSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == DEPTH_AHAT)
            {
                m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_depthSensor);
                m_depthSensor->QueryInterface(IID_PPV_ARGS(&m_pDepthCameraSensor));
                m_pDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_depthCameraPose);
                m_depthCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_depthCameraPose));
                OutputDebugString(L"Short Depth Sensor initialized.\n");
                break;
            }
        }
    }

    void HL2IRTracking::StartDepthSensorLoop()
    {
        if (!m_pDepthCameraSensor) InitializeDepthSensor();

        // Check consent before starting thread
        if (SUCCEEDED(CheckCamConsent())) m_pDepthUpdateThread = new std::thread(HL2IRTracking::DepthSensorLoop, this);
    }

    void HL2IRTracking::DepthSensorLoop(HL2IRTracking* pHL2IRTracking)
    {
        // prevent starting loop for multiple times
        if (!pHL2IRTracking->m_depthSensorLoopStarted)
            pHL2IRTracking->m_depthSensorLoopStarted = true;
        else return;

        OutputDebugString(L"Opening Depth Stream...\n");
        winrt::check_hresult(pHL2IRTracking->m_depthSensor->OpenStream());

        try
        {
            UINT64 lastTs = 0;
            IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
            ResearchModeSensorResolution resolution;
            ResearchModeSensorTimestamp timestamp;
            while (pHL2IRTracking->m_depthSensorLoopStarted)
            {
                pHL2IRTracking->m_depthSensor->GetNextBuffer(&pDepthSensorFrame);
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                if (timestamp.HostTicks == lastTs)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));    // sleep for 0.01 second
                    OutputDebugString(L"No New Depth Frame\n");
                    continue;
                }
                lastTs = timestamp.HostTicks;

                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2IRTracking->m_depthResolution = resolution;

                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);

                size_t outAbBufferCount = 0;
                const UINT16* pAbImage = nullptr;
                pDepthFrame->GetAbDepthBuffer(&pAbImage, &outAbBufferCount);

                auto pDepthTexture = std::make_unique<uint8_t[]>(outBufferCount);
                std::vector<float> pointCloud;


                // get tracking transform
                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                auto transToWorld = pHL2IRTracking->m_locator.TryLocateAtTimestamp(ts, pHL2IRTracking->m_refFrame);
                if (transToWorld == nullptr)
                {
                    OutputDebugString(L"Trans To World is null\n");
                    continue;
                }
                pHL2IRTracking->m_latestShortDepthTimestamp = ts.TargetTime().time_since_epoch().count();

                XMMATRIX depthToWorld = pHL2IRTracking->m_depthCameraPoseInvMatrix * SpatialLocationToDxMatrix(transToWorld);

                pHL2IRTracking->mu.lock();
                auto roiCenterFloat = XMFLOAT3(pHL2IRTracking->m_roiCenter[0], pHL2IRTracking->m_roiCenter[1], pHL2IRTracking->m_roiCenter[2]);
                auto roiBoundFloat = XMFLOAT3(pHL2IRTracking->m_roiBound[0], pHL2IRTracking->m_roiBound[1], pHL2IRTracking->m_roiBound[2]);
                pHL2IRTracking->mu.unlock();
                XMVECTOR roiCenter = XMLoadFloat3(&roiCenterFloat);
                XMVECTOR roiBound = XMLoadFloat3(&roiBoundFloat);

                winrt::Windows::Foundation::Numerics::float4x4 depthToWorld_float4x4;
                XMStoreFloat4x4(&depthToWorld_float4x4, depthToWorld);

                std::vector<float> floatContainer(12);
                auto pFloatContainer = floatContainer.data();
                if (transToWorld)
                {
                    // Rotation
                    *pFloatContainer++ = depthToWorld_float4x4.m11;
                    *pFloatContainer++ = depthToWorld_float4x4.m12;
                    *pFloatContainer++ = depthToWorld_float4x4.m13;

                    *pFloatContainer++ = depthToWorld_float4x4.m21;
                    *pFloatContainer++ = depthToWorld_float4x4.m22;
                    *pFloatContainer++ = depthToWorld_float4x4.m23;

                    *pFloatContainer++ = depthToWorld_float4x4.m31;
                    *pFloatContainer++ = depthToWorld_float4x4.m32;
                    *pFloatContainer++ = depthToWorld_float4x4.m33;

                    // Translation
                    *pFloatContainer++ = depthToWorld_float4x4.m41;
                    *pFloatContainer++ = depthToWorld_float4x4.m42;
                    *pFloatContainer++ = depthToWorld_float4x4.m43;
                }

                if (!pHL2IRTracking->m_LUTGenerated_short)
                {
                    float uv[2];
                    float xy[2];
                    std::vector<float> lutTable(size_t(resolution.Width * resolution.Height) * 3);
                    auto pLutTable = lutTable.data();

                    for (size_t y = 0; y < resolution.Height; y++)
                    {
                        uv[1] = (y + 0.5f);
                        for (size_t x = 0; x < resolution.Width; x++)
                        {
                            uv[0] = (x + 0.5f);
                            HRESULT hr = pHL2IRTracking->m_pDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                            if (FAILED(hr))
                            {
                                *pLutTable++ = xy[0];
                                *pLutTable++ = xy[1];
                                *pLutTable++ = 0.f;
                                continue;
                            }
                            float z = 1.0f;
                            const float norm = sqrtf(xy[0] * xy[0] + xy[1] * xy[1] + z * z);
                            const float invNorm = 1.0f / norm;
                            xy[0] *= invNorm;
                            xy[1] *= invNorm;
                            z *= invNorm;

                            // Dump LUT row
                            *pLutTable++ = xy[0];
                            *pLutTable++ = xy[1];
                            *pLutTable++ = z;
                        }
                    }
                    OutputDebugString(L"Create Space for lut...\n");
                    pHL2IRTracking->m_lut_short = new float[outBufferCount * 3];
                    memcpy(pHL2IRTracking->m_lut_short, lutTable.data(), lutTable.size() * sizeof(float));
                    pHL2IRTracking->m_lutLength_short = lutTable.size();
                    pHL2IRTracking->m_LUTGenerated_short = true;
                }


                // ------------------------------- Tool tracking start -------------------------------
                std::vector<float> irToolCenters;
                if (pHL2IRTracking->m_IRToolTracker != nullptr && pHL2IRTracking->m_IRToolTracker->IsTracking() && pHL2IRTracking->m_latestShortDepthTimestamp > pHL2IRTracking->m_latestTrackedFrame)
                {

                    cv::Mat transform_matrix = cv::Mat(4, 4, CV_32F, &depthToWorld_float4x4).t();
                    pHL2IRTracking->m_IRToolTracker->AddFrame((void*)pAbImage, (void*)pDepth, resolution.Width, resolution.Height, transform_matrix, pHL2IRTracking->m_latestShortDepthTimestamp);
                    pHL2IRTracking->m_latestTrackedFrame = pHL2IRTracking->m_latestShortDepthTimestamp;

                }
                // ------------------------------- Tool tracking end -------------------------------

                pDepthTexture.reset();

                // release space
                if (pDepthFrame) {
                    pDepthFrame->Release();
                }
                if (pDepthSensorFrame)
                {
                    pDepthSensorFrame->Release();
                }
            }
        }
        catch (const std::exception& exc)
        {
            std::stringstream ss;
            ss << "Depth Loop Exception: " << exc.what();
            std::string msg = ss.str();
            std::wstring widemsg = std::wstring(msg.begin(), msg.end());
            OutputDebugString(widemsg.c_str());
        }
        pHL2IRTracking->m_depthSensor->CloseStream();
        pHL2IRTracking->m_depthSensor->Release();
        pHL2IRTracking->m_depthSensor = nullptr;
        pHL2IRTracking->m_pDepthCameraSensor = nullptr;

    }

    void HL2IRTracking::CamAccessOnComplete(ResearchModeSensorConsent consent)
    {
        camAccessCheck = consent;
        SetEvent(camConsentGiven);
    }

    void HL2IRTracking::ImuAccessOnComplete(ResearchModeSensorConsent consent)
    {
        imuAccessCheck = consent;
        SetEvent(imuConsentGiven);
    }


    inline INT64 HL2IRTracking::GetShortDepthTimestamp() { return m_latestShortDepthTimestamp; }


    inline int HL2IRTracking::GetDepthBufferSize() { return m_depthBufferSize; }

    inline bool HL2IRTracking::ShortThrowLUTUpdated() { return m_LUTGenerated_short; }


    hstring HL2IRTracking::PrintDepthResolution()
    {
        std::string res_c_ctr = std::to_string(m_depthResolution.Height) + "x" + std::to_string(m_depthResolution.Width) + "x" + std::to_string(m_depthResolution.BytesPerPixel);
        return winrt::to_hstring(res_c_ctr);
    }

    hstring HL2IRTracking::PrintDepthExtrinsics()
    {
        std::stringstream ss;
        ss << MatrixToString(m_depthCameraPose);
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        //OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

    std::string HL2IRTracking::MatrixToString(DirectX::XMFLOAT4X4 mat)
    {
        std::stringstream ss;
        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < 4; j++)
            {
                ss << mat(i, j) << ",";
            }
            ss << "\n";
        }
        return ss.str();
    }



    // Set the reference coordinate system. Need to be set before the sensor loop starts; otherwise, default coordinate will be used.
    void HL2IRTracking::SetReferenceCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem refCoord)
    {
        std::unique_lock<std::shared_mutex> l(mu);
        m_refFrame = refCoord;
    }



    XMMATRIX HL2IRTracking::SpatialLocationToDxMatrix(SpatialLocation location) {
        auto rot = location.Orientation();
        auto quatInDx = XMFLOAT4(rot.x, rot.y, rot.z, rot.w);
        auto rotMat = XMMatrixRotationQuaternion(XMLoadFloat4(&quatInDx));
        auto pos = location.Position();
        auto posMat = XMMatrixTranslation(pos.x, pos.y, pos.z);
        return rotMat * posMat;
    }

    //On device IR Tool tracking
    bool HL2IRTracking::IsTrackingTools()
    {
        if (m_IRToolTracker == nullptr)
        {
            return false;
        }
        return m_IRToolTracker->IsTracking();
    }

    bool HL2IRTracking::AddToolDefinition(int sphere_count, array_view<const float> sphere_positions, float sphere_radius, hstring identifier)
    {
        return AddToolDefinition(sphere_count, sphere_positions, sphere_radius, identifier, sphere_count, 0.3f, 0.6f);
    }

    bool HL2IRTracking::AddToolDefinition(int sphere_count, array_view<const float> sphere_positions, float sphere_radius, hstring identifier, int min_visible_spheres)
    {
        return AddToolDefinition(sphere_count, sphere_positions, sphere_radius, identifier, min_visible_spheres, 0.3f, 0.6f);
    }

    bool HL2IRTracking::AddToolDefinition(int sphere_count, array_view<const float> sphere_positions, float sphere_radius, hstring identifier, int min_visible_spheres, float lowpass_rotation, float lowpass_position)
    {
        if (m_IRToolTracker == nullptr)
        {
            OutputDebugString(L"On Device Tracking First Initialization\n");
            m_IRToolTracker = new IRToolTracker(this);
        }
        //Minimum required spheres for a tool is 3
        if (sphere_count < 3) {
            return false;
        }
        if (sphere_positions.size() != 3 * sphere_count)
            return false;

        cv::Mat3f spheres = cv::Mat3f(sphere_count, 1);
        int j = 0;
        for (int i = 0; i < sphere_count; i++) {
            //Flip z and convert from unity meters to millimeters
            spheres.at<cv::Vec3f>(i, 0) = cv::Vec3f(sphere_positions[j] * 1000, sphere_positions[j + 1] * 1000, sphere_positions[j + 2] * -1000);
            j += 3;
        }
        OutputDebugString(L"On Device Tracking Constructed Tool.\n");
        return m_IRToolTracker->AddTool(spheres, sphere_radius, to_string(identifier), min_visible_spheres, std::clamp(lowpass_rotation, 0.f, 1.f), std::clamp(lowpass_position, 0.f, 1.f));
    }

    bool HL2IRTracking::RemoveToolDefinition(hstring identifier)
    {
        if (m_IRToolTracker == nullptr)
            return false;
        return m_IRToolTracker->RemoveTool(to_string(identifier));
    }

    bool HL2IRTracking::RemoveAllToolDefinitions()
    {
        if (m_IRToolTracker == nullptr)
            return false;
        return m_IRToolTracker->RemoveAllTools();
    }

    bool HL2IRTracking::StartToolTracking()
    {
        if (m_IRToolTracker == nullptr)
        {
            OutputDebugString(L"On Device Tracking First Initialization\n");
            m_IRToolTracker = new IRToolTracker(this);
        }
        //Start the depth camera
        StartDepthSensorLoop();

        OutputDebugString(L"On Device Tracking Starting.\n");

        return m_IRToolTracker->StartTracking();;
    }

    bool HL2IRTracking::StopToolTracking()
    {
        if (m_IRToolTracker == nullptr)
            return false;
        OutputDebugString(L"On Device Tracking Stopping.\n");
        //Stop Tracking thread
        m_IRToolTracker->StopTracking();

        //Stop depth sensor thread
        m_depthSensorLoopStarted = false;
        m_pDepthUpdateThread->join();
        OutputDebugString(L"On Device Tracking Stopped.\n");
        return true;

    }

    com_array<float> HL2IRTracking::GetToolTransform(hstring identifier)
    {
        if (m_IRToolTracker == nullptr)
            return com_array<float>(8, 0);

        cv::Mat transform = m_IRToolTracker->GetToolTransform(to_string(identifier));
        std::vector<float> array;
        if (transform.isContinuous()) {
            array.assign((float*)transform.data, (float*)transform.data + transform.total() * transform.channels());
        }
        else {
            for (int i = 0; i < transform.rows; ++i) {
                array.insert(array.end(), transform.ptr<float>(i), transform.ptr<float>(i) + transform.cols * transform.channels());
            }
        }
        com_array<float> tempBuffer = com_array<float>(array.begin(), array.end());
        return tempBuffer;
    }

    INT64 HL2IRTracking::GetTrackingTimestamp()
    {
        return m_latestTrackedFrame;
    }

    bool HL2IRTracking::DepthMapImagePointToCameraUnitPlane(float(&uv)[2], float(&xy)[2])
    {
        if (m_pDepthCameraSensor == nullptr)
        {
            return false;
        }
        return SUCCEEDED(m_pDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy));
    }
}

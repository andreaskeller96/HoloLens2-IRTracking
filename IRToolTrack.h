#pragma once

#include <vector>
#include <map>
#include <thread>
#include <cstdint>
#include <wrl.h>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <DirectXMath.h>

#include "IRStructs.h"

//Forward Decl
namespace winrt::HL2IRToolTracking::implementation
{
	struct HL2IRTracking;
}


class IRToolTracker
{
public:
	IRToolTracker(winrt::HL2IRToolTracking::implementation::HL2IRTracking* pResearchMode) {
		m_pResearchMode = pResearchMode;
	}


	void AddFrame(void* pAbImage, void* pDepth, UINT32 depthWidth, UINT32 depthHeight, cv::Mat _pose, INT64 _timestamp);
	void AddEnvFrame(void* pLFImage, void* pRFImage, size_t LFOutBufferCount, INT64 tsLF, INT64 tsRF, float* pLFExtr, float* pRFExtr);
	bool AddTool(cv::Mat3f spheres, float sphere_radius, std::string identifier, uint min_visible_spheres, float lowpass_rotation, float lowpass_position);
	bool RemoveTool(std::string identifier);
	bool RemoveAllTools();
	bool StartTracking();


	void StopTracking();
	inline bool IsTracking() { return m_bIsCurrentlyTracking; }

	cv::Mat GetToolTransform(std::string identifier);
	void TrackTools();


private:

	bool ProcessFrame(AHATFrame* rawFrame, ProcessedAHATFrame& result);
	
	void TrackTool(IRTrackedTool &tool, ProcessedAHATFrame &frame, ToolResultContainer &result);

	void UnionSegmentation(ToolResultContainer* raw_solutions, int num_tools, ProcessedAHATFrame frame);

	cv::Mat MatchPointsKabsch(IRTrackedTool tool, ProcessedAHATFrame frame, std::vector<int> sphere_ids, std::vector<int> occluded_nodes);

	cv::Mat FlipTransformRightLeft(cv::Mat hololens_transform);

	void ConstructMap(cv::Mat3f spheres_xyz, int num_spheres, cv::Mat& result_map, std::vector<Side>& result_ordered_sides);


	bool m_bShouldStop = false;

	std::vector<IRTrackedTool> m_Tools;

	AHATFrame* m_CurrentFrame = nullptr;
	std::vector<EnvFrame*> m_CurEnvFrameBuffer;
	int m_iCurEnvFrameBufferMaxSize = 3;

	std::mutex m_MutexCurFrame;
	std::mutex m_MutexCurEnvFrame;

	std::map<std::string, int> m_ToolIndexMapping;

	float m_fToleranceSide = 4.0f;
	float m_fToleranceAvg = 4.0f;

	bool m_bIsCurrentlyTracking = false;

	std::thread m_TrackingThread{};

	long long m_lTrackedTimestamp = 0;

	winrt::HL2IRToolTracking::implementation::HL2IRTracking* m_pResearchMode;

};

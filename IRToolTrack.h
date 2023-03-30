#pragma once

#include <vector>
#include <map>
#include <thread>
#include <cstdint>
#include <wrl.h>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>


//Forward Decl
namespace winrt::HL2IRToolTracking::implementation
{
	struct HL2IRTracking;
}

struct Side
{
	int id_from{ 0 };
	int id_to{ 0 };
	float distance{ 0 };

	static bool compare(Side a, Side b) {
		if (a.distance < b.distance) {
			return true;
		}
		return false;
	}

};

struct AHATFrame {
	long long timestamp;
	cv::Mat hololens_pose;
	cv::Mat cvAbImage;
	UINT16* pDepth;
	UINT32 depthWidth;
	UINT32 depthHeight;
};

struct ProcessedAHATFrame
{
	long long timestamp;
	cv::Mat hololens_pose;
	uint num_spheres;
	cv::Mat3f spheres_xyd;
	std::map<float, cv::Mat3f> spheres_xyz_per_mm;
	std::map<float, std::vector<Side>> ordered_sides_per_mm;
	std::map<float, cv::Mat> map_per_mm;
};

struct ToolResult
{
	int tool_id{ -1 };
	std::vector<int> sphere_ids;
	float error{ 0 };
	static bool compare(ToolResult a, ToolResult b)
	{
		if (a.error < b.error) {
			return true;
		}
		return false;
	}
};

struct ToolResultContainer
{
	int tool_id{ -1 };
	std::vector<ToolResult> candidates;
};


class IRToolKalmanFilter
{
public:
	IRToolKalmanFilter() {
		m_filter = cv::KalmanFilter(6, 3, 0, CV_32F);

	}
	IRToolKalmanFilter(float measurementNoise, float positionNoise, float velocityNoise) {
		m_filter = cv::KalmanFilter(6, 3, 0, CV_32F);
		m_fPositionNoise = positionNoise;
		m_fVelocityNoise = velocityNoise;
		m_fMeasurementNoise = measurementNoise;
	}
	cv::Vec3f FilterData(cv::Vec3f value) {
		if (!m_bInitialized) {
			InitializeFilter(value);
		}
		auto prediction = m_filter.predict();
		measurement = cv::Mat(value).reshape(1, 3);
		cv::Mat correction = m_filter.correct(measurement);

		return cv::Vec3f(correction.at<float>(0, 0), correction.at<float>(1, 0), correction.at<float>(2, 0));
	}
private:
	void InitializeFilter(cv::Vec3f value) {
		// Initialize the state transition matrix (A)
		cv::Mat A = cv::Mat::eye(6, 6, CV_32F);
		A.at<float>(0, 3) = 1.f; // x += vx*dt
		A.at<float>(1, 4) = 1.f; // y += vy*dt
		A.at<float>(2, 5) = 1.f; // z += vz*dt
		m_filter.transitionMatrix = A;

		// Initialize the measurement matrix (H)
		cv::Mat H = cv::Mat::zeros(3, 6, CV_32F);
		H.at<float>(0, 0) = 1.f;// x
		H.at<float>(1, 1) = 1.f;// y
		H.at<float>(2, 2) = 1.f;// z
		m_filter.measurementMatrix = H;

		// Initialize the process noise covariance matrix (Q)
		cv::Mat Q = cv::Mat::zeros(6, 6, CV_32F);
		Q.at<float>(0, 0) = Q.at<float>(1, 1) = Q.at<float>(2, 2) = m_fPositionNoise; // position noise
		Q.at<float>(3, 3) = Q.at<float>(4, 4) = Q.at<float>(5, 5) = m_fVelocityNoise; // velocity noise
		m_filter.processNoiseCov = Q;

		// Initialize the measurement noise covariance matrix (R)
		cv::Mat R = cv::Mat::eye(3, 3, CV_32F) * m_fMeasurementNoise; // measurement noise
		m_filter.measurementNoiseCov = R;

		// Initialize the state estimate (x) and the error covariance matrix (P)
		cv::Mat x = cv::Mat::zeros(6, 1, CV_32F); // initial state is all zeros
		cv::Mat P = cv::Mat::eye(6, 6, CV_32F); // initial error covariance is identity matrix
		m_filter.statePre = x;
		m_filter.errorCovPost = P;

		m_bInitialized = true;
		return;
	}

	cv::Mat measurement = cv::Mat(1, 3, CV_32F);
	//cv::Mat_<float> measurement(3, 1);
	bool m_bInitialized = false;
	cv::KalmanFilter m_filter;
	float m_fMeasurementNoise = 1;// 1e-1; - measurements are usually within 3mm of position (3^2=9)
	float m_fPositionNoise = 1e-4;//1e-4; - prediction is usually within .1mm of position (.1^2 = 1e-2)
	float m_fVelocityNoise = 1e-2;// 1e-2; - velocity is probably similar

};

struct IRToolDefinition
{
	std::string identifier;
	uint num_spheres;
	cv::Mat3f spheres_xyz;
	std::vector<IRToolKalmanFilter> sphere_kalman_filters;
	IRToolKalmanFilter kalman_filter_position{ 1e-3, 1e-7, 1e-5 };
	std::vector<Side> ordered_sides;

	cv::Mat map;
	float sphere_radius;
};


class IRToolTracker
{
public:
	IRToolTracker(winrt::HL2IRToolTracking::implementation::HL2IRTracking* pResearchMode) {
		m_pResearchMode = pResearchMode;
	}


	void AddFrame(void* pAbImage, void* pDepth, UINT32 depthWidth, UINT32 depthHeight, cv::Mat _pose, long long _timestamp);
	bool AddTool(cv::Mat3f spheres, float sphere_radius, std::string identifier);
	bool RemoveTool(std::string identifier);
	bool RemoveAllToolDefinitions();
	bool StartTracking();


	void StopTracking();
	inline bool IsTracking() { return m_bIsCurrentlyTracking; }

	cv::Mat GetToolTransform(std::string identifier);
	void TrackTools();


private:

	bool ProcessFrame(AHATFrame* rawFrame, ProcessedAHATFrame& result);

	void TrackTool(IRToolDefinition tool, ProcessedAHATFrame frame, ToolResultContainer& result);

	cv::Mat* UnionSegmentation(ToolResultContainer* raw_solutions, int num_tools, ProcessedAHATFrame frame);

	cv::Mat MatchPointsKabsch(IRToolDefinition tool, ProcessedAHATFrame frame, std::vector<int> sphere_ids);

	cv::Mat FlipTransformRightLeft(cv::Mat hololens_transform);

	void ConstructMap(cv::Mat3f spheres_xyz, int num_spheres, cv::Mat& result_map, std::vector<Side>& result_ordered_sides);


	bool m_bShouldStop = false;

	std::vector<IRToolDefinition> m_Tools;
	std::vector<ProcessedAHATFrame> m_FrameQueue;

	AHATFrame* m_CurrentFrame = nullptr;

	std::mutex m_MutexCurFrame;

	std::map<std::string, int> m_ToolIndexMapping;
	std::atomic<cv::Mat*> m_CurToolTransforms = nullptr;



	int m_nDownLimitAb = 128;
	int m_nUpLimitAb = 512;
	int m_nQueueSize = 5;

	float m_fToleranceSide = 4.0f;
	float m_fToleranceAvg = 4.0f;

	bool m_bIsCurrentlyTracking = false;

	std::thread m_TrackingThread{};

	winrt::HL2IRToolTracking::implementation::HL2IRTracking* m_pResearchMode;

	//Crude way of doing uniqueness checks, I dont like it but oh well
	int m_prime_numbers[500] = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37,
		41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107,
		109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179,
		181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251,
		257, 263, 269, 271, 277, 281, 283, 293, 307, 311, 313, 317, 331,
		337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409,
		419, 421, 431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487,
		491, 499, 503, 509, 521, 523, 541, 547, 557, 563, 569, 571, 577,
		587, 593, 599, 601, 607, 613, 617, 619, 631, 641, 643, 647, 653,
		659, 661, 673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743,
		751, 757, 761, 769, 773, 787, 797, 809, 811, 821, 823, 827, 829,
		839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929,
		937, 941, 947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013, 1019,
		1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069, 1087, 1091,
		1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163, 1171,
		1181, 1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231, 1237, 1249,
		1259, 1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319,
		1321, 1327, 1361, 1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429,
		1433, 1439, 1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489,
		1493, 1499, 1511, 1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571,
		1579, 1583, 1597, 1601, 1607, 1609, 1613, 1619, 1621, 1627, 1637,
		1657, 1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733,
		1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 1823,
		1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889, 1901, 1907,
		1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997, 1999,
		2003, 2011, 2017, 2027, 2029, 2039, 2053, 2063, 2069, 2081, 2083,
		2087, 2089, 2099, 2111, 2113, 2129, 2131, 2137, 2141, 2143, 2153,
		2161, 2179, 2203, 2207, 2213, 2221, 2237, 2239, 2243, 2251, 2267,
		2269, 2273, 2281, 2287, 2293, 2297, 2309, 2311, 2333, 2339, 2341,
		2347, 2351, 2357, 2371, 2377, 2381, 2383, 2389, 2393, 2399, 2411,
		2417, 2423, 2437, 2441, 2447, 2459, 2467, 2473, 2477, 2503, 2521,
		2531, 2539, 2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617,
		2621, 2633, 2647, 2657, 2659, 2663, 2671, 2677, 2683, 2687, 2689,
		2693, 2699, 2707, 2711, 2713, 2719, 2729, 2731, 2741, 2749, 2753,
		2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819, 2833, 2837, 2843,
		2851, 2857, 2861, 2879, 2887, 2897, 2903, 2909, 2917, 2927, 2939,
		2953, 2957, 2963, 2969, 2971, 2999, 3001, 3011, 3019, 3023, 3037,
		3041, 3049, 3061, 3067, 3079, 3083, 3089, 3109, 3119, 3121, 3137,
		3163, 3167, 3169, 3181, 3187, 3191, 3203, 3209, 3217, 3221, 3229,
		3251, 3253, 3257, 3259, 3271, 3299, 3301, 3307, 3313, 3319, 3323,
		3329, 3331, 3343, 3347, 3359, 3361, 3371, 3373, 3389, 3391, 3407,
		3413, 3433, 3449, 3457, 3461, 3463, 3467, 3469, 3491, 3499, 3511,
		3517, 3527, 3529, 3533, 3539, 3541, 3547, 3557, 3559, 3571 };
};

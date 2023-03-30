#include "IRToolTrack.h"
#include "HL2IRToolTracking.h"

#define DEBUG_OUTPUT FALSE
#define DEBUG_TIME FALSE
#define DEBUG_NO_FILTER FALSE

void IRToolTracker::StopTracking()
{
#if DEBUG_OUTPUT
	OutputDebugString(L"StopTracking\n");
#endif
	m_bShouldStop = true;
	//Wait until thread shuts down
	m_TrackingThread.join();
}

cv::Mat IRToolTracker::GetToolTransform(std::string identifier)
{
#if DEBUG_OUTPUT
	std::string funcoutput = "GetToolTransform for " + identifier + "\n";
	OutputDebugString(std::wstring(funcoutput.begin(), funcoutput.end()).c_str());
#endif
	if (m_CurToolTransforms.load() == nullptr || m_ToolIndexMapping.count(identifier) == 0)
		return cv::Mat::zeros(8, 1, CV_32F);

	auto it = m_ToolIndexMapping.find(identifier);
	int index = it->second;
	auto transform = m_CurToolTransforms.load()[index];

	return transform;
}

void IRToolTracker::TrackTools()
{
#if DEBUG_OUTPUT
	OutputDebugString(L"TrackTools\n");
#endif
	while (!m_bShouldStop) {
#if DEBUG_TIME
		auto start = std::chrono::high_resolution_clock::now();
#endif
		m_bIsCurrentlyTracking = true;
		
		if (m_CurrentFrame == nullptr) {
			Sleep(10);
			continue;
		}
		m_MutexCurFrame.lock();
		//Copy pointer to frame
		AHATFrame* rawFrame = m_CurrentFrame;
		m_CurrentFrame = nullptr;
		m_MutexCurFrame.unlock();
		int current_num_tools = m_Tools.size();
		ToolResultContainer* raw_results = new ToolResultContainer[current_num_tools];
		for (int i = 0; i < current_num_tools; i++) {
			ToolResultContainer result{ i, std::vector<ToolResult>() };
			raw_results[i] = result;
		}

		ProcessedAHATFrame processedFrame;

		if (!ProcessFrame(rawFrame, processedFrame)) {
			continue;
		}
		

		//std::vector<std::thread> tool_track_threads(current_num_tools);

		for (int i = 0; i < current_num_tools; i++) {
			IRToolDefinition tool = m_Tools.at(i);
			TrackTool(tool, processedFrame, raw_results[i]);
			//tool_track_threads.at(i) = std::thread(&NDIToolTracker::TrackTool, this, tool, FrameCurWorkingOn, raw_results[i]);
		}

		//for (auto & thread : tool_track_threads) {
		//	thread.join();
		//}
		auto result = UnionSegmentation(raw_results, current_num_tools, processedFrame);
		auto old_result = m_CurToolTransforms.load();
		m_CurToolTransforms.store(result);
		delete[] old_result;
		delete[] raw_results;

#if DEBUG_TIME
		auto finish = std::chrono::high_resolution_clock::now();
		std::string my_str = "Tool Tracking loop ran for ";
		my_str += std::to_string((std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() / 1000000.f)) += " ms.\n";
		std::wstring stemp = std::wstring(my_str.begin(), my_str.end());
		LPCWSTR my2 = stemp.c_str();
		OutputDebugString(my2);
#endif
	}
	m_bIsCurrentlyTracking = false;
}

void IRToolTracker::TrackTool(IRToolDefinition tool, ProcessedAHATFrame frame, ToolResultContainer& result)
{
#if DEBUG_OUTPUT
	OutputDebugString(L"TrackTool\n");
#endif
	if (frame.num_spheres < tool.num_spheres) {
		//Not enough spheres for the tool are available
		return;
	}
	std::vector<Side> eligible_sides;
	std::vector<int> eligible_sides_indices;
	float tool_first_size_length = tool.map.at<float>(0, 1);

	auto it_sides = frame.ordered_sides_per_mm.find(tool.sphere_radius);
	auto frame_ordered_sides = it_sides->second;

	auto it_map = frame.map_per_mm.find(tool.sphere_radius);
	auto frame_map = it_map->second;
	//Find the set of eligible side to start with - aka sides that have similar length to first side of tool
	for (int i = 0; i < frame_ordered_sides.size(); i++) {
		Side s = frame_ordered_sides.at(i);

		if (cv::abs(s.distance - tool_first_size_length) < m_fToleranceSide) {
			eligible_sides.push_back(s);
			eligible_sides_indices.push_back(i);
		}
	}
	if (eligible_sides.size() == 0)
		return;

	std::vector<std::tuple<std::vector<int>, float, int>> search_list;

	//From the start sides, add each direction to search queue
	for (Side s : eligible_sides)
	{

		std::tuple<std::vector<int>, float, int> forward(std::vector<int>{s.id_from, s.id_to}, s.distance - tool_first_size_length, 1);
		std::tuple<std::vector<int>, float, int> backward(std::vector<int>{s.id_to, s.id_from}, s.distance - tool_first_size_length, 1);
		search_list.push_back(forward);
		search_list.push_back(backward);
	}

	while (search_list.size() > 0) {
		auto curr = search_list.at(0);
		search_list.erase(search_list.begin());
		std::vector<int> searched_ids = std::get<0>(curr);
		float error_searched = std::get<1>(curr);
		int num_searched = searched_ids.size();

		if (searched_ids.size() == tool.num_spheres) {
			ToolResult r{};
			r.error = error_searched;
			r.sphere_ids = searched_ids;
			if (error_searched / std::get<2>(curr))
				result.candidates.push_back(r);
			continue;
		}

		for (int i = 0; i < frame.num_spheres; i++) {
			if (std::find(searched_ids.begin(), searched_ids.end(), i) != searched_ids.end()) {
				//Already used this element
				continue;
			}
			std::vector<float> temp_err;
			bool exceeded_side_tolerance = false;
			float error_new = 0.f;
			float error_counter = 0.f;
			int candidate_sphere_id = i;
			for (int j = 0; j < searched_ids.size(); j++) {
				int id1 = searched_ids.at(j);
				int id2 = i;
				float error_side = cv::abs(frame_map.at<float>(id1, id2) - tool.map.at<float>(j, num_searched));
				if (error_side > m_fToleranceSide) {
					exceeded_side_tolerance = true;
					break;
				}
				error_new += error_side;
				error_counter++;
				temp_err.push_back(error_side);
			}
			if (exceeded_side_tolerance)
				continue;
			std::vector<int> searched_ids_new = std::vector<int>(searched_ids);
			searched_ids_new.push_back(i);
			search_list.push_back(std::tuple<std::vector<int>, float, int>(searched_ids_new, error_searched + error_new, std::get<2>(curr) + error_counter));
		}
	}
}



cv::Mat* IRToolTracker::UnionSegmentation(ToolResultContainer* raw_solutions, int num_tools, ProcessedAHATFrame frame) {
#if DEBUG_OUTPUT
	OutputDebugString(L"UnionSegmentation\n");
#endif
	int* tool_solutions = new int[num_tools];
	std::vector<ToolResult> unique_solutions;
	cv::Mat* tool_transforms = new cv::Mat[num_tools];
	for (int i = 0; i < num_tools; i++)
	{
		tool_transforms[i] = cv::Mat::zeros(cv::Size(8, 1), CV_32F);
	}
	for (int i = 0; i < num_tools; i++)
	{
		tool_solutions[i] = 0;
		ToolResultContainer tool_results = raw_solutions[i];

		if (tool_results.candidates.size() == 0)
			continue;

		std::vector<ToolResult> ordered_candidates = tool_results.candidates;
		std::sort(ordered_candidates.begin(), ordered_candidates.end(), &ToolResult::compare);
		std::vector<int> unique_primes;

		for (ToolResult candidate : ordered_candidates)
		{
			int prime = 1;
			for (int index : candidate.sphere_ids)
			{
				prime *= m_prime_numbers[index];
			}
			if (std::find(unique_primes.begin(), unique_primes.end(), prime) != unique_primes.end()) {
				continue;
			}
			candidate.tool_id = i;
			unique_solutions.push_back(candidate);
			unique_primes.push_back(prime);
			tool_solutions[i]++;
		}
	}

	std::sort(unique_solutions.begin(), unique_solutions.end(), &ToolResult::compare);

	while (unique_solutions.size() > 0)
	{
		ToolResult current = unique_solutions.front();
		int cur_toolid = current.tool_id;
		unique_solutions.erase(unique_solutions.begin());
		tool_transforms[cur_toolid] = MatchPointsKabsch(m_Tools[cur_toolid], frame, current.sphere_ids);

		std::vector<ToolResult> remaining_unique_solutions;
		for (ToolResult next_check : unique_solutions) {
			if (next_check.tool_id == cur_toolid)
				continue;

			bool used = false;
			for (auto cursphere : current.sphere_ids)
			{
				if (used)
				{
					break;
				}
				for (auto nexsphere : next_check.sphere_ids)
				{
					if (cursphere == nexsphere)
					{
						used = true;
						break;
					}

				}
			}
			// std::vector<int> intersection;
			//std::set_intersection(current.sphere_ids.begin(), current.sphere_ids.end(), next_check.sphere_ids.begin(), next_check.sphere_ids.end(), intersection.begin());
			//if (intersection.size() > 0)
			//	continue;
			if (used)
			{
				continue;
			}
			remaining_unique_solutions.push_back(next_check);
		}
		unique_solutions = remaining_unique_solutions;
	}
	delete[] tool_solutions;
	return tool_transforms;
}

cv::Mat IRToolTracker::MatchPointsKabsch(IRToolDefinition tool, ProcessedAHATFrame frame, std::vector<int> sphere_ids) {
#if DEBUG_OUTPUT
	OutputDebugString(L"MatchPointsKabsch\n");
#endif
	int num_points = tool.num_spheres;
	cv::Mat p = cv::Mat(num_points, 3, CV_32F);
	cv::Mat q = cv::Mat(num_points, 3, CV_32F);
	cv::Vec3f p_center = cv::Vec3f(0.f);
	cv::Vec3f q_center = cv::Vec3f(0.f);

	auto it_spheres_xyz = frame.spheres_xyz_per_mm.find(tool.sphere_radius);
	auto frame_spheres_xyz = it_spheres_xyz->second;

	cv::Mat hololens_pose_mm = frame.hololens_pose.clone();

	hololens_pose_mm.at<float>(0, 3) = hololens_pose_mm.at<float>(0, 3) * 1000.f;
	hololens_pose_mm.at<float>(1, 3) = hololens_pose_mm.at<float>(1, 3) * 1000.f;
	hololens_pose_mm.at<float>(2, 3) = hololens_pose_mm.at<float>(2, 3) * 1000.f;


	for (int i = 0; i < num_points; i++) {
		cv::Vec3f sphere = tool.spheres_xyz.at<cv::Vec3f>(i, 0);
		p.at<float>(i, 0) = sphere[0];
		p.at<float>(i, 1) = sphere[1];
		p.at<float>(i, 2) = sphere[2];
		p_center[0] += sphere[0];
		p_center[1] += sphere[1];
		p_center[2] += sphere[2];

		int frame_sphere_id = sphere_ids.at(i);
		cv::Vec3f sphere_frame = frame_spheres_xyz.at<cv::Vec3f>(frame_sphere_id, 0);

		cv::Mat sphere_frame_mat = cv::Mat(4, 1, CV_32F);
		sphere_frame_mat.at<float>(0, 0) = sphere_frame[0];
		sphere_frame_mat.at<float>(0, 1) = sphere_frame[1];
		sphere_frame_mat.at<float>(0, 2) = sphere_frame[2];
		sphere_frame_mat.at<float>(0, 3) = 1.f;



		cv::Mat sphere_world_mat = hololens_pose_mm * sphere_frame_mat;
		cv::Vec3f sphere_world = cv::Vec3f(sphere_world_mat.at<float>(0, 0), sphere_world_mat.at<float>(1, 0), sphere_world_mat.at<float>(2, 0));

		//Filter the resulting world position
#if DEBUG_NO_FILTER
		if (false)
#endif
		sphere_world = tool.sphere_kalman_filters.at(i).FilterData(sphere_world);





		q.at<float>(i, 0) = sphere_world[0];
		q.at<float>(i, 1) = sphere_world[1];
		q.at<float>(i, 2) = sphere_world[2];
		q_center[0] += sphere_world[0];
		q_center[1] += sphere_world[1];
		q_center[2] += sphere_world[2];
	}

	p_center /= num_points;
	q_center /= num_points;

	cv::Mat p_residual = cv::Mat(p);
	cv::Mat q_residual = cv::Mat(q);
	for (int r = 0; r < p_residual.rows; ++r) {
		p_residual.at<float>(r, 0) -= p_center[0];
		p_residual.at<float>(r, 1) -= p_center[1];
		p_residual.at<float>(r, 2) -= p_center[2];
		q_residual.at<float>(r, 0) -= q_center[0];
		q_residual.at<float>(r, 1) -= q_center[1];
		q_residual.at<float>(r, 2) -= q_center[2];
	}

	//SVD
	cv::Mat cov = p_residual.t() * q_residual;
	cv::SVD svd(cov);

	//Find rotation
	double d = cv::determinant(svd.vt.t() * svd.u.t());

	if (d > 0)
		d = 1.0;
	else
		d = -1.0;

	cv::Mat I = cv::Mat::eye(3, 3, CV_32F);

	I.at<float>(2, 2) = (float)d;

	cv::Mat R = svd.vt.t() * I * svd.u.t();

	cv::Mat q_avgMat = cv::Mat(3, 1, CV_32F);
	cv::Mat p_avgMat = cv::Mat(3, 1, CV_32F);
	q_avgMat.at<float>(0, 0) = q_center[0];
	q_avgMat.at<float>(1, 0) = q_center[1];
	q_avgMat.at<float>(2, 0) = q_center[2];
	p_avgMat.at<float>(0, 0) = p_center[0];
	p_avgMat.at<float>(1, 0) = p_center[1];
	p_avgMat.at<float>(2, 0) = p_center[2];

	cv::Mat t = q_avgMat - (R * p_avgMat);


	//Build transformation matrix
	cv::Mat transform_matrix = cv::Mat::zeros(4, 4, CV_32F);

	R.copyTo(transform_matrix(cv::Rect_<float>(0, 0, 3, 3)));


	transform_matrix.at<float>(0, 3) = t.at<float>(0, 0) / 1000.f;
	transform_matrix.at<float>(1, 3) = t.at<float>(1, 0) / 1000.f;
	transform_matrix.at<float>(2, 3) = t.at<float>(2, 0) / 1000.f;
	transform_matrix.at<float>(3, 3) = 1.f;

	transform_matrix = FlipTransformRightLeft(transform_matrix);

	//Copy translation and convert mm to m
	cv::Vec3f position;
	position[0] = transform_matrix.at<float>(0, 3);
	position[1] = transform_matrix.at<float>(1, 3);
	position[2] = transform_matrix.at<float>(2, 3);

	//Create Quaternion
	cv::Vec4f quat;
	quat[3] = cv::sqrt(cv::max(0.f, 1.f + transform_matrix.at<float>(0, 0) + transform_matrix.at<float>(1, 1) + transform_matrix.at<float>(2, 2))) / 2.f;
	quat[0] = cv::sqrt(cv::max(0.f, 1.f + transform_matrix.at<float>(0, 0) - transform_matrix.at<float>(1, 1) - transform_matrix.at<float>(2, 2))) / 2.f;
	quat[1] = cv::sqrt(cv::max(0.f, 1.f - transform_matrix.at<float>(0, 0) + transform_matrix.at<float>(1, 1) - transform_matrix.at<float>(2, 2))) / 2.f;
	quat[2] = cv::sqrt(cv::max(0.f, 1.f - transform_matrix.at<float>(0, 0) - transform_matrix.at<float>(1, 1) + transform_matrix.at<float>(2, 2))) / 2.f;
	quat[0] *= (quat[0] * (transform_matrix.at<float>(2, 1) - transform_matrix.at<float>(1, 2))) >= 0.f ? 1.f : -1.f;
	quat[1] *= (quat[1] * (transform_matrix.at<float>(0, 2) - transform_matrix.at<float>(2, 0))) >= 0.f ? 1.f : -1.f;
	quat[2] *= (quat[2] * (transform_matrix.at<float>(1, 0) - transform_matrix.at<float>(0, 1))) >= 0.f ? 1.f : -1.f;

#if DEBUG_NO_FILTER
	if (false)
#endif
		position = tool.kalman_filter_position.FilterData(position);

	cv::Mat position_rotation = cv::Mat::zeros(8, 1, CV_32F);
	//Position in xyz
	position_rotation.at<float>(0, 0) = position[0];
	position_rotation.at<float>(1, 0) = position[1];
	position_rotation.at<float>(2, 0) = position[2];
	//Quaternion
	position_rotation.at<float>(3, 0) = quat[0];
	position_rotation.at<float>(4, 0) = quat[1];
	position_rotation.at<float>(5, 0) = quat[2];
	position_rotation.at<float>(6, 0) = quat[3];
	//Last float is used to determine visibility of the tool
	position_rotation.at<float>(7, 0) = 1.f;

	return position_rotation;


}

cv::Mat IRToolTracker::FlipTransformRightLeft(cv::Mat transform_rhs)
{
	//Bring to unity coordinate system
	cv::Mat flipz = cv::Mat::ones(4, 4, CV_32F);
	flipz.at<float>(2, 0) = -1.f;
	flipz.at<float>(0, 2) = -1.f;
	flipz.at<float>(2, 1) = -1.f;
	flipz.at<float>(1, 2) = -1.f;
	flipz.at<float>(2, 3) = -1.f;
	cv::Mat transform_lhs = transform_rhs.mul(flipz);
	return transform_lhs;
}

void IRToolTracker::ConstructMap(cv::Mat3f spheres_xyz, int num_spheres, cv::Mat& map, std::vector<Side>& ordered_sides)
{
#if DEBUG_OUTPUT
	OutputDebugString(L"ConstructMap\n");
#endif
	for (int i = 0; i < num_spheres; i++) {
		for (int j = 0; j < num_spheres; j++) {
			float distance = cv::norm(spheres_xyz.at<cv::Vec3f>(i, 0) - spheres_xyz.at<cv::Vec3f>(j, 0));
			map.at<float>(i, j) = distance;
			if (i == j)
				continue;
			Side s{};
			s.distance = distance;
			s.id_from = i;
			s.id_to = j;
			if (ordered_sides.size() < 1)
				ordered_sides.push_back(s);
			else {
				if (distance >= ordered_sides.back().distance)
					ordered_sides.push_back(s);
				else if (distance <= ordered_sides.front().distance)
					ordered_sides.insert(ordered_sides.begin(), s);
				else {
					auto it = std::upper_bound(ordered_sides.cbegin(), ordered_sides.cend(), s, &Side::compare);
					ordered_sides.insert(it, s);
				}
			}

		}
	}
}

void IRToolTracker::AddFrame(void* pAbImage, void* pDepth, UINT32 depthWidth, UINT32 depthHeight, cv::Mat _pose, long long _timestamp) {

#if DEBUG_OUTPUT
	OutputDebugString(L"Add Frame\n");
#endif
	cv::Mat cvAbImage_origin(512, 512, CV_16UC1, (void*)pAbImage);
	cv::Mat cvAbImage = cvAbImage_origin.clone();

	m_MutexCurFrame.lock();

	if (m_CurrentFrame != nullptr) {
		delete[] m_CurrentFrame->pDepth;
		delete m_CurrentFrame;
	}

	m_CurrentFrame = new AHATFrame { _timestamp, _pose, cvAbImage,  new UINT16[depthWidth * depthHeight], depthWidth, depthHeight };
	memcpy(m_CurrentFrame->pDepth, pDepth, depthWidth * depthHeight * sizeof(UINT16));

	m_MutexCurFrame.unlock();
	

}

bool IRToolTracker::ProcessFrame(AHATFrame* rawFrame, ProcessedAHATFrame &result) {
#if DEBUG_OUTPUT
	OutputDebugString(L"ProcessFrame\n");
#endif


	ushort lowerLimit = 256 * 1.5;
	ushort upperLimit = 256 * 20;
	int minSize = 10, maxSize = 180;
	cv::Mat labels, stats, centroids;
	std::vector<float> irToolCenters;
	
	rawFrame->cvAbImage.forEach<ushort>(
		[&](ushort& ir, const int* position) -> void {
			ir = (std::clamp(ir, lowerLimit, upperLimit) - lowerLimit) * (256 * 256 / upperLimit) / (upperLimit - lowerLimit);
		}
	);
	
	rawFrame->cvAbImage.convertTo(rawFrame->cvAbImage, CV_8UC1);

	int areaCount = cv::connectedComponentsWithStats(rawFrame->cvAbImage, labels, stats, centroids, 8);


	for (int i = 1; i < areaCount; ++i)
	{
		auto area = stats.at<int32_t>(i, cv::CC_STAT_AREA);
		if (area <= maxSize && area >= minSize)
		{
			double _u = centroids.at<double>(i, 0);
			double _v = centroids.at<double>(i, 1);
			float uv[2] = { _u + 0.5, _v + 0.5 };
			float xy[2] = { 0, 0 };
#if DEBUG_OUTPUT
			{
				auto finish_detail = std::chrono::high_resolution_clock::now();
				std::string my_str = "U: ";
				my_str += (std::to_string(uv[0]) + " V: " + std::to_string(uv[1]) + " X: " + std::to_string(xy[0]) + " Y: " + std::to_string(xy[1]) + "\n");
				std::wstring stemp = std::wstring(my_str.begin(), my_str.end());
				LPCWSTR my2 = stemp.c_str();
				OutputDebugString(my2);
			}
#endif
			m_pResearchMode->DepthMapImagePointToCameraUnitPlane(uv, xy);
#if DEBUG_OUTPUT
			{
				auto finish_detail = std::chrono::high_resolution_clock::now();
				std::string my_str = "U: ";
				my_str += (std::to_string(uv[0]) + " V: " + std::to_string(uv[1]) + " X: " + std::to_string(xy[0]) + " Y: " + std::to_string(xy[1]) + "\n");
				std::wstring stemp = std::wstring(my_str.begin(), my_str.end());
				LPCWSTR my2 = stemp.c_str();
				OutputDebugString(my2);
			}
#endif
			//UINT16 depth = pDepth[resolution.Width * (UINT16)(_v + 0.5) + (UINT16)(_u + 0.5)];

			//UINT16 depth = MIN(MIN(MIN(pDepth[resolution.Width * (UINT16)_v + (UINT16)_u],
			//    pDepth[resolution.Width * (UINT16)_v + (UINT16)_u + 1]),
			//    pDepth[resolution.Width * ((UINT16)_v + 1) + (UINT16)_u]),
			//    pDepth[resolution.Width * ((UINT16)_v + 1) + (UINT16)_u + 1]);

			float depth = (static_cast<float>(rawFrame->pDepth[rawFrame->depthWidth * (UINT16)_v + (UINT16)_u] +
				rawFrame->pDepth[rawFrame->depthWidth * (UINT16)_v + (UINT16)_u + 1] +
				rawFrame->pDepth[rawFrame->depthWidth * ((UINT16)_v + 1) + (UINT16)_u] +
				rawFrame->pDepth[rawFrame->depthWidth * ((UINT16)_v + 1) + (UINT16)_u + 1])) / 4.f;




			irToolCenters.push_back(xy[0]);
			irToolCenters.push_back(xy[1]);
			irToolCenters.push_back(depth);
#if DEBUG_OUTPUT
			{
				auto finish_detail = std::chrono::high_resolution_clock::now();
				std::string my_str = "X: ";
				my_str += (std::to_string(xy[0]) + " Y: " + std::to_string(xy[1]) + " Z: " + std::to_string(depth) + "\n");
				std::wstring stemp = std::wstring(my_str.begin(), my_str.end());
				LPCWSTR my2 = stemp.c_str();
				OutputDebugString(my2);
			}
#endif
		}
	}

	int irToolCentersSize = irToolCenters.size();

	cv::Mat3f spheres = cv::Mat3f(irToolCentersSize / 3, 1);
	int j_sphere = 0;
	for (int i_sphere = 0; i_sphere < irToolCentersSize; i_sphere += 3)
	{
		spheres.at<cv::Vec3f>(j_sphere, 0) = cv::Vec3f(irToolCenters.at(i_sphere), irToolCenters.at(i_sphere + 1), irToolCenters.at(i_sphere + 2));
		j_sphere++;
	}

	int num_spheres = spheres.size().height;

	if (num_spheres < 3) {
		//If theres less than 3 points visible, theres no tool to track
		//Free memory
		delete[] rawFrame->pDepth;
		delete rawFrame;
		return false;
	}

	//Create 3d coordinates for every possible sphere size

	std::map<float, std::vector<Side>> ordered_sides_per_mm;
	std::map<float, cv::Mat> map_per_mm;
	std::map<float, cv::Mat3f> spheres_xyz_per_mm;

	for (IRToolDefinition tool : m_Tools)
	{
		float cur_radius = tool.sphere_radius;
		if (!(spheres_xyz_per_mm.find(cur_radius) == spheres_xyz_per_mm.end())) {
			//We already created this map
			continue;
		}


		cv::Mat3f spheres_xyz = spheres.clone();


		spheres_xyz.forEach(
			[&](cv::Vec3f& xyz, const int* position) -> void {
				xyz[2] = xyz[2] + cur_radius;
		cv::Vec3f temp_vec(xyz[0], xyz[1], 1);
		xyz = cv::Vec3f((temp_vec / cv::norm(temp_vec)) * xyz[2]);
			}
		);

		//Construct map
		cv::Mat map(cv::Size(num_spheres, num_spheres), CV_32F);
		std::vector<Side> ordered_sides;

		ConstructMap(spheres_xyz, num_spheres, map, ordered_sides);

		ordered_sides_per_mm.insert({ cur_radius, ordered_sides });
		map_per_mm.insert({ cur_radius, map });
		spheres_xyz_per_mm.insert({ cur_radius, spheres_xyz });

#if DEBUG_OUTPUT
		std::string my_str = "Sphere Positions " + std::to_string(cur_radius) + " mm:\n";
		my_str << spheres_xyz;
		std::wstring stemp = std::wstring(my_str.begin(), my_str.end());
		LPCWSTR my2 = stemp.c_str();
		OutputDebugString(my2);
		OutputDebugString(L"\n");
#endif

	}


	result.timestamp = rawFrame->timestamp;
	result.hololens_pose = rawFrame->hololens_pose;
	result.num_spheres = static_cast<uint>(num_spheres);
	result.spheres_xyd = spheres.clone();
	result.spheres_xyz_per_mm = spheres_xyz_per_mm;
	result.ordered_sides_per_mm = ordered_sides_per_mm;
	result.map_per_mm = map_per_mm;


	//Free memory
	delete[] rawFrame->pDepth;
	delete rawFrame;

	return true;

}

bool IRToolTracker::AddTool(cv::Mat3f spheres, float sphere_radius, std::string identifier)
{
#if DEBUG_OUTPUT
	OutputDebugString(L"AddTool\n");
#endif
	//Do we already have this tool?
	if (m_ToolIndexMapping.count(identifier) > 0)
		return false;

	bool restartTracking = false;
	if (m_bIsCurrentlyTracking) {
		restartTracking = true;
		StopTracking();
	}
	IRToolDefinition tool{};
	tool.identifier = identifier;
	tool.num_spheres = spheres.size().height;
	tool.spheres_xyz = spheres;
	tool.sphere_radius = sphere_radius;

	m_ToolIndexMapping.insert({ identifier, m_Tools.size() });

	//Construct map
	cv::Mat map(cv::Size(tool.num_spheres, tool.num_spheres), CV_32F);
	std::vector<Side> ordered_sides;
	ConstructMap(spheres, tool.num_spheres, map, ordered_sides);
	tool.map = map.clone();
	tool.ordered_sides = ordered_sides;
	//One Kalman filter per sphere
	for (int i = 0; i < tool.num_spheres; i++) {
		tool.sphere_kalman_filters.push_back(IRToolKalmanFilter());
	}
	m_Tools.push_back(tool);
	OutputDebugString(L"On Device Tracking Added Tool\n");

#if DEBUG_OUTPUT
	OutputDebugString(L"New Tool Added:\n");
	std::string my_str = "Spheres:\n";
	my_str << tool.spheres_xyz;
	std::wstring stemp = std::wstring(my_str.begin(), my_str.end());
	LPCWSTR my2 = stemp.c_str();
	OutputDebugString(my2);
	OutputDebugString(L"\n");

	OutputDebugString(L"Tool Map Added:\n");
	my_str = "Map:\n";
	my_str << tool.map;
	stemp = std::wstring(my_str.begin(), my_str.end());
	my2 = stemp.c_str();
	OutputDebugString(my2);
	OutputDebugString(L"\n");
#endif

	if (restartTracking) {
		StartTracking();
	}
	return true;
}

bool IRToolTracker::RemoveTool(std::string identifier)
{
#if DEBUG_OUTPUT
	OutputDebugString(L"RemoveTool\n");
#endif
	//Do we even have this tool?
	if (m_ToolIndexMapping.count(identifier) == 0)
		return false;


	auto it = m_ToolIndexMapping.find(identifier);
	int index = it->second;

	bool restartTracking = false;
	if (m_bIsCurrentlyTracking) {
		restartTracking = true;
		StopTracking();
	}

	m_ToolIndexMapping.erase(identifier);
	std::vector<IRToolDefinition> oldTools(m_Tools);
	std::map<std::string, int> oldMapping(m_ToolIndexMapping);
	m_Tools.clear();
	m_ToolIndexMapping.clear();
	for (auto pair : oldMapping)
	{
		m_ToolIndexMapping.insert({ pair.first, m_Tools.size() });
		m_Tools.push_back(oldTools.at(pair.second));
	}


	if (restartTracking) {

		StartTracking();
	}
	return true;
}

bool IRToolTracker::RemoveAllToolDefinitions()
{
#if DEBUG_OUTPUT
	OutputDebugString(L"RemoveAllTools\n");
#endif
	m_Tools.clear();
	m_ToolIndexMapping.clear();
	return true;
}

bool IRToolTracker::StartTracking() {
#if DEBUG_OUTPUT
	OutputDebugString(L"StartTracking\n");
#endif
	if (m_bIsCurrentlyTracking || m_Tools.size() == 0)
		return false;
	m_bShouldStop = false;
	m_TrackingThread = std::thread(&IRToolTracker::TrackTools, this);
	return true;
}
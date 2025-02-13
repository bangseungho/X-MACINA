#include "EnginePch.h"
#include "Component/Agent.h"
#include "Component/Camera.h"
#include "Component/Collider.h"
#include "KdTree.h"

#include "DXGIMgr.h"
#include "Scene.h"
#include "Timer.h"
#include "VoxelManager.h"
#include "InputMgr.h"

namespace {
	float HeuristicManhattan(const Pos& start, const Pos& dest) {
		return static_cast<float>(std::abs(dest.X - start.X) +
			std::abs(dest.Y - start.Y) +
			std::abs(dest.Z - start.Z));
	}
	float HeuristicEuclidean(const Pos& start, const Pos& dest) {
		return static_cast<float>(std::sqrt(std::pow(dest.X - start.X, 2) +
			std::pow(dest.Y - start.Y, 2) +
			std::pow(dest.Z - start.Z, 2)));
	}
}

void Agent::ClearPathList()
{
	for (auto& voxel : mCloseList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
	}

	for (auto& voxel : mOpenList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
	}

	mCloseList.clear();
	mOpenList.clear();
}

void Agent::Start()
{
	AgentManager::I->AddAgent(this);
	mCloseList.reserve(10000);
	mOpenList.reserve(10000);
	mPathDir = mObject->GetLook();

	Matrix mtxWorld = mObject->GetWorldTransform();
	mtxWorld = mtxWorld.CreateScale(0.4f);
	SetWorldMatrix(mtxWorld);
	mObject->GetComponent<ObjectCollider>()->SetScale(0.4f);
	mObject->SetPosition(100, 0, 260);
	mMaxNeighbors = 1;
	mNeighborDist = 5.f;
	mTimeHorizon = 2.5f;
	mRadius = 0.2f;
	mMaxSpeed = 3.5f;
	mTarget = mObject->GetPosition();
}

bool Compare(float a, float b, int flag)
{
	if (flag == 1) {
		return a > b;
	}
	else if (flag == -1) {
		return a < b;
	}
	return false;
}

void Agent::UpdatePosition()
{
	const Vec3& objectPos = mObject->GetPosition();
	mVoxelIndex = Scene::I->GetVoxelIndex(objectPos);

	if (!PathOption::I->GetStartFlag()) {
		mIsStart = true;
	}

	if (!mIsStart) {
		return;
	}

	mNewVelocity.y = mPrefVelocity.y;
	mVelocity = mNewVelocity;

	//static const float mdx[4]{ -0.1f, +0.1f, -0.1f, +0.1f };
	//static const float mdz[4]{ -0.1f, -0.1f, +0.1f, +0.1f };
	//static const float odx[4]{ +0.25f, -0.25f, +0.25f, -0.25f };
	//static const float odz[4]{ +0.25f, +0.25f, -0.25f, -0.25f };
	//static const int idx[4]{ +1, -1, +1, -1 };
	//static const int idz[4]{ +1, +1, -1, -1 };

	//const Pos& crntIndex = Scene::I->GetVoxelIndex(objectPos);
	//Vec3 nextPos = objectPos + mVelocity * DeltaTime();


	//for (int i = 0; i < 4; ++i) {
	//	const Vec3& vertexPos = nextPos + Vec3{ mdx[i], 0.f, mdz[i] };
	//	const Pos& vertexIndex = Scene::I->GetVoxelIndex(vertexPos);

	//	if (Scene::I->CanGoNextVoxel(vertexIndex.Up())) {
	//		continue;
	//	}

	//	const Vec3& obstaclePos = Scene::I->GetVoxelPos(vertexIndex);

	//	int both{};
	//	if (Compare(objectPos.x + mdx[i], obstaclePos.x + odx[i], idx[i])) {
	//		mVelocity.x = 0.f;
	//		both++;
	//	}
	//	if (Compare(objectPos.z + mdz[i], obstaclePos.z + odz[i], idz[i])) {
	//		mVelocity.z = 0.f;
	//		both++;
	//	}

	//	Pos neighborX = vertexIndex + Pos{ 0, idx[i], 1 };
	//	Pos neighborZ = vertexIndex + Pos{ idz[i], 0, 1 };
	//	if (both == 2) {
	//		if (!Scene::I->CanGoNextVoxel(neighborZ) && Scene::I->CanGoNextVoxel(neighborX)) {
	//			mVelocity.z = mNewVelocity.z;
	//		}
	//		else if (!Scene::I->CanGoNextVoxel(neighborX) && Scene::I->CanGoNextVoxel(neighborZ)) {
	//			mVelocity.x = mNewVelocity.x;
	//		}
	//	}
	//}

	//nextPos = objectPos + mVelocity * DeltaTime();
	//for (int i = 0; i < 4; ++i) {
	//	const Vec3& vertexPos = nextPos + Vec3{ mdx[i], 0.f, mdz[i] };
	//	const Pos& vertexIndex = Scene::I->GetVoxelIndex(vertexPos);

	//	if (!Scene::I->CanGoNextVoxel(vertexIndex.Up())) {
	//		mVelocity = Vec3{};
	//		break;
	//	}
	//}

	mObject->SetPosition(objectPos + mVelocity * DeltaTime());
}

const Pos Agent::GetPathIndex(int index) const
{
	if (!mGlobalPath.empty() && mGlobalPath.size() > index) {
		return Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - index - 1]);
	}

	return Pos{};
}


void Agent::SetTarget(const Vec3& target)
{
	mTarget = target;
	mCloseList.push_back(Scene::I->GetVoxelIndex(mTarget));
}

std::vector<Vec3> Agent::PathPlanningToAstar(const Pos& dest, const std::unordered_map<Pos, int>& avoidCostMap, bool clearPathList, bool inputDest, int maxOpenNodeCount)
{
	if (clearPathList) {
		ClearPathList();
	}

	std::vector<Vec3> finalPath{};

	if (dest == mStart) {
		return finalPath;
	}

	mDest = dest;
	std::stack<Pos>	path{};
	std::unordered_map<Pos, Pos>	parent;
	std::unordered_map<Pos, float>	distance;
	std::unordered_map<Pos, bool>	visited;

	std::function<float(const Pos&, const Pos&)> heuristic{};

	switch (mOption.Heuri) {
	case Heuristic::Euclidean:
		heuristic = HeuristicEuclidean;
		break;
	case Heuristic::Manhattan:
		heuristic = HeuristicManhattan;
		break;
	default:
		break;
	}

	// f = g + h
	std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
	float g = 0;
	float h = heuristic(mStart, dest) * PathOption::I->GetHeuristicWeight();
	pq.push({ g + h, g, mStart });
	distance[mStart] = g + h;
	parent[mStart] = mStart;

	Pos prevDir;
	PQNode curNode{};
	int openNodeCount{};
	bool failedPlanningPath{};
	while (!pq.empty()) {
		curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (openNodeCount >= maxOpenNodeCount) {failedPlanningPath = true; break; }
		if (visited.contains(curNode.Pos)) continue;
		if (distance[curNode.Pos] < curNode.F) continue;
		if (CheckCurNodeContainPathCache(curNode.Pos)) break;
		if (curNode.Pos == dest) break;

		visited[curNode.Pos] = true;

		for (int dir = 0; dir < 8; ++dir) {
			Pos nextPosZX = curNode.Pos + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Pos nextPos = Pos{ it->first.first, it->first.second, it->second };
				int diffPosY = abs(nextPos.Y - curNode.Pos.Y);
				int avoidCost{}, dirPathCost{};
				int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
				float edgeCost = GetEdgeCost(nextPos, gkFront[dir]) * PathOption::I->GetEdgeWeight();
				if (diffPosY > mOption.AllowedHeight) continue;
				if (visited.contains(nextPos)) continue;
				if (!distance.contains(nextPos)) distance[nextPos] = FLT_MAX;
				if (prevDir != gkFront[dir]) dirPathCost = gkCost[dir];

				auto findIt = avoidCostMap.find(nextPos.XZ());
				if (findIt != avoidCostMap.end()) {
					avoidCost = avoidCostMap.at(nextPos.XZ());
				}

				float g = curNode.G + gkCost[dir] + avoidCost + dirPathCost + proximityCost + edgeCost;
				float h = (heuristic(nextPos, dest) + mOpenListMinusCost[nextPos] + mPrevPathMinusCost[nextPos]) * PathOption::I->GetHeuristicWeight();

				if (g + h < distance[nextPos]) {
					distance[nextPos] = g + h;
					pq.push({ g + h, g, nextPos });
					parent[nextPos] = curNode.Pos;
					openNodeCount++;
					mOpenList.push_back(nextPos);
				}
			}
		}
	}

	// 경로 설정 실패
	if (failedPlanningPath || pq.empty()) {
		ClearPathList();
		ClearPath();
		mDest = AgentManager::I->FindEmptyDestVoxel(this);
		return finalPath;
	}

	Pos pos = curNode.Pos;
	prevDir.Init();

	// 도착점을 넣지 않을 경우
	if (!inputDest) {
		pos = parent[pos];
	}

	// 부모를 통해 경로 설정
	while (pos != parent[pos]) {
		Pos dir = parent[pos] - pos;

		if (!PathOption::I->GetDirPathOptimize() || prevDir != dir) {
			path.push(pos);
		}

		pos = parent[pos];
		prevDir = dir;
	}

	// 시작점이 적용되지 않을 수 있음
	if (!path.empty() && path.top() != mStart && inputDest) {
		path.push(mStart);
	}

	// 광선을 이용한 경로 최소화
	if (PathOption::I->GetRayPathOptimize()) {
		RayPathOptimize(path, dest);
	}

	// 최종 경로 설정
	while (!path.empty()) {
		const Pos& now = path.top();
		mCloseList.push_back(now);
		mGlobalPathCache.insert({ now, static_cast<int>(finalPath.size()) });
		finalPath.push_back(Scene::I->GetVoxelPos(now));
		path.pop();
	}

	// 경로 캣멀롬 곡선화
	if (PathOption::I->GetSplinePath()) {
		MakeSplinePath(finalPath);
	}

	if (!finalPath.empty()) {
		mPathDir = Vector3::Normalized(finalPath.back() - mObject->GetPosition());
	}

	std::reverse(finalPath.begin(), finalPath.end());

	// 음수 가중치 적용
	for (const Pos& openList : mOpenList) {
		mOpenListMinusCost[openList] = -10;
	}
	for (const Vec3& path : finalPath) {
		mPrevPathMinusCost[Scene::I->GetVoxelIndex(path)] = -20;
	}

	return finalPath;
}

void Agent::ReadyPlanningToPath(const Pos& start)
{
	mStart = start;
	mIsStart = false;
	mCloseList.push_back(mStart);
	mNewVelocity = Vec3{};
	mPrefVelocity = Vec3{};
	mVelocity = Vec3{};
	ClearPath();
	mOption.Heuri = Heuristic::Manhattan;
}

bool Agent::CheckCurNodeContainPathCache(const Pos& curNode)
{
	if (mGlobalPathCache.count(curNode)) {
		while (mGlobalPath.size() > 1 && Scene::I->GetVoxelIndex(mGlobalPath.back()) != curNode) {
			mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
			mGlobalPath.pop_back();
		}
		return true;
	}
	return false;
}

void Agent::RayPathOptimize(std::stack<Pos>& path, const Pos& dest)
{
	std::stack<Pos> optimizePath{};

	// 현재 시작 지점 설정
	Pos now{};
	if (!path.empty()) {
		now = path.top();
		path.pop();
		optimizePath.push(now);
	}

	Pos prev = now;
	while (!path.empty()) {
		Pos next = path.top();
		Ray ray{};
		ray.Position = Scene::I->GetVoxelPos(now);
		ray.Direction = Scene::I->GetVoxelPos(next) - Scene::I->GetVoxelPos(now);
		ray.Direction.Normalize();

		Pos startPoint = Pos::Min(now, next);
		Pos endPoint = Pos::Max(now, next);
		for (int z = startPoint.Z; z <= endPoint.Z; ++z) {
			for (int x = startPoint.X; x <= endPoint.X; ++x) {
				for (int y = startPoint.Y; y <= endPoint.Y; ++y) {
					const Pos voxel = Pos{ z, x, y };
					VoxelState state = Scene::I->GetVoxelState(voxel); float dist;
					if (ray.Intersects(BoundingBox{ Scene::I->GetVoxelPos(voxel), Grid::mkVoxelExtent }, dist)) {
						if (state == VoxelState::None) {
							optimizePath.push(prev);
							now = prev;
							path.pop();
							goto NoOptimizePath;
						}
						else if (state == VoxelState::Static) {
							if (/*GetOnVoxelCount(voxel) >= mOption.AllowedHeight ||*/ prev.Y != y) {
								optimizePath.push(prev);
								now = prev;
								goto NoOptimizePath;
							}
						}
					}
				}
			}
		}
		path.pop();

	NoOptimizePath:
		prev = next;
	}

	optimizePath.push(dest);

	while (!optimizePath.empty()) {
		path.push(optimizePath.top());
		optimizePath.pop();
	}
}

static Vec3 QuadraticBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, float t) {
	float u = 1.0f - t;
	return (p0 * (u * u)) + (p1 * (2 * u * t)) + (p2 * (t * t));
}

static Vec3 CubicBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t) {
	float u = 1.0f - t;
	float uu = u * u;
	float uuu = uu * u;
	float tt = t * t;
	float ttt = tt * t;
	return (p0 * uuu) + (p1 * (3 * uu * t)) + (p2 * (3 * u * tt)) + (p3 * ttt);
}

void Agent::MakeSplinePath(std::vector<Vec3>& path)
{
	if (path.size() <= 2) {
		return;
	}

	std::vector<Vec3> splinePath{};
	const int pathSize = static_cast<int>(path.size());
	const int sampleCount = 10;

	Vec3 p4 = path.back() + path[pathSize - 1].xz() - path[pathSize - 2].xz();
	path.push_back(p4);

	for (int i = 0; i < pathSize - 2; ++i) {
		Vec3 p0 = path[i];
		Vec3 p1 = path[i + 1];
		Vec3 p2 = path[i + 2];
		Vec3 p3 = path[i + 3];

		for (int j = 0; j <= sampleCount; ++j) {
			float t = static_cast<float>(j) / sampleCount;
			splinePath.push_back(Vec3::CatmullRom(p0, p1, p2, p3, t));
		}
	}
	path = splinePath;
}

void Agent::RePlanningToPathAvoidStatic(const Pos& crntPathIndex)
{
	for (int i = mkAvoidForwardStaticObjectCount; i > 0; --i) {
		if (mGlobalPath.size() <= i) {
			continue;
		}

		Pos nextPathIndex = Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - i]);
		VoxelState nextPathUpVoxelState = Scene::I->GetVoxelState(nextPathIndex.Up());
		if (!Scene::I->CanGoNextVoxel(nextPathIndex)) {
			for (int j = 0; j < i; ++j) {
				mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
				mGlobalPath.pop_back();
			}

			mStart = crntPathIndex;
			mOption.Heuri = Heuristic::Euclidean;
			mLocalPath = PathPlanningToAstar(mDest, {}, false, false);
			std::copy(mLocalPath.begin(), mLocalPath.end(), std::back_inserter(mGlobalPath));
			std::cout << "Create Path Count : " << mLocalPath.size() << '\n';
			return;
		}
	}
}

void Agent::RePlanningToPathAvoidDynamic()
{
	if (mGlobalPath.size() <= 1) {
		return;
	}

	const Pos& nextPathIndex = GetPathIndex(1);
	const std::unordered_map<Pos, int>& costMap = AgentManager::I->CheckAgentIndex(nextPathIndex, this);
	if (costMap.empty()) {
		return;
	}

	bool inputDest{};
	if (!mGlobalPath.empty()) {
		mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
		mGlobalPath.pop_back();
	}
	else {
		inputDest = true;
	}

	mOption.Heuri = Heuristic::Euclidean;
	mStart = mVoxelIndex;
	mLocalPath = PathPlanningToAstar(mDest, costMap, false, inputDest, 100);

	mSlowSpeedCount = static_cast<int>(mLocalPath.size());
	std::copy(mLocalPath.begin(), mLocalPath.end(), std::back_inserter(mGlobalPath));
}

float Agent::GetEdgeCost(const Pos& nextPos, const Pos& dir)
{
	float cost{};
	if (dir.X != 0 && dir.Z != 0) {
		cost = (Scene::I->GetEdgeCost(nextPos, true) + Scene::I->GetEdgeCost(nextPos, true)) / 2.f;
	}
	else if (dir.Z != 0 && dir.X == 0) {
		cost = Scene::I->GetEdgeCost(nextPos, true);
	}
	else if (dir.X != 0 && dir.Z == 0) {
		cost = Scene::I->GetEdgeCost(nextPos, false);
	}

	return cost;
}

void Agent::InsertAgentNeightbor(const Agent* agent, float& rangeSq)
{
	if (this != agent) {
		const float distSq = Vec3::AbsSq(mObject->GetPosition() - agent->GetWorldPosition());

		if (distSq < rangeSq) {
			if (mAgentNeighbors.size() < mMaxNeighbors) {
				mAgentNeighbors.push_back(std::make_pair(distSq, agent));
			}
			
			int i = mAgentNeighbors.size() - 1;
			
			while (i != 0 && distSq < mAgentNeighbors[i - 1].first) {
				mAgentNeighbors[i] = mAgentNeighbors[i - 1];
				--i;
			}

			mAgentNeighbors[i] = std::make_pair(distSq, agent);

			if (mAgentNeighbors.size() == mMaxNeighbors) {
				rangeSq = mAgentNeighbors.back().first;
			}
		}
	}
}

void Agent::ComputeNeighbors()
{
	mAgentNeighbors.clear();

	if (mMaxNeighbors > 0) {
		AgentManager::I->mKdTree->ComputeAgentNeighbors(this, mNeighborDist * mNeighborDist);
	}
}

bool LinearProgram1(const std::vector<Plane>& planes, std::size_t planeNo, const Line& line, float radius, const Vec3& optVelocity, bool directionOpt, Vec3& result) { /* NOLINT(runtime/references) */
	const float dotProduct = Vec3::Multiply(line.Point, line.Direction);
	const float discriminant = dotProduct * dotProduct + radius * radius - Vec3::AbsSq(line.Point);

	if (discriminant < 0.0F) {
		/* Max speed sphere fully invalidates line. */
		return false;
	}

	const float sqrtDiscriminant = std::sqrt(discriminant);
	float tLeft = -dotProduct - sqrtDiscriminant;
	float tRight = -dotProduct + sqrtDiscriminant;

	for (std::size_t i = 0U; i < planeNo; ++i) {
		const float numerator = Vec3::Multiply((planes[i].Point - line.Point), planes[i].Normal);
		const float denominator = Vec3::Multiply(line.Direction, planes[i].Normal);

		if (denominator * denominator <= FLT_EPSILON) {
			/* Lines line is (almost) parallel to plane i. */
			if (numerator > 0.0F) {
				return false;
			}

			continue;
		}

		const float t = numerator / denominator;

		if (denominator >= 0.0F) {
			/* Plane i bounds line on the left. */
			tLeft = max(tLeft, t);
		}
		else {
			/* Plane i bounds line on the right. */
			tRight = min(tRight, t);
		}

		if (tLeft > tRight) {
			return false;
		}
	}

	if (directionOpt) {
		/* Optimize direction. */
		if (Vec3::Multiply(optVelocity, line.Direction) > 0.0F) {
			/* Take right extreme. */
			result = line.Point + tRight * line.Direction;
		}
		else {
			/* Take left extreme. */
			result = line.Point + tLeft * line.Direction;
		}
	}
	else {
		/* Optimize closest point. */
		const float t = Vec3::Multiply(line.Direction, (optVelocity - line.Point));

		if (t < tLeft) {
			result = line.Point + tLeft * line.Direction;
		}
		else if (t > tRight) {
			result = line.Point + tRight * line.Direction;
		}
		else {
			result = line.Point + t * line.Direction;
		}
	}

	return true;
}


bool LinearProgram2(const std::vector<Plane>& planes, std::size_t planeNo,
	float radius, const Vec3& optVelocity, bool directionOpt,
	Vec3& result) { /* NOLINT(runtime/references) */
	const float planeDist = Vec3::Multiply(planes[planeNo].Point, planes[planeNo].Normal);
	const float planeDistSq = planeDist * planeDist;
	const float radiusSq = radius * radius;

	if (planeDistSq > radiusSq) {
		/* Max speed sphere fully invalidates plane planeNo. */
		return false;
	}

	const float planeRadiusSq = radiusSq - planeDistSq;

	const Vec3 planeCenter = planeDist * planes[planeNo].Normal;

	if (directionOpt) {
		/* Project direction optVelocity on plane planeNo. */
		const Vec3 planeOptVelocity =
			optVelocity -
			(optVelocity * planes[planeNo].Normal) * planes[planeNo].Normal;
		const float planeOptVelocityLengthSq = Vec3::AbsSq(planeOptVelocity);

		if (planeOptVelocityLengthSq <= FLT_EPSILON) {
			result = planeCenter;
		}
		else {
			result =
				planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq) *
				planeOptVelocity;
		}
	}
	else {
		/* Project point optVelocity on plane planeNo. */
		result = optVelocity +
			((planes[planeNo].Point - optVelocity) * planes[planeNo].Normal) *
			planes[planeNo].Normal;

		/* If outside planeCircle, project on planeCircle. */
		if (Vec3::AbsSq(result) > radiusSq) {
			const Vec3 planeResult = result - planeCenter;
			const float planeResultLengthSq = Vec3::AbsSq(planeResult);
			result = planeCenter +
				std::sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
		}
	}

	for (std::size_t i = 0U; i < planeNo; ++i) {
		if (Vec3::Multiply(planes[i].Normal, (planes[i].Point - result)) > 0.0F) {
			/* Result does not satisfy constraint i. Compute new optimal result.
			 * Compute intersection line of plane i and plane planeNo.
			 */
			Vec3 crossProduct = planes[i].Normal.Cross(planes[planeNo].Normal);

			if (Vec3::AbsSq(crossProduct) <= FLT_EPSILON) {
				/* Planes planeNo and i are (almost) parallel, and plane i fully
				 * invalidates plane planeNo.
				 */
				return false;
			}

			Line line;
			line.Direction = Vector3::Normalized(crossProduct);
			const Vec3 lineNormal = line.Direction.Cross(planes[planeNo].Normal);
			line.Point = planes[planeNo].Point + (((planes[i].Point - planes[planeNo].Point) * planes[i].Normal) / (lineNormal * planes[i].Normal)) * lineNormal;

			if (!LinearProgram1(planes, i, line, radius, optVelocity, directionOpt,
				result)) {
				return false;
			}
		}
	}

	return true;
}

std::size_t LinearProgram3(const std::vector<Plane>& planes, float radius,
	const Vec3& optVelocity, bool directionOpt,
	Vec3& result) { /* NOLINT(runtime/references) */

	if (directionOpt) {
		/* Optimize direction. Note that the optimization velocity is of unit length
		 * in this case.
		 */
		result = optVelocity * radius;
	}
	else if (Vec3::AbsSq(optVelocity) > radius * radius) {
		/* Optimize closest point and outside circle. */
		result = Vector3::Normalized(optVelocity) * radius;
	}
	else {
		/* Optimize closest point and inside circle. */
		result = optVelocity;
	}

	for (std::size_t i = 0U; i < planes.size(); ++i) {
		if (Vec3::Multiply(planes[i].Normal, (planes[i].Point - result)) > 0.0F) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			const Vec3 tempResult = result;

			if (!LinearProgram2(planes, i, radius, optVelocity, directionOpt,
				result)) {
				result = tempResult;
				return i;
			}
		}
	}

	return planes.size();
}

void LinearProgram4(const std::vector<Plane>& planes, std::size_t beginPlane,
	float radius,
	Vec3& result) { /* NOLINT(runtime/references) */
	float distance = 0.0F;

	for (std::size_t i = beginPlane; i < planes.size(); ++i) {
		if (Vec3::Multiply(planes[i].Normal, (planes[i].Point - result)) > distance) {
			/* Result does not satisfy constraint of plane i. */
			std::vector<Plane> projPlanes;

			for (std::size_t j = 0U; j < i; ++j) {
				Plane plane;

				const Vec3 crossProduct = planes[j].Normal.Cross(planes[i].Normal);

				if (Vec3::AbsSq(crossProduct) <= FLT_EPSILON) {
					/* Plane i and plane j are (almost) parallel. */
					if (Vec3::Multiply(planes[i].Normal, planes[j].Normal) > 0.0F) {
						/* Plane i and plane j point in the same direction. */
						continue;
					}

					/* Plane i and plane j point in opposite direction. */
					plane.Point = 0.5F * (planes[i].Point + planes[j].Point);
				}
				else {
					/* Plane.point is point on line of intersection between plane i and
					 * plane j.
					 */
					const Vec3 lineNormal = crossProduct.Cross(planes[i].Normal);
					plane.Point =
						planes[i].Point +
						(((planes[j].Point - planes[i].Point) * planes[j].Normal) /
							(lineNormal * planes[j].Normal)) *
						lineNormal;
				}

				plane.Normal = Vector3::Normalized(planes[j].Normal - planes[i].Normal);
				projPlanes.push_back(plane);
			}

			const Vec3 tempResult = result;

			if (LinearProgram3(projPlanes, radius, planes[i].Normal, true, result) <
				projPlanes.size()) {
				/* This should in principle not happen. The result is by definition
				 * already in the feasible region of this linear program. If it fails,
				 * it is due to small floating point error, and the current result is
				 * kept.
				 */
				result = tempResult;
			}

			distance = Vec3::Multiply(planes[i].Normal, (planes[i].Point - result));
		}
	}
} 

void Agent::ComputeNewVelocity()
{
	mORCAPlanes.clear();
	const float invTimeHorizon = 1.f / mTimeHorizon;

	for (int i = 0; i < mAgentNeighbors.size(); ++i) {
		const Agent* const other = mAgentNeighbors[i].second;
		const Vec3 relativePosition = other->GetWorldPosition() - mObject->GetPosition();
		const Vec3 relativeVelocity = mVelocity - other->GetVelocity();
		const float distSq = Vec3::AbsSq(relativePosition);
		const float combinedRadius = mRadius + other->GetRadius();
		const float combinedRadiusSq = combinedRadius * combinedRadius;

		Plane plane;
		Vec3 u;
		
		if (distSq > combinedRadiusSq) {
			const Vec3 w = relativeVelocity - invTimeHorizon * relativePosition;
			const float wLengthSq = Vec3::AbsSq(w);
			const float dotProduct = Vec3::Multiply(w, relativePosition);

			if (dotProduct < 0.f && dotProduct * dotProduct > combinedRadiusSq * wLengthSq) {
				const float wLength = std::sqrt(wLengthSq);
				const Vec3 unitW = w / wLength;

				plane.Normal = unitW;
				u = (combinedRadius * invTimeHorizon - wLength) * unitW;
			}
			else {
				/* Project on cone. */
				const float a = distSq;
				const float b = Vec3::Multiply(relativePosition, relativeVelocity);
				const float c = Vec3::AbsSq(relativeVelocity) - Vec3::AbsSq(relativePosition.Cross(relativeVelocity)) /(distSq - combinedRadiusSq);
				const float t = (b + std::sqrt(b * b - a * c)) / a;
				const Vec3 ww = relativeVelocity - t * relativePosition;
				const float wwLength = Vec3::Abs(ww);
				const Vec3 unitWW = ww / wwLength;

				plane.Normal = unitWW;
				u = (combinedRadius * t - wwLength) * unitWW;
			}
		}
		else {
			/* Collision. */
			const float invTimeStep = 1.0F / DeltaTime();
			const Vec3 w = relativeVelocity - invTimeStep * relativePosition;
			const float wLength = Vec3::Abs(w);
			const Vec3 unitW = w / wLength;

			plane.Normal = unitW;
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}

		plane.Point = mVelocity + 0.5f * u;
		mORCAPlanes.push_back(plane);
	}

	const std::size_t planeFail = LinearProgram3(mORCAPlanes, mMaxSpeed, mPrefVelocity, false, mNewVelocity);
	if (planeFail < mORCAPlanes.size()) {
		LinearProgram4(mORCAPlanes, planeFail, mMaxSpeed, mNewVelocity);
	}
}

void Agent::SetPreferredVelocity()
{
	const Vec3& toNext = AgentManager::I->GetFlowFieldDirection(mVoxelIndex);

	if (Vector3::IsZero(toNext)) {
		return;
	}

	Vec3 toDest = Scene::I->GetVoxelPos(mDest) - mObject->GetPosition();
	if (Vec3::AbsSq(toDest) > 1.f) {
		toDest = Vector3::Normalized(toNext);
	}

	mPrefVelocity = toDest * mOption.AgentSpeed;
}

void Agent::ClearPath()
{
	mGlobalPath.clear();
	mLocalPath.clear();
	mGlobalPathCache.clear();
	mPrevPathMinusCost.clear();
	mOpenListMinusCost.clear();
}

bool Agent::PickAgent()
{
	const Ray& ray = MAIN_CAMERA->ScreenToWorldRay(InputMgr::I->GetMousePos());

	float distance = 0.f;
	if (ray.Intersects(mObject->GetComponent<ObjectCollider>()->GetBS(), distance)) {
		return true;
	}

	return false;
}

void Agent::RenderOpenList()
{
	for (auto& voxel : mOpenList) {
		if (Scene::I->GetVoxelCondition(voxel) != VoxelCondition::Picked) {
			Scene::I->SetVoxelCondition(voxel, VoxelCondition::Opened);
		}
	}
}

void Agent::RenderCloseList()
{
	for (auto& voxel : mCloseList) {
		if (Scene::I->GetVoxelCondition(voxel) != VoxelCondition::Picked) {
			Scene::I->SetVoxelCondition(voxel, VoxelCondition::Closed);
		}
	}
}

Vec3 AgentManager::GetFlowFieldDirection(const Pos& index)
{
	if (mFlowFieldMap.count(index)) {
		return mFlowFieldMap[index] - Scene::I->GetVoxelPos(index);
	}

	return Vec3{};
}

void AgentManager::Start()
{
	mKdTree = std::make_shared<KdTree>();
}

void AgentManager::Update()
{
	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->SetPreferredVelocity();
	}

	mKdTree->BuildAgentTree();

	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->ComputeNeighbors();
		mAgents[i]->ComputeNewVelocity();
	}

	for (int i = 0; i < static_cast<int>(mAgents.size()); ++i) {
		mAgents[i]->UpdatePosition();
	}
}

void AgentManager::PathPlanningToFlowField(const Pos& dest)
{
	std::unordered_map<Pos, float>	distance;
	std::unordered_map<Pos, bool>	visited;

	std::priority_queue<std::pair<float, Pos>, std::vector<std::pair<float, Pos>>, std::greater<std::pair<float, Pos>>> pq;
	pq.push({ 0, dest });
	distance[dest] = 0;

	std::pair<int, Pos> curNode{};
	while (!pq.empty()) {
		curNode = pq.top();
		pq.pop();

		if (distance[curNode.second] < curNode.first) 
			continue;

		for (int dir = 0; dir < 8; ++dir) {
			Pos nextPosZX = curNode.second + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Pos nextPos = Pos{ it->first.first, it->first.second, it->second };
				int diffPosY = abs(nextPos.Y - curNode.second.Y);
				int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
				int nextCost = distance[curNode.second] + gkCost[dir] + proximityCost;

				if (diffPosY >= 10) continue;
				if (!distance.contains(nextPos)) distance[nextPos] = FLT_MAX;
				if (nextCost >= distance[nextPos]) continue;

				pq.push({ nextCost, nextPos });
				distance[nextPos] = nextCost;
				mFlowFieldMap[nextPos] = Scene::I->GetVoxelPos(curNode.second);
			}
		}
	}

	for (auto agent : mAgents) {
		agent->ClearPath();
		agent->SetPathDest(dest);
	}
}

void AgentManager::AllAgentPathPlanning(const Pos& dest)
{
	for (auto agent : mAgents) {
		agent->ReadyPlanningToPath(agent->GetVoxelIndex());
		agent->PathPlanningToAstar(dest, {}, true);
	}
}

void AgentManager::StartMoveToPath()
{
	for (Agent* agent : mAgents) {
		agent->SetStartMoveToPath(true);
	}
}

void AgentManager::RenderPathList()
{
	for (Agent* agent : mAgents) {
		agent->RenderOpenList();
	}

	for (Agent* agent : mAgents) {
		agent->RenderCloseList();
	}
}

void AgentManager::ClearPathList()
{
	for (Agent* agent : mAgents) {
		agent->ClearPathList();
	}
}

std::unordered_map<Pos, int> AgentManager::CheckAgentIndex(const Pos& index, Agent* invoker)
{
	std::unordered_map<Pos, int> costMap{};
	for (auto agent : mAgents) {
		if (agent == invoker) {
			continue;
		}
		if (agent->GetPathDirection() == invoker->GetPathDirection() && agent->IsStart()) {
			continue;
		}

		Agent* otherAgent{};
		const Pos& agentVoxelIndex = agent->GetVoxelIndex();
		const Pos& agentCrntPathIndex = agent->GetPathIndex(0);
		const Pos& agentNextPathIndex = agent->GetPathIndex(1);

		if (agentVoxelIndex == index) {
			otherAgent = agent;
		}
		else if (agentCrntPathIndex == index) {
			otherAgent = agent;
		}
		else if (agentNextPathIndex == index) {
			otherAgent = agent;
		}

		if (!otherAgent) {
			continue;
		}

		const Vec3& pos = Scene::I->GetVoxelPos(index);
		for (int z = 1; z >= -1; --z) {
			for (int x = -1; x <= 1; ++x) {
				int dz = index.Z + z;
				int dx = index.X + x;
				const Pos& neighborIndex = Pos{ dz, dx, 0 };
				const Vec3& neighborPos = Scene::I->GetVoxelPos(neighborIndex);
				const Vec3& dir = Vector3::Normalized(neighborPos.xz() - pos.xz());
				float angle = Vector3::Angle(dir, otherAgent->GetPathDirection());
				int cost = static_cast<int>(pow(1.f - angle / 180.f, 5) * 10);
				costMap[neighborIndex] = max(costMap[neighborIndex], cost);
			}
		}

		// speed clamp : 0.7 ~ 1.4
		float angle = Vector3::Angle(otherAgent->GetPathDirection(), invoker->GetPathDirection());
		float normAngle = max(0.f, (angle / 180.f) * 0.7f) + 0.7f;
		invoker->SetAngleSpeedRatio(normAngle);

		if (agent->GetVoxelIndex() == invoker->GetPathDest()) {
			invoker->ClearPathList();
			invoker->ClearPath();
			invoker->SetPathDest(FindEmptyDestVoxel(invoker));
			break;
		}
	}

	if (!costMap.empty()) {
		for (auto agent : mAgents) {
			if (agent == invoker) {
				continue;
			}

			costMap[agent->GetVoxelIndex()] = 1000000;
		}
	}

	return costMap;
}

void AgentManager::PickAgent(Agent** agent)
{
	Agent* pickedAgent{};
	for (Agent* mAgent : mAgents) {
		if (mAgent->PickAgent()) {
			pickedAgent = mAgent;
		}
		mAgent->SetRimFactor(0.f);
	}

	if (pickedAgent) {
		*agent = pickedAgent;
	}

	if (*agent) {
		(*agent)->SetRimFactor(1.f);
	}
}

Pos AgentManager::FindEmptyDestVoxel(Agent* invoker)
{
	std::queue<Pos> q;
	std::map<Pos, bool> visited;
	q.push(invoker->GetPathDest());

	Pos curPos{};
	while (!q.empty()) {
		curPos = q.front();
		q.pop();

		bool isFind = true;
		for (auto agent : mAgents) {
			if (agent == invoker) {
				continue;
			}
			
			if (curPos == agent->GetPathDest()) {
				isFind = false;
				break;
			}
		}

		if (isFind && curPos != invoker->GetPathDest() && curPos != invoker->GetVoxelIndex()) {
			break;
		}

		if (visited[curPos])
			continue;

		visited[curPos] = true;

		for (int dir = 0; dir < 4; ++dir) {
			Pos nextPosZX = curPos + gkFront[dir];
			PairMapRange range = Scene::I->GetCanWalkVoxels(nextPosZX);
			for (auto it = range.first; it != range.second; ++it) {
				Pos nextPos = Pos{ it->first.first, it->first.second, it->second };
				q.push(nextPos);
			}
		}
	}

	return curPos;
}

Pos AgentManager::RandomDest(int x, int z)
{
	const Vec3& cameraTargetPos = MAIN_CAMERA->GetTargetPosition();
	const Pos& cameraTargetIndex = Scene::I->GetVoxelIndex(cameraTargetPos);
	int randZ = Math::RandInt(-x, x);
	int randX = Math::RandInt(-z, z);
	const Vec3& randPos = Scene::I->GetVoxelPos(Pos{ randZ, randX, 0 });
	float y = Scene::I->GetTerrainHeight(randPos.x, randPos.z);
	const Pos& randIndex = Scene::I->GetVoxelIndex(Vec3{ randPos.x, y, randPos.z });
	return cameraTargetIndex.XZ() + randIndex;
}

void AgentManager::ShuffleMoveToPath()
{
	std::unordered_set<Pos> usedPos{};
	for (auto agent : mAgents) {
		Pos newPos = RandomDest(10, 0);
		while (usedPos.count(newPos)) {
			newPos = RandomDest(10, 0);
		}
		usedPos.insert(newPos);

		agent->ReadyPlanningToPath(newPos);
	}

	usedPos.clear();
	for (auto agent : mAgents) {
		Pos newPos = RandomDest(10, 0);
		while (usedPos.count(newPos)) {
			newPos = RandomDest(10, 0);
		}
		usedPos.insert(newPos);

		std::vector<Vec3> path = agent->PathPlanningToAstar(newPos, {});
		if (!path.empty()) {
			agent->SetPath(path);
		}
	}
}

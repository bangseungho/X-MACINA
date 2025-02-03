#include "EnginePch.h"
#include "Component/Agent.h"
#include "Component/Camera.h"
#include "Component/Collider.h"

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
	mObject->SetPosition(100, 0, 260);
}

void Agent::Update()
{
	MoveToPath();
}

std::vector<Vec3> Agent::PathPlanningToAstar(const Pos& dest, bool clearPathList)
{
	if (clearPathList) {
		ClearPathList();
	}

	std::vector<Vec3> finalPath{};

	mDest = dest;
	std::stack<Pos>	path{};
	std::unordered_map<Pos, Pos> parent;
	std::unordered_map<Pos, int> onVoxel;
	std::unordered_map<Pos, float> distance;
	std::unordered_map<Pos, bool> visited;

	std::function<float(const Pos&, const Pos&)> heuristic{};
	switch (PathOption::I->GetHeuristic())
	{
	case Heuristic::Manhattan:
		heuristic = HeuristicManhattan;
		break;
	case Heuristic::Euclidean:
		heuristic = HeuristicEuclidean;
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
	int openNodeCount{};
	PQNode curNode{};
	while (!pq.empty()) {
		curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (openNodeCount >= PathOption::I->GetMaxOpenNodeCount()) break;
		if (visited.contains(curNode.Pos)) continue;
		if (distance[curNode.Pos] < curNode.F) continue;
		if (mGlobalPathCache.count(curNode.Pos)) {
			while (mGlobalPath.size() > 1 && Scene::I->GetVoxelIndex(mGlobalPath.back()) != curNode.Pos) {
				mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
				mGlobalPath.pop_back();
			}
			break;
		}
		visited[curNode.Pos] = true;

		if (curNode.Pos == dest)
			break;

		// 26방향으로 탐색
		for (int dir = 0; dir < gkCubeDirCount; ++dir) {
			Pos nextPos = curNode.Pos + gkFront3D[dir];
			VoxelState nextVoxel = Scene::I->GetVoxelState(nextPos);

			// costs
			int diffPosY = nextPos.Y - curNode.Pos.Y;
			int onVoxelCount = GetOnVoxelCount(nextPos);
			int onVoxelCountDiffPosY = onVoxelCount + diffPosY;
			int dirPathCost{};
			int onVoxelCountCost = onVoxelCount * PathOption::I->GetOnVoxelCost();
			int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
			float edgeCost = GetEdgeCost(nextPos, gkFront3D[dir]) * PathOption::I->GetEdgeWeight();
			if (onVoxelCountDiffPosY < PathOption::I->GetAllowedHeight()) onVoxel[nextPos] = onVoxelCount;
			if ((nextVoxel == VoxelState::Static || nextVoxel == VoxelState::TerrainStatic) && onVoxelCountDiffPosY > PathOption::I->GetAllowedHeight()) continue;
			if (nextVoxel == VoxelState::None) continue;
			if (visited.contains(nextPos)) continue;
			if (!distance.contains(nextPos)) distance[nextPos] = FLT_MAX;
			if (prevDir != gkFront3D[dir]) dirPathCost = gkCost3D[dir];

			float g = curNode.G + gkCost3D[dir] + dirPathCost + onVoxelCountCost + proximityCost + edgeCost;
			float h = heuristic(nextPos, dest) * PathOption::I->GetHeuristicWeight();
			if (g + h < distance[nextPos]) {
				distance[nextPos] = g + h;
				pq.push({ g + h, g, nextPos });
				parent[nextPos] = curNode.Pos;
				openNodeCount++;
				mOpenList.push_back(nextPos);
			}
		}
	}

	//// 경로를 못 찾은 경우
	//if (curNode.Pos != dest) {
	//	ClearPathList();
	//	mCloseList.push_back(mStart);
	//	return finalPath;
	//}

	Pos pos = curNode.Pos;
	prevDir.Init();
	// 부모를 통해 경로 설정
	while (pos != parent[pos]) {
		Pos newPos = pos + Pos{ 0, 0, onVoxel[pos] };
		parent[newPos] = parent[pos];
		pos = newPos;
		Pos dir = parent[pos] - newPos;
		if (!PathOption::I->GetDirPathOptimize() || prevDir != dir) {
			path.push(pos);
		}
		pos = parent[pos];
		prevDir = dir;
	}

	if (path.top() != mStart) {
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

	if (PathOption::I->GetSplinePath()) {
		MakeSplinePath(finalPath);
	}

	std::reverse(finalPath.begin(), finalPath.end());

	return finalPath;
}

void Agent::ReadyPlanningToPath(const Pos& start)
{
	mStart = start;
	mCloseList.push_back(mStart);
	mObject->SetPosition(Scene::I->GetVoxelPos(start));
	ClearPath();
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
						else if (state == VoxelState::Static || state == VoxelState::TerrainStatic) {
							if (GetOnVoxelCount(voxel) >= PathOption::I->GetAllowedHeight() || prev.Y != y) {
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

void Agent::MoveToPath()
{
	if (mGlobalPath.empty()) {
		return;
	}
	
	Vec3 nextPos = (mGlobalPath.back() - mObject->GetPosition());
	mObject->RotateTargetAxisY(mGlobalPath.back(), 1000.f);
	mObject->Translate(XMVector3Normalize(nextPos), PathOption::I->GetAgentSpeed() * DeltaTime());

	const float kMinDistance = 0.1f;

	if (nextPos.Length() < kMinDistance) {
		const Vec3& crntPathPos = mGlobalPath.back();
		const Pos& crntPathIndex = Scene::I->GetVoxelIndex(crntPathPos);
		mGlobalPath.pop_back();
		mGlobalPathCache.erase(crntPathIndex);
		AvoidStaticVoxel(crntPathIndex);
	}
}

void Agent::AvoidStaticVoxel(const Pos& crntPathIndex)
{
	bool addPath{};
	for (int i = mkAvoidPathCount; i > 0; --i) {
		if (mGlobalPath.size() <= i) {
			continue;
		}

		Pos nextPathIndex = Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - i]);
		VoxelState nextPathUpVoxelState = Scene::I->GetVoxelState(nextPathIndex.Up());

		if (nextPathUpVoxelState == VoxelState::Static) {
			for (int j = 0; j < i; ++j) {
				mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
				mGlobalPath.pop_back();
			}

			mStart = crntPathIndex;
			mLocalPath = PathPlanningToAstar(mDest, false);
			std::copy(mLocalPath.begin(), mLocalPath.end(), std::back_inserter(mGlobalPath));
			return;
		}
	}
}

int Agent::GetOnVoxelCount(const Pos& pos)
{
	Pos next = pos;
	int cnt{};
	while (true) {
		next.Y += 1;
		if (Scene::I->GetVoxelState(next) == VoxelState::None) {
			break;
		}
		cnt++;
	}

	return cnt;
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

void Agent::ClearPath()
{
	mGlobalPath.clear();
	mLocalPath.clear();
	mGlobalPathCache.clear();
	mFirstCollisionVoxel.Init();
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

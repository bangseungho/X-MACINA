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
	mPathDir = mObject->GetLook();

	Matrix mtxWorld = mObject->GetWorldTransform();
	mtxWorld = mtxWorld.CreateScale(0.4f);
	SetWorldMatrix(mtxWorld);
	mObject->GetComponent<ObjectCollider>()->SetScale(0.4f);
	mObject->SetPosition(100, 0, 260);
}

// 부딪히면 거기만 잠깐 변경
void Agent::Update()
{
	if (!PathOption::I->GetStartFlag()) {
		mIsStart = true;
	}

	MoveToPath();

	Pos voxelIndex = Scene::I->GetVoxelIndex(mObject->GetPosition());
	if (mVoxelIndex != voxelIndex) {
		if (AgentManager::I->CheckOtherAgent(mVoxelIndex, this) == 0) {
			Scene::I->SetVoxelState(mVoxelIndex, VoxelState::Terrain);
			Scene::I->SetVoxelState(mVoxelIndex.Up(), VoxelState::None);
		}
		Scene::I->SetVoxelState(voxelIndex.Up(), VoxelState::DynamicAgent);
		mVoxelIndex = voxelIndex;
	}
}

const Pos Agent::GetPathIndex(int index) const
{
	if (!mGlobalPath.empty() && mGlobalPath.size() > index) {
		return Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - index - 1]);
	}

	return Pos{};
}

std::vector<Vec3> Agent::PathPlanningToAstar(const Pos& dest, std::unordered_map<Pos, int> avoidCostMap, bool clearPathList, bool inputDest, bool avoidAgents)
{
	if (clearPathList) {
		ClearPathList();
	}

	std::vector<Vec3> finalPath{};

	mDest = dest;
	std::stack<Pos>	path{};
	std::unordered_map<Pos, Pos>	parent;
	std::unordered_map<Pos, int>	onVoxel;
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

	std::unordered_set<Pos> otherIndices = AgentManager::I->GetOtherAgentIndices(this);
	while (!pq.empty()) {
		curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (openNodeCount >= PathOption::I->GetMaxOpenNodeCount()) {failedPlanningPath = true; break; }
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
				int dirPathCost{};
				int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
				float edgeCost = GetEdgeCost(nextPos, gkFront[dir]) * PathOption::I->GetEdgeWeight();
				if (diffPosY > mOption.AllowedHeight) continue;
				if (visited.contains(nextPos)) continue;
				if (avoidAgents && otherIndices.count(nextPos)) continue;
				if (!distance.contains(nextPos)) distance[nextPos] = FLT_MAX;
				if (prevDir != gkFront[dir]) dirPathCost = gkCost[dir];

				float g = curNode.G + gkCost[dir] + avoidCostMap[nextPos.XZ()] + dirPathCost + proximityCost + edgeCost;
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
	mObject->SetPosition(Scene::I->GetVoxelPos(start));
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



void Agent::MoveToPath()
{
	if (!mIsStart) {
		return;
	}

	if (mGlobalPath.empty()) {
		mIsStart = false;
		return;
	}
	
	Vec3 nextPos = (mGlobalPath.back() - mObject->GetPosition());
	mObject->RotateTargetAxisY(mGlobalPath.back(), 1000.f);
	mObject->Translate(XMVector3Normalize(nextPos), mOption.AgentSpeed* DeltaTime());

	const float kMinDistance = 0.05f;

	const Vec3& crntPathPos = mGlobalPath.back();
	const Pos& crntPathIndex = Scene::I->GetVoxelIndex(crntPathPos);
	if (nextPos.Length() < kMinDistance) {
		mGlobalPath.pop_back();
		mGlobalPathCache.erase(crntPathIndex);
		mSlowSpeedCount = max(mSlowSpeedCount - 1, 0);
		RePlanningToPathAvoidStatic(crntPathIndex);

		if (!mGlobalPath.empty()) {
			mPathDir = Vector3::Normalized(mGlobalPath.back() - mObject->GetPosition());
		}
	}
}

void Agent::RePlanningToPathAvoidStatic(const Pos& crntPathIndex)
{
	for (int i = mkAvoidForwardStaticObjectCount; i > 0; --i) {
		if (mGlobalPath.size() <= i) {
			continue;
		}

		Pos nextPathIndex = Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - i]);
		Pos backPathIndex = Scene::I->GetVoxelIndex(mGlobalPath[mGlobalPath.size() - 1]);
		VoxelState nextPathUpVoxelState = Scene::I->GetVoxelState(nextPathIndex.Up());
		if (nextPathUpVoxelState == VoxelState::Dynamic || nextPathUpVoxelState == VoxelState::Static || nextPathUpVoxelState == VoxelState::DynamicAgent) {
			for (int j = 0; j < i; ++j) {
				mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
				mGlobalPath.pop_back();
			}

			if (nextPathUpVoxelState == VoxelState::DynamicAgent) {
				Scene::I->RemoveCanWalkVoxel(backPathIndex);
			}

			mStart = crntPathIndex;
			mOption.Heuri = Heuristic::Euclidean;
			mLocalPath = PathPlanningToAstar(mDest, {}, false, false, true);
			std::copy(mLocalPath.begin(), mLocalPath.end(), std::back_inserter(mGlobalPath));

			if (nextPathUpVoxelState == VoxelState::DynamicAgent) {
				Scene::I->SetVoxelState(backPathIndex, VoxelState::Terrain);
			}
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
	std::unordered_map<Pos, int> costMap = AgentManager::I->CheckAgentIndex(nextPathIndex, this);
	if (costMap.empty()) {
		return;
	}

	mGlobalPathCache.erase(Scene::I->GetVoxelIndex(mGlobalPath.back()));
	mGlobalPath.pop_back();

	mOption.Heuri = Heuristic::Euclidean;
	mStart = mVoxelIndex;
	mLocalPath = PathPlanningToAstar(mDest, costMap, false, false);
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

void AgentManager::Update()
{
	bool finish{};
	for (Agent* agent : mAgents) {
		if (agent->IsStart()) {
			mFinishAllAgentMoveToPath = false;
			return;
		}
	}
	mFinishAllAgentMoveToPath = true;
}

int AgentManager::CheckOtherAgent(const Pos& index, Agent* invoker)
{
	int cnt{};
	for (Agent* agent : mAgents) {
		if (invoker == agent) {
			continue;
		}

		if (index == agent->GetVoxelIndex()) {
			cnt++;
		}
	}

	return cnt;
}

std::unordered_set<Pos> AgentManager::GetOtherAgentIndices(Agent* invoker)
{
	std::unordered_set<Pos> result{};
	for (Agent* agent : mAgents) {
		if (agent == invoker) {
			continue;
		}
		result.insert(agent->GetVoxelIndex());
	}
	return result;
}

void AgentManager::StartMoveToPath()
{
	for (Agent* agent : mAgents) {
		agent->SetStartMoveToPath(true);
	}
}

void AgentManager::RenderPathList()
{
	//for (Agent* agent : mAgents) {
	//	agent->RenderOpenList();
	//}

	VoxelManager::I->GetPickedAgent()->RenderOpenList();
	VoxelManager::I->GetPickedAgent()->RenderCloseList();

	//for (Agent* agent : mAgents) {
	//	agent->RenderCloseList();
	//}
}

void AgentManager::ClearPathList()
{
	for (Agent* agent : mAgents) {
		agent->ClearPathList();
	}
}

std::unordered_map<Pos, int> AgentManager::CheckAgentIndex(const Pos& index, Agent* invoker)
{
	std::vector<Pos> collisionPos{};
	std::unordered_map<Pos, int> costMap{};
	for (auto agent : mAgents) {
		if (agent == invoker) {
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
		costMap[index] = 10000000;
		break;
	}

	//for (auto agent : mAgents) {
	//	if (agent == invoker) {
	//		continue;
	//	}

	//	costMap[agent->GetVoxelIndex()] = 10000000;
	//}

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
		Pos newPos = RandomDest(5, 5);
		//while (usedPos.count(newPos)) {
		//	newPos = RandomDest(10, 0);
		//}
		//usedPos.insert(newPos);

		agent->ReadyPlanningToPath(newPos);
	}

	usedPos.clear();
	for (auto agent : mAgents) {
		Pos newPos = RandomDest(5, 5);
		//while (usedPos.count(newPos)) {
		//	newPos = RandomDest(10, 0);
		//}
		//usedPos.insert(newPos);

		std::vector<Vec3> path = agent->PathPlanningToAstar(newPos, {});
		if (!path.empty()) {
			agent->SetPath(path);
		}
	}
}

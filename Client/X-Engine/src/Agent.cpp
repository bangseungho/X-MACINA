#include "EnginePch.h"
#include "Component/Agent.h"

#include "Scene.h"
#include "Timer.h"
#include "VoxelManager.h"

namespace {
	int HeuristicManhattan(const Pos& start, const Pos& dest) {
		return std::abs(dest.X - start.X) +
			std::abs(dest.Y - start.Y) +
			std::abs(dest.Z - start.Z);
	}
	int HeuristicEuclidean(const Pos& start, const Pos& dest) {
		return static_cast<int>(std::sqrt(std::pow(dest.X - start.X, 2) +
			std::pow(dest.Y - start.Y, 2) +
			std::pow(dest.Z - start.Z, 2)));
	}
}

void Agent::Update()
{
	MoveToPath();
}

bool Agent::PathPlanningToAstar(const Pos& dest)
{
	VoxelManager::I->ClearPathList();

	std::unordered_map<Pos, Pos> parent;
	std::unordered_map<Pos, int> onVoxel;
	std::unordered_map<Pos, int> distance;
	std::unordered_map<Pos, bool> visited;

	std::function<int(const Pos&, const Pos&)> heuristic{};
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
	int g = 0;
	int h = heuristic(mStart, dest) * PathOption::I->GetHeuristicWeight();
	pq.push({ g + h, g, mStart });
	distance[mStart] = g + h;
	parent[mStart] = mStart;

	Pos prevDir;
	int closedListSize{};
	PQNode curNode{};
	while (!pq.empty()) {
		curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (closedListSize >= PathOption::I->GetMaxClosedListSize()) break;
		if (visited.contains(curNode.Pos)) continue;
		if (distance[curNode.Pos] < curNode.F) continue;

		visited[curNode.Pos] = true;

		// 해당 지점이 목적지인 경우 종료
		if (curNode.Pos == dest) 
			break;

		// 26방향으로 탐색
		for (int dir = 0; dir < 26; ++dir) {
			Pos nextPos = curNode.Pos + gkFront3D[dir];
			VoxelState nextTile = Scene::I->GetVoxelState(nextPos);

			// 다음 방향 노드의 상태가 static이라면 continue
			int onVoxelCount = GetOnVoxelCount(nextPos);
			int pathSmoothingCost{}; 
			int onVoxelCountCost = onVoxelCount * PathOption::I->GetOnVoxelCost();
			int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
			if (onVoxelCount > 1 && onVoxelCount <= PathOption::I->GetAllowedHeight()) onVoxel[nextPos] = onVoxelCount;
			if ((nextTile == VoxelState::Static || nextTile == VoxelState::TerrainStatic) && onVoxelCount >= PathOption::I->GetAllowedHeight()) continue;
			if (nextTile == VoxelState::None) continue;
			if (visited.contains(nextPos)) continue;
			if (!distance.contains(nextPos)) distance[nextPos] = INT32_MAX;
			if (prevDir != gkFront3D[dir]) pathSmoothingCost = gkCost3D[dir] / 2;

			int g = curNode.G + gkCost3D[dir] + pathSmoothingCost + onVoxelCountCost + proximityCost;
			int h = heuristic(nextPos, dest) * PathOption::I->GetHeuristicWeight();
			if (g + h < distance[nextPos]) {
				distance[nextPos] = g + h;
				pq.push({ g + h, g, nextPos });
				parent[nextPos] = curNode.Pos;
				closedListSize++;
				VoxelManager::I->PushOpenedVoxel(nextPos);
			}
		}
	}
	
	// 경로를 못 찾은 경우
	if (curNode.Pos != dest) {
		VoxelManager::I->ClearPathList();
		ClearPath();
		VoxelManager::I->PushClosedVoxel(mStart);
		return false;
	}

	Pos pos = dest;
	// 부모 경로를 따라가 스택에 넣어준다. top이 first path이다.
	while (pos != parent[pos]) {
		Pos newPos = pos + Pos{ 0, 0, onVoxel[pos]};
		parent[newPos] = parent[pos];
		pos = newPos;

		mPath.push(Scene::I->GetVoxelPos(pos));
		VoxelManager::I->PushClosedVoxel(pos);

		pos = parent[pos];
	}

	return true;
}

void Agent::ReadyPlanningToPath(const Pos& start)
{
	mStart = start;
	VoxelManager::I->PushClosedVoxel(start);
	mObject->SetPosition(Scene::I->GetVoxelPos(start));
	ClearPath();
}

void Agent::MoveToPath()
{
	if (mPath.empty()) {
		return;
	}

	Vec3 nextPos = (mPath.top() - mObject->GetPosition());
	nextPos.y += Grid::mkTileHeight;

	mObject->RotateTargetAxisY(mPath.top(), 1000.f);
	mObject->Translate(XMVector3Normalize(nextPos), PathOption::I->GetAgentSpeed() * DeltaTime());

	const float kMinDistance = 0.1f;
	if (nextPos.Length() < kMinDistance) {
		mPath.pop();
		if (mPath.empty()) {
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

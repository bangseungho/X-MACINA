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
	std::stack<Pos>	path{};

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
	int openNodeCount{};
	PQNode curNode{};
	while (!pq.empty()) {
		curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (openNodeCount >= PathOption::I->GetMaxOpenNodeCount()) break;
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
			int dirPathCost{};
			int onVoxelCountCost = onVoxelCount * PathOption::I->GetOnVoxelCost();
			int proximityCost = Scene::I->GetProximityCost(nextPos) * PathOption::I->GetProximityWeight();
			int edgeCost = Scene::I->GetEdgeCost(nextPos);
			if (onVoxelCount < PathOption::I->GetAllowedHeight()) onVoxel[nextPos] = onVoxelCount;
			if ((nextTile == VoxelState::Static || nextTile == VoxelState::TerrainStatic) && onVoxelCount >= PathOption::I->GetAllowedHeight()) continue;
			if (nextTile == VoxelState::None) continue;
			if (visited.contains(nextPos)) continue;
			if (!distance.contains(nextPos)) distance[nextPos] = INT32_MAX;
			if (prevDir != gkFront3D[dir]) dirPathCost = gkCost3D[dir] / 2;

			int g = curNode.G + gkCost3D[dir] + dirPathCost + onVoxelCountCost + edgeCost + proximityCost ;
			int h = heuristic(nextPos, dest) * PathOption::I->GetHeuristicWeight();
			if (g + h < distance[nextPos]) {
				distance[nextPos] = g + h;
				pq.push({ g + h, g, nextPos });
				parent[nextPos] = curNode.Pos;
				openNodeCount++;
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
	prevDir = Pos{};
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
		VoxelManager::I->PushClosedVoxel(now);
		mFinalPath.push_back(Scene::I->GetVoxelPos(now));
		path.pop();
	}

	if (PathOption::I->GetSplinePath()) {
		MakeSplinePath();
	}

	std::reverse(mFinalPath.begin(), mFinalPath.end());

	return true;
}

void Agent::ReadyPlanningToPath(const Pos& start)
{
	mStart = start;
	VoxelManager::I->PushClosedVoxel(start);
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
					if (ray.Intersects(BoundingBox{ Scene::I->GetVoxelPos(voxel), Grid::mkTileExtent }, dist)) {
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

void Agent::MakeSplinePath()
{
	if (mFinalPath.size() <= 2) {
		return;
	}

	std::vector<Vec3> splinePath{};
	const int pathSize = static_cast<int>(mFinalPath.size());
	const int sampleCount = 10;

	Vec3 p4 = mFinalPath.back() + mFinalPath[pathSize - 1].xz() - mFinalPath[pathSize - 2].xz();
	mFinalPath.push_back(p4);

	for (int i = 0; i < pathSize - 2; ++i) {
		Vec3 p0 = mFinalPath[i];
		Vec3 p1 = mFinalPath[i + 1];
		Vec3 p2 = mFinalPath[i + 2];
		Vec3 p3 = mFinalPath[i + 3];

		for (int j = 0; j <= sampleCount; ++j) {
			float t = static_cast<float>(j) / sampleCount;
			splinePath.push_back(Vec3::CatmullRom(p0, p1, p2, p3, t));
		}
	}
	mFinalPath = splinePath;
}

void Agent::MoveToPath()
{
	if (mFinalPath.empty()) {
		return;
	}

	if (mFinalPath.back() == mObject->GetPosition()) {
		mFinalPath.pop_back();
		return;
	}
	
	Vec3 nextPos = (mFinalPath.back() - mObject->GetPosition());

	mObject->RotateTargetAxisY(mFinalPath.back(), 1000.f);
	mObject->Translate(XMVector3Normalize(nextPos), PathOption::I->GetAgentSpeed() * DeltaTime());

	const float kMinDistance = 0.1f;
	if (nextPos.Length() < kMinDistance) {
		mFinalPath.pop_back();
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

void Agent::ClearPath()
{
	mFinalPath.clear();
}

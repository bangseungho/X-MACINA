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
	// ��Ŭ���� �Ÿ� ��� �޸���ƽ �Լ�
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

void Agent::PathPlanningToAstar(Pos dest)
{
	while (!mPath.empty()) mPath.pop();
	VoxelManager::I->ClearClosedList();

	std::map<Pos, Pos>	parent;
	std::map<Pos, int>	onVoxel;
	std::map<Pos, int>	distance;
	std::map<Pos, bool>	visited;

	// f = g + h
	Pos start = Scene::I->GetTileUniqueIndexFromPos(mObject->GetPosition());
	std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
	int g = 0;
	int h = HeuristicManhattan(start, dest) * mkWeight;
	pq.push({ g + h, g, start });
	distance[start] = g + h;
	parent[start] = start;

	Pos prevDir;
	int closedListSize{};
	while (!pq.empty()) {
		PQNode curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		if (closedListSize >= PathOption::I->GetMaxClosedListSize()) return;
		if (visited.contains(curNode.Pos)) continue;
		if (distance[curNode.Pos] < curNode.F) continue;

		visited[curNode.Pos] = true;
		Scene::I->GetClosedList().push_back(Scene::I->GetTilePosFromUniqueIndex(curNode.Pos));

		// �ش� ������ �������� ��� ����
		if (curNode.Pos == dest) break;

		// 26�������� Ž��
		for (int dir = 0; dir < 26; ++dir) {
			Pos nextPos = curNode.Pos + gkFront3D[dir];
			VoxelState nextTile = Scene::I->GetTileFromUniqueIndex(nextPos);

			// ���� ���� ����� ���°� static�̶�� continue
			int onVoxelCount = GetOnVoxelCount(nextPos);
			int pathSmoothingCost{}; 
			int onVoxelCountCost = onVoxelCount * 10;
			if (onVoxelCount > 1 && onVoxelCount <= PathOption::I->GetAllowedHeight()) onVoxel[nextPos] = onVoxelCount;
			if (nextTile == VoxelState::Static && onVoxelCount >= PathOption::I->GetAllowedHeight()) continue;
			if (nextTile == VoxelState::None) continue;
			if (visited.contains(nextPos)) continue;
			if (!distance.contains(nextPos)) distance[nextPos] = INT32_MAX;
			if (prevDir != gkFront3D[dir]) pathSmoothingCost = gkCost3D[dir] / 2;

			int g = curNode.G + gkCost3D[dir] + pathSmoothingCost + onVoxelCountCost;
			int h = HeuristicManhattan(nextPos, dest) * mkWeight;
			if (g + h <= distance[nextPos]) {
				distance[nextPos] = g + h;
				pq.push({ g + h, g, nextPos });
				parent[nextPos] = curNode.Pos;
				closedListSize++;
			}
		}
	}

	Pos pos = dest;
	// �θ� ��θ� ���� ���ÿ� �־��ش�. top�� first path�̴�.
	while (pos != parent[pos]) {
		Pos newPos = pos + Pos{ 0, 0, onVoxel[pos]};
		parent[newPos] = parent[pos];
		pos = newPos;

		mPath.push(Scene::I->GetTilePosFromUniqueIndex(pos));
		VoxelManager::I->PushClosedVoxel(pos);

		pos = parent[pos];
	}
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
		if (Scene::I->GetTileFromUniqueIndex(next) == VoxelState::None) {
			break;
		}
		cnt++;
	}
	return cnt;
}

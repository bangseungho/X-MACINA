#include "EnginePch.h"
#include "Component/Agent.h"

#include "Scene.h"
#include "Timer.h"

namespace {
	int HeuristicManhattan(const Pos& start, const Pos& dest) {
		return std::abs(dest.X - start.X) +
			std::abs(dest.Y - start.Y) +
			std::abs(dest.Z - start.Z);
	}
	// 유클리드 거리 기반 휴리스틱 함수
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
	while (!mPath.empty()) {
		mPath.pop();
	}

	Pos start = Scene::I->GetTileUniqueIndexFromPos(mObject->GetPosition());

	Scene::I->ClearPathList();
	if (Scene::I->GetTileFromUniqueIndex(dest) == Tile::Static) {
		return;
	}

	std::map<Pos, Pos>	parent;
	std::map<Pos, int>	onVoxel;
	std::map<Pos, int>	distance;
	std::map<Pos, bool>	visited;

	// f = g + h
	std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
	int g = 0;
	int h = HeuristicManhattan(start, dest) * mkWeight;
	pq.push({ g + h, g, start });
	distance[start] = g + h;
	parent[start] = start;

	Pos prevDir;
	while (!pq.empty()) {
		PQNode curNode = pq.top();
		prevDir = curNode.Pos - parent[curNode.Pos];
		pq.pop();

		// 방문하지 않은 노드들만 방문
		if (visited.contains(curNode.Pos))
			continue;

		visited[curNode.Pos] = true;
		Scene::I->GetClosedList().push_back(Scene::I->GetTilePosFromUniqueIndex(curNode.Pos));

		// 해당 지점이 목적지인 경우 종료
		if (curNode.Pos == dest)
			break;

		// 8방향으로 탐색
		for (int dir = 0; dir < 26; ++dir) {
			Pos nextPos = curNode.Pos + gkFront3D[dir];
			Tile nextTile = Scene::I->GetTileFromUniqueIndex(nextPos);

			// 다음 방향 노드의 상태가 static이라면 continue
			int onVoxelCount = GetOnVoxelCount(nextPos);
			int onAddCost = onVoxelCount * 10;
			if (onVoxelCount > 1 && onVoxelCount <= PathOption::I->GetAllowedHeight()) {
				onVoxel[nextPos] = onVoxelCount;
			}

			if (nextTile == Tile::Static && onVoxelCount >= PathOption::I->GetAllowedHeight()) continue;
			if (nextTile == Tile::None) continue;
			if (visited.contains(nextPos)) continue;
			if (!distance.contains(nextPos)) distance[nextPos] = INT32_MAX;

			// 비용 계산 보통의 1 / 2
			int addCost{};
			if (prevDir != gkFront3D[dir]) {
				addCost = gkCost3D[dir] / 2;
			}

			int g = curNode.G + gkCost3D[dir] + addCost + onAddCost;
			int h = HeuristicManhattan(nextPos, dest) * mkWeight;
			if (g + h < distance[nextPos]) {
				distance[nextPos] = g + h;
				pq.push({ g + h, g, nextPos });
				parent[nextPos] = curNode.Pos;
			}
		}
	}

	Pos pos = dest;
	//prevDir = { 0, 0 };

	// 부모 경로를 따라가 스택에 넣어준다. top이 first path이다.
	while (true) {
		Pos dir = parent[pos] - pos;
		//if (prevDir != dir) {
			pos.Y += onVoxel[pos];
			mPath.push(Scene::I->GetTilePosFromUniqueIndex(pos));
			Scene::I->GetOpenList().push_back(mPath.top());
		//}

		if (pos == parent[pos])
			break;

		pos = parent[pos];
		//prevDir = dir;
	}

	// 자연스러운 움직임을 위해 첫 번째 경로는 삭제
	//Scene::I->GetOpenList().push_back(Scene::I->GetTilePosFromUniqueIndex(start));
	if (!mPath.empty()) {
		mPath.pop();
	}
}

void Agent::MoveToPath()
{
	if (mPath.empty()) {
		return;
	}

	Vec3 nextPos = (mPath.top() - mObject->GetPosition());
	//nextPos.y += Grid::mkTileHeight;

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
	int cnt{};

	Pos next = pos;
	while (true) {
		next.Y += 1;

		if (Scene::I->GetTileFromUniqueIndex(next) == Tile::None) {
			break;
		}

		cnt++;
	}

	return cnt;
}

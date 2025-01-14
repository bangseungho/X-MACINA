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
	while (!mPath.empty()) {
		mPath.pop();
	}

	Pos start = Scene::I->GetTileUniqueIndexFromPos(mObject->GetPosition());

	Scene::I->ClearPathList();

	std::map<Pos, Pos>	mParent;
	std::map<Pos, int>	mDistance;
	std::map<Pos, bool>	mVisited;

	// f = g + h
	std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
	int g = 0;
	int h = HeuristicManhattan(start, dest) * mkWeight;
	pq.push({ g + h, g, start });
	mDistance[start] = g + h;
	mParent[start] = start;

	Pos prevDir;
	while (!pq.empty()) {
		PQNode curNode = pq.top();
		prevDir = curNode.Pos - mParent[curNode.Pos];
		pq.pop();

		// �湮���� ���� ���鸸 �湮
		if (mVisited.contains(curNode.Pos))
			continue;

		mVisited[curNode.Pos] = true;
		Scene::I->GetClosedList().push_back(Scene::I->GetTilePosFromUniqueIndex(curNode.Pos));

		// �ش� ������ �������� ��� ����
		if (curNode.Pos == dest)
			break;

		// 8�������� Ž��
		for (int dir = 0; dir < 26; ++dir) {
			Pos nextPos = curNode.Pos + gkFront3D[dir];

			// ���� ���� ����� ���°� static�̶�� continue
			if (Scene::I->GetTileFromUniqueIndex(nextPos) == Tile::Static)
				continue;

			if (Scene::I->GetTileFromUniqueIndex(nextPos) == Tile::None)
				continue;

			// �̹� �湮�� ���̸� continue
			if (mVisited.contains(nextPos))
				continue;

			// ���� �Ÿ� ������ ���ٸ� �Ÿ� ����� �ִ����� ����
			if (!mDistance.contains(nextPos))
				mDistance[nextPos] = INT32_MAX;

			// ��� ��� ������ 1 / 2
			int addCost{};
			if (prevDir != gkFront3D[dir])
				addCost = gkCost3D[dir] / 2;

			int g = curNode.G + gkCost3D[dir] + addCost;
			int h = HeuristicManhattan(nextPos, dest) * mkWeight;
			if (mDistance[nextPos] <= g + h)
				continue;

			mDistance[nextPos] = g + h;
			pq.push({ g + h, g, nextPos });
			mParent[nextPos] = curNode.Pos;
		}
	}

	Pos pos = dest;
	prevDir = { 0, 0 };

	// �θ� ��θ� ���� ���ÿ� �־��ش�. top�� first path�̴�.
	while (true) {
		Pos dir = mParent[pos] - pos;

		if (prevDir != dir) {
			mPath.push(Scene::I->GetTilePosFromUniqueIndex(pos));
			Scene::I->GetOpenList().push_back(mPath.top());
		}

		if (pos == mParent[pos])
			break;

		pos = mParent[pos];
		prevDir = dir;
	}

	// �ڿ������� �������� ���� ù ��° ��δ� ����
	Scene::I->GetOpenList().push_back(Scene::I->GetTilePosFromUniqueIndex(start));
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
	nextPos.y += Grid::mkTileHeight;

	mObject->RotateTargetAxisY(mPath.top(), 1000.f);
	mObject->Translate(XMVector3Normalize(nextPos), mWalkSpeed * DeltaTime());

	const float kMinDistance = 0.1f;
	if (nextPos.Length() < kMinDistance) {
		mPath.pop();
		if (mPath.empty()) {
		}
	}
}

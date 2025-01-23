#include "EnginePch.h"
#include "Grid.h"

#include "Scene.h"
#include "Object.h"
#include "Component/Collider.h"
#include "MeshRenderer.h"

#include "DXGIMgr.h"
#include "FrameResource.h"
#include "Component/Camera.h"
#include "Component/Agent.h"
#include "InputMgr.h"

int Grid::mTileRows = 0;
int Grid::mTileCols = 0;

Grid::Grid(int index, int width, const BoundingBox& bb)
	:
	mIndex(index),
	mBB(bb)
{
	mTileRows = static_cast<int>(width / mkTileWidth);
	mTileCols = static_cast<int>(width / mkTileWidth);
}

VoxelState Grid::GetVoxelState(const Pos& index)
{
	auto findIt = mVoxels.find(index);
	if (findIt == mVoxels.end()) {
		return VoxelState{};
	}
	else if (findIt->second.State == VoxelState::None) {
		return VoxelState{};
	}

	return mVoxels[index].State;
}

VoxelCondition Grid::GetVoxelCondition(const Pos& index)
{
	auto findIt = mVoxels.find(index);
	if (findIt == mVoxels.end()) {
		return VoxelCondition{};
	}
	else if (findIt->second.Condition == VoxelCondition::None) {
		return VoxelCondition{};
	}

	return mVoxels[index].Condition;
}

Voxel Grid::GetVoxel(const Pos& index)
{
	if (mVoxels.count(index)) {
		return mVoxels[index];
	}
	else {
		return Voxel{};
	}
}

int Grid::GetProximityCost(const Pos& index)
{
	if (mProximityCosts.count(index)) {
		return mProximityCosts[index];
	}
	else {
		return 0;
	}
}

float Grid::GetEdgeCost(const Pos& index, bool isRowEdge)
{
	if (isRowEdge) {
		if (mRowEdgeCosts.count(index)) {
			return mRowEdgeCosts[index];
		}
	}
	else {
		if (mColEdgeCosts.count(index)) {
			return mColEdgeCosts[index];
		}
	}

	return 0;
}

void Grid::SetVoxelState(const Pos& index, VoxelState state)
{
	if (mVoxels.count(index)) {
		if (state == VoxelState::None) {
			mVoxels.erase(index);
		}
		else {
			mVoxels[index].State = state;
		}
	}
	else {
		Voxel voxel;
		voxel.State = state;
		const Matrix scaleMtx = Matrix::CreateScale(mkTileWidth, mkTileHeight, mkTileWidth);
		const Matrix translationMtx = Matrix::CreateTranslation(Scene::I->GetVoxelPos(index));
		const Matrix matrix = scaleMtx * translationMtx;
		voxel.MtxWorld = matrix.Transpose();
		mVoxels.insert({ index, voxel });
	}
}

void Grid::SetVoxelCondition(const Pos& index, VoxelCondition condition)
{
	if (mVoxels.count(index)) {
		mVoxels[index].Condition = condition;
	}
	else {
		Voxel voxel;
		voxel.Condition = condition;
		const Matrix scaleMtx = Matrix::CreateScale(mkTileWidth, mkTileHeight, mkTileWidth);
		const Matrix translationMtx = Matrix::CreateTranslation(Scene::I->GetVoxelPos(index));
		const Matrix matrix = scaleMtx * translationMtx;
		voxel.MtxWorld = matrix.Transpose();
		mVoxels.insert({ index, voxel });
	}
}

void Grid::SetProximityCost(const Pos& index, int cost)
{
	mProximityCosts[index] = max(cost, mProximityCosts[index]);
}

void Grid::AddObject(GridObject* object)
{
	if (mObjects.count(object)) {
		return;
	}

	mObjects.insert(object);

	switch (object->GetType()) {
	case ObjectType::DynamicMove:
		mDynamicObjets.insert(object);
		break;
	case ObjectType::Env:
		mEnvObjects.insert(object);
		break;
	default:
		mStaticObjects.insert(object);
		UpdateVoxels(VoxelState::Static, object);
		break;
	}
}

inline bool IsNotBuilding(ObjectTag tag)
{
	return tag != ObjectTag::Building && tag != ObjectTag::DissolveBuilding;
}

void Grid::UpdateVoxels(VoxelState tile, GridObject* object)
{
	// 정적 오브젝트가 Building 태그인 경우에만 벽으로 설정
	if (IsNotBuilding(object->GetTag()))
		return;

	// 오브젝트의 충돌 박스
	for (const auto& collider : object->GetComponent<ObjectCollider>()->GetColliders()) {
		std::unordered_set<Pos> boundingVoxels;

		std::queue<Pos> q;
		std::unordered_map<Pos, bool> visited(2000);

		if (collider->GetType() != Collider::Type::Box) {
			continue;
		}

		// 오브젝트의 타일 기준 인덱스 계산
		Vec3 pos = collider->GetCenter();
		Pos index = Scene::I->GetVoxelIndex(pos);
		Scene::I->SetVoxelState(index, tile);
		q.push(index);

		// q가 빌 때까지 BFS를 돌며 현재 타일이 오브젝트와 충돌 했다면 해당 타일을 업데이트
		while (!q.empty()) {
			Pos curNode = q.front();
			q.pop();

			for (int dir = 0; dir < 6; ++dir) {
				Pos nextPosT = curNode + gkFront2[dir];

				if (visited[nextPosT]) {
					continue;
				}

				Vec3 nextPosW = Scene::I->GetVoxelPos(nextPosT);
				BoundingBox bb{ nextPosW, mkTileExtent };

				visited[nextPosT] = true;

				if (collider->Intersects(bb)) {
					boundingVoxels.insert(nextPosT);
					Scene::I->SetVoxelState(nextPosT, tile);
					q.push(nextPosT);
				}
			}
		}
		UpdateVoxelsEdgeCost(boundingVoxels);
	}
}

void Grid::UpdateVoxelsEdgeCost(const std::unordered_set<Pos>& boundingVoxels)
{
	float maxRowValue{};
	float maxColValue{};
	for (const Pos& voxel : boundingVoxels) {
		bool isOverlapped = false;

		if (mRowEdgeCosts.count(voxel) && mColEdgeCosts.count(voxel)) {
			isOverlapped = true;
		}

		maxRowValue = max(maxRowValue, CalcRowEdgeCost(voxel, boundingVoxels));
		maxColValue = max(maxColValue, CalcColEdgeCost(voxel, boundingVoxels));

		if (isOverlapped) {
			mRowEdgeCosts[voxel] *= 0.5f;
			mRowEdgeCosts[voxel] *= 0.5f;
		}
	}

	for (const Pos& voxel : boundingVoxels) {
		if (maxRowValue != 0) {
			mRowEdgeCosts[voxel] /= maxRowValue;
		}
		if (maxColValue != 0) {
			mColEdgeCosts[voxel] /= maxColValue;
		}
		mRowEdgeCosts[voxel] *= 10;
		mColEdgeCosts[voxel] *= 10;
	}
}

float Grid::CalcRowEdgeCost(const Pos& voxel, const std::unordered_set<Pos>& boundingVoxels)
{
	int leftMoveCnt{};
	Pos nextLeft = voxel.Left();
	while (boundingVoxels.count(nextLeft)) {
		nextLeft = nextLeft.Left();
		leftMoveCnt++;
	}

	int rightMoveCnt{};
	Pos nextRight = voxel.Right();
	while (boundingVoxels.count(nextRight)) {
		nextRight = nextRight.Right();
		rightMoveCnt++;
	}

	mRowEdgeCosts[voxel] = static_cast<float>(abs(leftMoveCnt - rightMoveCnt));
	return mRowEdgeCosts[voxel];
}

float Grid::CalcColEdgeCost(const Pos& voxel, const std::unordered_set<Pos>& boundingVoxels)
{
	int forwardMoveCnt{};
	Pos nextForward = voxel.Forward();
	while (boundingVoxels.count(nextForward)) {
		nextForward = nextForward.Forward();
		forwardMoveCnt++;
	}

	int backwardMoveCnt{};
	Pos nextBackward = voxel.Backward();
	while (boundingVoxels.count(nextBackward)) {
		nextBackward = nextBackward.Backward();
		backwardMoveCnt++;
	}

	mColEdgeCosts[voxel] = static_cast<float>(abs(forwardMoveCnt - backwardMoveCnt));
	return mColEdgeCosts[voxel];
}

void Grid::RemoveObject(GridObject* object)
{
	if (!mObjects.count(object)) {
		return;
	}

	mObjects.erase(object);

	switch (object->GetType()) {
	case ObjectType::DynamicMove:
		mDynamicObjets.erase(object);
		break;
	case ObjectType::Env:
		mEnvObjects.erase(object);
		break;
	default:
		mStaticObjects.erase(object);
		UpdateVoxels(VoxelState::Static, object);
		break;
	}
}

bool Grid::Intersects(GridObject* object)
{
	const auto& objCollider = object->GetCollider();
	if (!objCollider) {
		return true;
	}
	return mBB.Intersects(objCollider->GetBS());
}

void Grid::CheckCollisions()
{
	if (!mDynamicObjets.empty()) {
		CheckCollisionObjects(mDynamicObjets);						// 동적 객체간 충돌 검사를 수행한다.
		if (!mStaticObjects.empty()) {
			CheckCollisionObjects(mDynamicObjets, mStaticObjects);	// 동적<->정적 객체간 충돌 검사를 수행한다.
		}
	}
}

float Grid::CheckCollisionsRay(const Ray& ray) const
{
	float minDist = 999.f;
	for (const auto& object : mStaticObjects) {
		// 정적 오브젝트가 Building 태그인 경우에만 벽으로 설정
		if (IsNotBuilding(object->GetTag()))
			continue;

		float dist = 100;
		for (const auto& collider : object->GetCollider()->GetColliders()) {
			if (collider->Intersects(ray, dist)) {
				minDist = min(minDist, dist);
			}
		}
	}

	return minDist;
}

void Grid::CheckCollisions(rsptr<Collider> collider, std::vector<GridObject*>& out, CollisionType type) const
{
	if (type & CollisionType::Dynamic) {
		for (const auto& object : mDynamicObjets) {
			if (object->GetCollider()->Intersects(collider)) {
				out.push_back(object);
			}
		}
	}
	if (type & CollisionType::Static) {
		for (const auto& object : mStaticObjects) {
			if (object->GetCollider()->Intersects(collider)) {
				out.push_back(object);
			}
		}
	}
}

// check collision for each object in objects
void Grid::CheckCollisionObjects(std::unordered_set<GridObject*> objects)
{
	for (auto a = objects.begin(); a != std::prev(objects.end()); ++a) {
		GridObject* objectA = *a;
		if (!objectA->IsActive()) {
			continue;
		}

		for (auto b = std::next(a); b != objects.end(); ++b) {
			GridObject* objectB = *b;
			if (!objectB->IsActive()) {
				continue;
			}

			Grid::ProcessCollision(objectA, objectB);
		}
	}
}

// check collision for (each objectsA) <-> (each objectsB)
void Grid::CheckCollisionObjects(std::unordered_set<GridObject*> objectsA, std::unordered_set<GridObject*> objectsB)
{
	for (auto a = objectsA.begin(); a != objectsA.end(); ++a) {
		GridObject* objectA = *a;

		for (auto b = objectsB.begin(); b != objectsB.end(); ++b) {
			GridObject* objectB = *b;

			Grid::ProcessCollision(objectA, objectB);
		}
	}
}

// call collision function if collide
void Grid::ProcessCollision(GridObject* objectA, GridObject* objectB)
{
	if (ObjectCollider::Intersects(*objectA, *objectB)) {
		objectA->OnCollisionStay(*objectB);
		objectB->OnCollisionStay(*objectA);
	}
}
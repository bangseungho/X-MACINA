#include "EnginePch.h"
#include "Grid.h"

#include "Scene.h"
#include "Object.h"
#include "Component/Collider.h"
#include "MeshRenderer.h"

int Grid::mTileRows = 0;
int Grid::mTileCols = 0;


Grid::Grid(int index, int width, const BoundingBox& bb)
	:
	mIndex(index),
	mBB(bb)
{
	mTileRows = static_cast<int>(width / mkTileHeight);
	mTileCols = static_cast<int>(width / mkTileWidth);

	mTiles = std::vector<std::vector<std::vector<Tile>>>(mTileHeightCount, std::vector<std::vector<Tile>>(mTileCols, std::vector<Tile>(mTileRows, Tile::None)));
}

Tile Grid::GetTileFromUniqueIndex(const Pos& tPos) const
{
	if (tPos.Y >= mTileHeightCount || tPos.Y <= 0.f) {
		return Tile::None;
	}

	return mTiles[tPos.Y][tPos.Z][tPos.X];
}

void Grid::SetTileFromUniqueIndex(const Pos& tPos, const Pos& index, Tile tile)
{
	if (tPos.Y >= mTileHeightCount || tPos.Y < 0.f) {
		return;
	}

	mVoxelList.insert(Scene::I->GetTilePosFromUniqueIndex(index));
	mTiles[tPos.Y][tPos.Z][tPos.X] = tile;
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
		UpdateTiles(Tile::Static, object);
		break;
	}
}

inline bool IsNotBuilding(ObjectTag tag)
{
	return tag != ObjectTag::Building && tag != ObjectTag::DissolveBuilding;
}

void Grid::UpdateTiles(Tile tile, GridObject* object)
{
	// ���� ������Ʈ�� Building �±��� ��쿡�� ������ ����
	if (IsNotBuilding(object->GetTag()))
		return;

	// ������Ʈ�� �浹 �ڽ�
	for (const auto& collider : object->GetComponent<ObjectCollider>()->GetColliders()) {
		// BFS�� ���� �ֺ� Ÿ�ϵ� ������Ʈ
		std::queue<Pos> q;
		std::map<Pos, bool> visited;

		if (collider->GetType() != Collider::Type::Box) {
			continue;
		}

		// ������Ʈ�� Ÿ�� ���� �ε��� ���
		Vec3 pos = collider->GetCenter();
		Pos index = Scene::I->GetTileUniqueIndexFromPos(pos);
		Scene::I->SetTileFromUniqueIndex(index, tile);
		q.push(index);

		// q�� �� ������ BFS�� ���� ���� Ÿ���� ������Ʈ�� �浹 �ߴٸ� �ش� Ÿ���� ������Ʈ
		while (!q.empty()) {
			Pos curNode = q.front();
			q.pop();

			for (int dir = 0; dir < 6; ++dir) {
				Pos nextPosT = curNode + gkFront2[dir];

				if (nextPosT.Y < 0 || nextPosT.Y >= mTileHeightCount) {
					continue;
				}

				if (visited[nextPosT]) {
					continue;
				}

				Vec3 nextPosW = Scene::I->GetTilePosFromUniqueIndex(nextPosT);
				BoundingBox bb{ nextPosW, mkTileExtent };

				visited[nextPosT] = true;

				if (collider->Intersects(bb)) {
					Scene::I->SetTileFromUniqueIndex(nextPosT, tile);
					q.push(nextPosT);
				}
			}
		}
	}
}

void Grid::UpdateTilesOnTerrain()
{
	// ����ũ �ε����� Ÿ�� �ε�����


	// Ÿ�� �ε����� ����ũ �ε����� 
}

void Grid::RenderVoxels()
{
	// ���� ���
	for (auto& voxel : mVoxelList) {
		MeshRenderer::RenderBox(voxel, Grid::mkTileExtent, Vec4{ 0.f, 1.f, 0.f, 1.f });
	}
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
		UpdateTiles(Tile::None, object);
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
		CheckCollisionObjects(mDynamicObjets);						// ���� ��ü�� �浹 �˻縦 �����Ѵ�.
		if (!mStaticObjects.empty()) {
			CheckCollisionObjects(mDynamicObjets, mStaticObjects);	// ����<->���� ��ü�� �浹 �˻縦 �����Ѵ�.
		}
	}
}

float Grid::CheckCollisionsRay(const Ray& ray) const
{
	float minDist = 999.f;
	for (const auto& object : mStaticObjects) {
		// ���� ������Ʈ�� Building �±��� ��쿡�� ������ ����
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

Vec3 Grid::GetTilePosCollisionsRay(const Ray& ray, const Vec3& target) const
{
	std::map<Pos, bool> visited;
	std::queue<Pos> q;

	Pos index = Scene::I->GetTileUniqueIndexFromPos(target);
	q.push(index);

	// q�� �� ������ BFS�� ���� ���� Ÿ���� ������Ʈ�� �浹 �ߴٸ� �ش� Ÿ���� ������Ʈ
	while (!q.empty()) {
		Pos curNode = q.front();
		q.pop();

		if (visited[curNode])
			continue;

		visited[curNode] = true;

		for (int dir = 0; dir < 4; ++dir) {
			Pos nextPosT = curNode + gkFront[dir];
			Vec3 nextPosW = Scene::I->GetTilePosFromUniqueIndex(nextPosT);
			nextPosW.y = target.y;

			BoundingBox bb{ nextPosW, Vec3{mkTileWidth, mkTileWidth, mkTileHeight} };
			float temp{};
			if (ray.Intersects(bb, temp)) {
				return nextPosW;
			}

			q.push(nextPosT);
		}
	}

	return Vec3{};
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
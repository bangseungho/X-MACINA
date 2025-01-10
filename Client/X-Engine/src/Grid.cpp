#include "EnginePch.h"
#include "Grid.h"

#include "Scene.h"
#include "Object.h"
#include "Component/Collider.h"
#include "MeshRenderer.h"

#include "DXGIMgr.h"
#include "FrameResource.h"
#include "Component/Camera.h"

int Grid::mTileRows = 0;
int Grid::mTileCols = 0;


Grid::Grid(int index, int width, const BoundingBox& bb)
	:
	mIndex(index),
	mBB(bb)
{
	mTileRows = static_cast<int>(width / mkTileWidth);
	mTileCols = static_cast<int>(width / mkTileWidth);
	mTiles = std::vector<std::vector<std::vector<Tile>>>(mTileHeightCount, std::vector<std::vector<Tile>>(mTileCols, std::vector<Tile>(mTileRows, Tile::None)));
}

Tile Grid::GetTileFromUniqueIndex(const Pos& tPos) const
{
	if (tPos.Y >= mTileHeightCount || tPos.Y < 0) {
		return Tile::None;
	}
	if (tPos.Z >= mTileCols || tPos.Z < 0) {
		return Tile::None;
	}
	if (tPos.X >= mTileRows || tPos.X < 0) {
		return Tile::None;
	}

	return mTiles[tPos.Y][tPos.Z][tPos.X];
}

RenderVoxel Grid::GetVoxelFromUniqueIndex(const Pos& uniqueIndex) const
{
	if (uniqueIndex.Y >= mTileHeightCount || uniqueIndex.Y < 0) {
		return RenderVoxel{};
	}
	if (uniqueIndex.X < 0 || uniqueIndex.Z < 0) {
		return RenderVoxel{};
	}
	if (uniqueIndex.X >= 3000 || uniqueIndex.Z >= 3000) {
		return RenderVoxel{};
	}

	auto findIt = mRenderVoxels.find(uniqueIndex);
	if (findIt == mRenderVoxels.end()) {
		return RenderVoxel{};
	}

	return findIt->second;
}

void Grid::SetTileFromUniqueIndex(const Pos& tPos, const Pos& index, Tile tile)
{
	if (tPos.Y >= mTileHeightCount || tPos.Y < 0) {
		return;
	}
	if (tPos.X < 0 || tPos.Z < 0) {
		return;
	}
	if (tPos.X >= 3000 || tPos.Z >= 3000) {
		return;
	}

	RenderVoxel renderVoxel;
	renderVoxel.VType = tile;
	switch (tile)
	{
	case Tile::Static:
		renderVoxel.VColor = Vec4{ 1.f, 0.f, 0.f, 1.f };
		break;
	case Tile::Terrain:
		renderVoxel.VColor = Vec4{ 0.f, 1.f, 0.f, 1.f };
		break;
	default:
		break;
	}

	mRenderVoxels.insert({ index, renderVoxel });
	mTiles[tPos.Y][tPos.Z][tPos.X] = tile;
}

void Grid::SetVoxelColorFromUniqueIndex(const Pos& index, const Vec4& color)
{
	// TODO ����ó��
	mRenderVoxels[index].VColor = color;
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

void Grid::UpdateMtx()
{
	for (auto& voxel : mRenderVoxels) {
		const Matrix scaleMtx = Matrix::CreateScale(mkTileWidth, mkTileHeight, mkTileWidth);
		const Matrix translationMtx = Matrix::CreateTranslation(Scene::I->GetTilePosFromUniqueIndex(voxel.first));
		const Matrix matrix = scaleMtx * translationMtx;
		voxel.second.MtxWorld = matrix.Transpose();


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

	Pos firstPos{};
	Vec3 result{};
	// q�� �� ������ BFS�� ���� ���� Ÿ���� ������Ʈ�� �浹 �ߴٸ� �ش� Ÿ���� ������Ʈ
	while (!q.empty()) {
		Pos curNode = q.front();
		q.pop();

		if (visited[curNode])
			continue;

		visited[curNode] = true;

		bool isEnd{};

		for (int dir = 0; dir < 6; ++dir) {
			Pos nextPosT = curNode + gkFront2[dir];
			Vec3 nextPosW = Scene::I->GetTilePosFromUniqueIndex(nextPosT);

			BoundingBox bb{ nextPosW, Vec3{mkTileWidth, mkTileWidth, mkTileHeight} };
			float temp{};
			if (ray.Intersects(bb, temp)) {
				isEnd = true;
				firstPos = nextPosT;
				while (!q.empty()) {
					q.pop();
					visited.clear();
				}
				result = nextPosW;
				q.push(firstPos);
				visited[firstPos] = true;
				break;
			}

			q.push(nextPosT);
		}

		if (isEnd) {
			break;
		}
	}

	int cnt{};
	float minValue{ FLT_MAX };
	while (!q.empty()) {
		Pos curNode = q.front();
		q.pop();

		if (cnt > 100) {
			break;
		}

		for (int dir = 0; dir < 6; ++dir) {
			Pos nextPosT = curNode + gkFront2[dir];
			Vec3 nextPosW = Scene::I->GetTilePosFromUniqueIndex(nextPosT);

			if (Scene::I->GetTileFromUniqueIndex(nextPosT) != Tile::Terrain) {
				continue;
			}

			if (visited[nextPosT]) {
				continue;
			}

			BoundingBox bb{ nextPosW, mkTileExtent };
			float dist{};
			if (ray.Intersects(bb, dist)) {
				if (minValue > dist) {
					minValue = dist;
					result = nextPosW;
				}
			}

			q.push(nextPosT);
			visited[nextPosT] = true;
		}

		cnt++;
	}

	return result;
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

void RenderVoxelManager::Init(Object* player)
{
	mPlayer = player;
	mInstanceBuffers.resize(FrameResourceMgr::mkFrameResourceCount);
	for (auto& buffer : mInstanceBuffers) {
		buffer = std::make_unique<UploadBuffer<InstanceData>>(DEVICE.Get(), mkMaxRenderVoxels * mkMaxRenderVoxels * Grid::mTileHeightCount, false);
	}
}

void RenderVoxelManager::Update()
{
	mRenderVoxels.clear();

	Pos pos = Scene::I->GetTileUniqueIndexFromPos(mPlayer->GetPosition());

	int x = pos.X, y = pos.Z; // �߽� ��ǥ

	int halfSize = mkMaxRenderVoxels / 2; // �߽����κ��� Ȯ�� ũ��

	// ���簢�� ���� ��ȸ
	for (int i = x - halfSize; i < x + halfSize; ++i) {
		for (int j = y - halfSize; j < y + halfSize; ++j) {
			for (int k = 0; k < Grid::mTileHeightCount; ++k) {
				if (Scene::I->GetTileFromUniqueIndex(Pos{ j, i, k }) != Tile::None) {
					mRenderVoxels.push_back(Pos{ j, i, k });
				}
			}
		}
	}
}

void RenderVoxelManager::Render()
{
	Update();

	DXGIMgr::I->SetGraphicsRootShaderResourceView(RootParam::Instancing, FrameResourceMgr::GetBufferGpuAddr(0, mInstanceBuffers[CURR_FRAME_INDEX].get()));

	int buffIdx{};
	for (auto& voxel : mRenderVoxels) {
		InstanceData instData;
		instData.MtxWorld = Scene::I->GetVoxelFromUniqueIndex(voxel).MtxWorld;
		instData.Color = Scene::I->GetVoxelFromUniqueIndex(voxel).VColor;
		mInstanceBuffers[CURR_FRAME_INDEX]->CopyData(buffIdx++, instData);
	}

	MeshRenderer::RenderInstancingBox(mRenderVoxels.size());
}

Pos RenderVoxelManager::PickTopVoxel(const Ray& ray)
{
	Pos result{};
	// ���� ������������ ���� ����
	int pickIdx{};
	for (int i = 0; i < mRenderVoxels.size(); ++i) {
		Vec3 voxelPosW = Scene::I->GetTilePosFromUniqueIndex(mRenderVoxels[i]);
		BoundingBox bb{ voxelPosW, Grid::mkTileExtent };

		float minValue{ FLT_MAX }, dist{};
		if (ray.Intersects(bb, dist)) {
			if (minValue > dist) {
				minValue = dist;
				result = mRenderVoxels[i];
				pickIdx = i;
			}
		}
	}
	
	Scene::I->SetVoxelColorFromUniqueIndex(mRenderVoxels[pickIdx], Vec4{ 1.f, 0.f, 0.f, 1.f });

	return result;
}

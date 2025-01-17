#pragma once

#include "UploadBuffer.h"

#pragma region ClassForwardDecl
class GridObject;
class Collider;
#pragma endregion


#pragma region Class
struct Voxel {
	Matrix			MtxWorld{};
	VoxelState		State = VoxelState::None;
	VoxelCondition	Condition = VoxelCondition::None;
};

class Grid {
private:
	const int mIndex{};
	const BoundingBox mBB{};

	std::unordered_set<GridObject*> mObjects{};			// all objects (env, static, dynamic, ...)
	std::unordered_set<GridObject*> mEnvObjects{};
	std::unordered_set<GridObject*> mStaticObjects{};
	std::unordered_set<GridObject*> mDynamicObjets{};

public:
	static constexpr float mkTileHeight = 0.5f;
	static constexpr float mkTileWidth = 0.5f;
	static constexpr Vec3 mkTileExtent = Vec3{ mkTileWidth / 2.f, mkTileWidth / 2.f, mkTileHeight / 2.f };
	static int mTileRows;
	static int mTileCols;

	// 추후에 UINT8로 변경
	std::unordered_map<Pos, int> mNearbyStaticCosts{};

private:
	std::unordered_map<Pos, Voxel> mVoxels{};
	// xz축에 대응하는 어떤 y 값이 있는지 미리 저장?

	// Static 주변 복셀들에 대하여 코스트맵 추가

public:
	Grid(int index, int width, const BoundingBox& bb);
	virtual ~Grid() = default;

	int GetIndex() const { return mIndex; }
	const BoundingBox& GetBB() const	{ return mBB; }

	// return all objects
	const auto& GetObjects() const		{ return mObjects; }

	Voxel GetVoxel(const Pos& index);
	int GetNearbyStaticCost(const Pos& index);
	VoxelState GetVoxelState(const Pos& tPos);
	VoxelCondition GetVoxelCondition(const Pos& tPos);
	void SetVoxelState(const Pos& index, VoxelState state);
	void SetVoxelCondition(const Pos& index, VoxelCondition condition);
	void SetNearbyStaticCost(const Pos& index, int cost);

public:
	bool Empty() const { return mObjects.empty(); }

	// add [object] to gird
	void AddObject(GridObject* object);

	// remove [object] from gird
	void RemoveObject(GridObject* object);

	bool Intersects(GridObject* object);

	// BFS를 활용하여 타일 업데이트
	void UpdateTiles(VoxelState tile, GridObject* object);

	// collision check for objects contained in grid
	void CheckCollisions();
	float CheckCollisionsRay(const Ray& ray) const;
	void CheckCollisions(rsptr<Collider> collider, std::vector<GridObject*>& out, CollisionType type = CollisionType::All) const;

private:
	static void CheckCollisionObjects(std::unordered_set<GridObject*> objects);
	static void CheckCollisionObjects(std::unordered_set<GridObject*> objectsA, std::unordered_set<GridObject*> objectsB);
	static void ProcessCollision(GridObject* objectA, GridObject* objectB);
};
#pragma endregion
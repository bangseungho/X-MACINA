#pragma once

#include "UploadBuffer.h"

#pragma region ClassForwardDecl
class GridObject;
class Collider;
#pragma endregion


#pragma region enum
enum class VoxelState : UINT8 {
	None = 0,
	Static,
	Dynamic,
	Terrain,
};

enum class VoxelCondition : UINT8 {
	None = 0,
	Picked,
	Closed,
	ReadyCreate,
};
#pragma endregion


#pragma region Using
#pragma endregion


#pragma region Class
struct RenderVoxel {
	Matrix			MtxWorld{};
	VoxelState		State{};
	VoxelCondition	Condition{};
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
	static constexpr int mTileHeightCount = 10;
	static int mTileRows;
	static int mTileCols;
	static int mVoxelCount;
	static float mVoxelSize;

	std::unordered_map<Pos, RenderVoxel> mRenderVoxels{};

public:
	Grid(int index, int width, const BoundingBox& bb);
	virtual ~Grid() = default;

	int GetIndex() const { return mIndex; }
	const BoundingBox& GetBB() const	{ return mBB; }

	// return all objects
	const auto& GetObjects() const		{ return mObjects; }

	VoxelState GetVoxelState(const Pos& tPos);
	VoxelCondition GetVoxelCondition(const Pos& tPos);
	RenderVoxel GetVoxelFromUniqueIndex(const Pos& index) const;
	void SetTileFromUniqueIndex(const Pos& index, VoxelState tile);
	void SetTileFromUniqueIndex(const Pos& index, VoxelCondition condition);
	void SetVoxelState(const Pos& index, VoxelState state);
	void SetVoxelCondition(const Pos& index, VoxelCondition condition);

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
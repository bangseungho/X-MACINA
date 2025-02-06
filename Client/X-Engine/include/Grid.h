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
	static constexpr float mkVoxelHeight = 0.5f;
	static constexpr float mkVoxelWidth = 0.5f;
	static constexpr Vec3 mkVoxelExtent = Vec3{ mkVoxelWidth / 2.f, mkVoxelWidth / 2.f, mkVoxelHeight / 2.f };
	static int mVoxelRows;
	static int mVoxelCols;

private:
	PairMap mCanWalkVoxels{};
	std::unordered_map<Pos, Voxel> mVoxels{};

	// 추후에 UINT8로 변경
	std::unordered_map<Pos, int> mProximityCosts{};
	std::unordered_map<Pos, float> mRowEdgeCosts{};
	std::unordered_map<Pos, float> mColEdgeCosts{};

public:
	Grid(int index, int width, const BoundingBox& bb);
	virtual ~Grid() = default;

	int GetIndex() const { return mIndex; }
	const BoundingBox& GetBB() const	{ return mBB; }

	// return all objects
	const auto& GetObjects() const		{ return mObjects; }

	Voxel GetVoxel(const Pos& index);
	PairMapRange GetCanWalkVoxels(const Pos& index);
	int GetProximityCost(const Pos& index);
	float GetEdgeCost(const Pos& index, bool isRowEdge);
	VoxelState GetVoxelState(const Pos& tPos);
	VoxelCondition GetVoxelCondition(const Pos& tPos);

public:
	void SetVoxelState(const Pos& index, VoxelState state);
	void SetVoxelCondition(const Pos& index, VoxelCondition condition);
	void SetProximityCost(const Pos& index, int cost, bool isReset);
	void RemoveCanWalkVoxel(const Pos& index, VoxelState state = VoxelState::Static);

public:
	bool Empty() const { return mObjects.empty(); }

	// add [object] to gird
	void AddObject(GridObject* object);

	// remove [object] from gird
	void RemoveObject(GridObject* object);

	bool Intersects(GridObject* object);

	// BFS를 활용하여 타일 업데이트
	void UpdateVoxels(VoxelState voxel, GridObject* object);
	void UpdateVoxelsEdgeCost(const std::unordered_set<Pos>& boundingVoxels);
	void UpdateTopVoxels(const std::unordered_set<Pos>& boundingVoxels);
	float CalcRowEdgeCost(const Pos& voxel, const std::unordered_set<Pos>& boundingVoxels);
	float CalcColEdgeCost(const Pos& voxel, const std::unordered_set<Pos>& boundingVoxels);

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
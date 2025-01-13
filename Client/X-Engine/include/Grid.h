#pragma once

#include "UploadBuffer.h"

#pragma region ClassForwardDecl
class Object;
class GridObject;
class Collider;
class InstObject;
#pragma endregion


#pragma region enum
enum class Tile: UINT8{
	None = 0,
	Static,
	Dynamic,
	Terrain,
};
#pragma endregion

#pragma region Using
using namespace Path;
#pragma endregion

#pragma region Class
struct RenderVoxel {
	Vec4 VColor{};
	Tile VType{};
	Matrix MtxWorld{};
	bool IsPicked{};
};

class VoxelManager : public Singleton<VoxelManager> {
	friend Singleton;

private:
	Object*				mPlayer{};
	Pos					mSelectedVoxel{};
	std::vector<Pos>	mRenderVoxels{};
	bool				mReadyMakePath{};
	bool				mHoldingClick{};

	Pos mStartPos{};
	Pos mDestPos{};

private:
	std::vector<uptr<UploadBuffer<InstanceData>>> mInstanceBuffers{};

public:
	static constexpr UINT mkMaxRenderVoxels = 50;

public:
	const Pos& GetSelectedVoxelPos() const { return mSelectedVoxel; }
	void ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam);

public:
	void Init(Object* player);
	void Update();
	void Render();
	void PickTopVoxel(bool makePath);
};

class Grid {
private:
	const int mIndex{};
	const BoundingBox mBB{};

	std::vector<std::vector<std::vector<Tile>>> mTiles{};

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

	std::map<Pos, RenderVoxel> mRenderVoxels{};
	std::vector<RenderVoxel> mRederVoxels{};

public:
	Grid(int index, int width, const BoundingBox& bb);
	virtual ~Grid() = default;

	int GetIndex() const { return mIndex; }
	const BoundingBox& GetBB() const	{ return mBB; }

	// return all objects
	const auto& GetObjects() const		{ return mObjects; }

	Tile GetTileFromUniqueIndex(const Pos& tPos) const;
	RenderVoxel GetVoxelFromUniqueIndex(const Pos& uniqueIndex) const;
	void SetTileFromUniqueIndex(const Pos& tPos, const Pos& index, Tile tile);
	void SetVoxelColorFromUniqueIndex(const Pos& index, const Vec4& color);
	void SetPickingFlagFromUniqueIndex(const Pos& index, bool isPicked);

public:
	bool Empty() const { return mObjects.empty(); }

	// add [object] to gird
	void AddObject(GridObject* object);

	// remove [object] from gird
	void RemoveObject(GridObject* object);

	bool Intersects(GridObject* object);

	// BFS를 활용하여 타일 업데이트
	void UpdateTiles(Tile tile, GridObject* object);
	void UpdateMtx();

	// collision check for objects contained in grid
	void CheckCollisions();
	float CheckCollisionsRay(const Ray& ray) const;
	void CheckCollisions(rsptr<Collider> collider, std::vector<GridObject*>& out, CollisionType type = CollisionType::All) const;
	Vec3 GetTilePosCollisionsRay(const Ray& ray, const Vec3& target) const;

private:
	static void CheckCollisionObjects(std::unordered_set<GridObject*> objects);
	static void CheckCollisionObjects(std::unordered_set<GridObject*> objectsA, std::unordered_set<GridObject*> objectsB);
	static void ProcessCollision(GridObject* objectA, GridObject* objectB);
};
#pragma endregion
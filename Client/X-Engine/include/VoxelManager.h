#pragma once

#include "UploadBuffer.h"

#pragma region ClassForwardDecl
class Object;
#pragma endregion

#pragma region Using
#pragma endregion


#pragma region Enum
enum class RenderMode : UINT8 {
	Voxel = 0,
	World = 1,
	Both = 2,
};

enum class CreateMode : UINT8 {
	None = 0,
	Create,
	Remove,
};
#pragma endregion


#pragma region Struct
struct VoxelOption {
	int	RenderVoxelRows{};
	int	RenderVoxelCols{};
	int	RenderVoxelHeight{};

	RenderMode RenderMode{};
	CreateMode CreateMode{};
};
#pragma endregion


#pragma region Class
class VoxelManager : public Singleton<VoxelManager> {
	friend Singleton;

private:
	Object* mPlayer{};
	Pos					mSelectedVoxel{};
	std::vector<Pos>	mRenderVoxels{};
	bool				mReadyMakePath{};
	bool				mHoldingClick{};
	Pos					mStartPos{};

private:
	VoxelOption			mOption{};

private:
	std::vector<uptr<UploadBuffer<InstanceData>>> mInstanceBuffers{};
	std::vector<Pos> mClosedList{};

public:
	static constexpr UINT mkMaxRenderVoxels = 70;

public:
	const Pos& GetSelectedVoxelPos() const { return mSelectedVoxel; }

public:
	int GetRenderVoxelRows() const { return mOption.RenderVoxelRows; }
	CreateMode GetCreateMode() const { return mOption.CreateMode; }
	RenderMode GetRenderMode() const { return mOption.RenderMode; }

public:
	void SetRenderVoxelRows(int rows) { CalcRenderVoxelCount(rows); }
	void SetCreateMode(CreateMode mode) { mOption.CreateMode = mode; }
	void SetRenderMode(RenderMode mode) { mOption.RenderMode = mode; }

public:
	void PushClosedVoxel(const Pos& pos) { mClosedList.push_back(pos); }
	void ClearClosedList();

public:
	void ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam);
	void Init(Object* player);
	void Update();
	void Render();
	void PickTopVoxel(bool makePath);

public:
	

private:
	void CalcRenderVoxelCount(int renderVoxelRows);
};
#pragma endregion
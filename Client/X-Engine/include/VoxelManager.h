#pragma once

#include "UploadBuffer.h"

#pragma region ClassForwardDecl
class Object;
class Camera;
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

	RenderMode RenderMode = RenderMode::Voxel;
	CreateMode CreateMode = CreateMode::None;
};
#pragma endregion


#pragma region Class
class VoxelManager : public Singleton<VoxelManager> {
	friend Singleton;

private:
	Object*				mPlayer{};
	std::vector<Pos>	mRenderVoxels{};
	bool				mReadyMakePath{ true };
	bool				mHoldingClick{};

private:
	Pos					mSelectedVoxel{};
	Pos					mCenterPos{};

private:
	VoxelOption			mOption{};

private:
	std::vector<uptr<UploadBuffer<InstanceData>>> mInstanceBuffers{};
	std::vector<Pos> mClosedList{};

public:
	static constexpr UINT mkMaxRenderVoxelCount = 60000;

public:
	const Pos& GetSelectedVoxelPos() const { return mSelectedVoxel; }

public:
	int GetRenderVoxelRows() const { return mOption.RenderVoxelRows; }
	int GetRenderVoxelHeight() const { return mOption.RenderVoxelHeight; }
	CreateMode GetCreateMode() const { return mOption.CreateMode; }
	RenderMode GetRenderMode() const { return mOption.RenderMode; }

public:
	void SetRenderVoxelRows(int rows) { CalcRenderVoxelCount(rows); }
	void SetRenderVoxelHeight(int height) { mOption.RenderVoxelHeight = height; }
	void SetCreateMode(CreateMode mode) { mOption.CreateMode = mode; }
	void SetRenderMode(RenderMode mode) { mOption.RenderMode = mode; }
	void SetCenterPos(const Pos& pos) { mCenterPos = pos; }

public:
	void PushClosedVoxel(const Pos& pos) { mClosedList.push_back(pos); }
	void ClearClosedList();

public:
	void ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam);
	void Init(Object* player);
	void Update();
	void Render();

private:
	void PickTopVoxel(bool makePath);
	void UpdateCreateMode(VoxelState selectedVoxelState);
	void UpdateRemoveMode(VoxelState selectedVoxelState);
	void UpdateDefaultMode(bool makePath, VoxelState selectedVoxelState);
	
	void CalcRenderVoxelCount(int renderVoxelRows);
};
#pragma endregion
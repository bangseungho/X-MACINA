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
	World,
	Both,
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
	std::vector<Pos> mCloseList{};
	std::vector<Pos> mOpenList{};
	std::unordered_set<Pos> mUsedCreateModeVoxels{};

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
	void SetRenderVoxelRows(int rows) { CalcRenderVoxelCount(rows); SetCenterPos(mCenterPos, false); }
	void SetRenderVoxelHeight(int height) { mOption.RenderVoxelHeight = height; SetCenterPos(mCenterPos, false); }
	void SetCreateMode(CreateMode mode) { mOption.CreateMode = mode; }
	void SetRenderMode(RenderMode mode) { mOption.RenderMode = mode; }
	void SetCenterPos(const Pos& pos, bool checkCenterPos = true);

public:
	void PushClosedVoxel(const Pos& pos) { mCloseList.push_back(pos); }
	void PushOpenedVoxel(const Pos& pos) { mOpenList.push_back(pos); }
	void ClearPathList();

public:
	void ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam);
	void Init(Object* player);
	void Render();

private:
	void PickTopVoxel(bool makePath);
	void UpdateCreateMode(VoxelState selectedVoxelState);
	void UpdateRemoveMode(VoxelState selectedVoxelState);
	void UpdatePlanningPathMode(bool makePath, VoxelState selectedVoxelState);
	void CalcRenderVoxelCount(int renderVoxelRows);
};
#pragma endregion
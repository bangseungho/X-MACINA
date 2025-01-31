#pragma once

#include "UploadBuffer.h"

#pragma region ClassForwardDecl
class Object;
class Camera;
class Agent;
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
	struct VoxelSort {
		bool operator()(const Pos& lhs, const Pos& rhs)const {
			if (lhs.Y != rhs.Y) return lhs.Y > rhs.Y;
			else if (lhs.Z != rhs.Z) return lhs.Z < rhs.Z;
			return lhs.X < rhs.X;
		}
	};

private:
	std::set<Pos, VoxelSort> mRenderVoxels{};
	Agent*					mPickedAgent{};
	bool					mReadyMakePath{ true };
	bool					mHoldingClick{};

private:
	Pos						mSelectedVoxel{};
	Pos						mCenterPos{};
	int						mSelectedVoxelProximityCost{};
	std::pair<float, float>	mSelectedVoxelEdgeCost{};

private:
	Pos						mAboveVoxel{};

private:
	VoxelOption				mOption{};

private:
	std::vector<uptr<UploadBuffer<InstanceData>>> mInstanceBuffers{};
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
	const std::pair<float, float> GetSelectedVoxelEdgeCost() const { return mSelectedVoxelEdgeCost; }
	int GetSelectedVoxelProximityCost() const { return mSelectedVoxelProximityCost; }

public:
	void SetRenderVoxelRows(int rows) { CalcRenderVoxelCount(rows); UpdateRenderVoxels(mCenterPos, false); }
	void SetRenderVoxelHeight(int height) { mOption.RenderVoxelHeight = height; UpdateRenderVoxels(mCenterPos, false); }
	void SetCreateMode(CreateMode mode) { mOption.CreateMode = mode; }
	void SetRenderMode(RenderMode mode) { mOption.RenderMode = mode; }

public:
	void UpdateRenderVoxels(const Pos& pos, bool checkCenterPos = true);

public:
	void ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam);
	void ProcessKeyboardMsg(UINT messageID, WPARAM wParam, LPARAM lParam);
	void Init();
	void Render();

private:
	void PickTopVoxel(bool makePath);
	void UpdateCreateMode(VoxelState selectedVoxelState);
	void UpdateRemoveMode(VoxelState selectedVoxelState);
	void UpdatePlanningPathMode(bool makePath, VoxelState selectedVoxelState);
	void CalcRenderVoxelCount(int renderVoxelRows);
};
#pragma endregion
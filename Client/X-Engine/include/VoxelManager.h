#pragma once

#include "UploadBuffer.h"

#pragma region ClassForwardDecl
class Object;
#pragma endregion

#pragma region Using
using namespace Path;
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
#pragma endregion
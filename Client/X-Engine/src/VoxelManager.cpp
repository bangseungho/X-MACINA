#include "EnginePch.h"
#include "VoxelManager.h"

#include "Scene.h"
#include "Object.h"
#include "Component/Collider.h"
#include "MeshRenderer.h"

#include "DXGIMgr.h"
#include "Grid.h"
#include "FrameResource.h"
#include "Component/Camera.h"
#include "Component/Agent.h"
#include "InputMgr.h"


void VoxelManager::ClearPathList()
{
	for (auto& voxel : mCloseList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
	}

	for (auto& voxel : mOpenList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
	}

	mCloseList.clear();
	mOpenList.clear();
}

void VoxelManager::ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam)
{
	switch (messageID) {
	case WM_RBUTTONDOWN:
		mHoldingClick = true;
		PickTopVoxel(true);
		break;
	case WM_RBUTTONUP:
		mHoldingClick = false;
		mUsedCreateModeVoxels.clear();
		break;
	case WM_MOUSEMOVE:
		PickTopVoxel(false);
		break;

	default:
		break;
	}
}

void VoxelManager::Init(Object* player)
{
	mPlayer = player;
	CalcRenderVoxelCount(50);
	mCloseList.reserve(10000);
	mOpenList.reserve(10000);
	mRenderVoxels.reserve(10000);
	mOption.RenderVoxelHeight = 10;

	mInstanceBuffers.resize(FrameResourceMgr::mkFrameResourceCount);
	for (auto& buffer : mInstanceBuffers) {
		buffer = std::make_unique<UploadBuffer<InstanceData>>(DEVICE.Get(), mkMaxRenderVoxelCount, false);
	}
}

void VoxelManager::Update()
{
	mRenderVoxels.clear();

	int x = mCenterPos.X, y = mCenterPos.Z; // 중심 좌표

	int halfSizeX = mOption.RenderVoxelRows / 2; // 중심으로부터 확장 크기
	int halfSizeZ = mOption.RenderVoxelCols / 2;

	for (int i = x - halfSizeX; i < x + halfSizeX; ++i) {
		for (int j = y - halfSizeZ; j < y + halfSizeZ; ++j) {
			for (int k = 0; k < VoxelManager::mOption.RenderVoxelHeight; ++k) {
				const Pos index = Pos{ j, i, k };
				if (Scene::I->GetVoxelState(index) != VoxelState::None || Scene::I->GetVoxelCondition(index) == VoxelCondition::ReadyCreate) {
					mRenderVoxels.push_back(index);
				}
			}
		}
	}
}

void VoxelManager::Render()
{
	if (mOption.RenderMode == RenderMode::World) {
		return;
	}

	DXGIMgr::I->SetGraphicsRootShaderResourceView(RootParam::Instancing, FrameResourceMgr::GetBufferGpuAddr(0, mInstanceBuffers[CURR_FRAME_INDEX].get()));

	int buffIdx{};

	for (auto& voxel : mOpenList) {
		if (Scene::I->GetVoxelCondition(voxel) != VoxelCondition::Picked) {
			Scene::I->SetVoxelCondition(voxel, VoxelCondition::Opened);
		}
	}
	for (auto& voxel : mCloseList) {
		if (Scene::I->GetVoxelCondition(voxel) != VoxelCondition::Picked) {
			Scene::I->SetVoxelCondition(voxel, VoxelCondition::Closed);
		}
	}

	for (auto& voxel : mRenderVoxels) {
		Voxel renderVoxel = Scene::I->GetVoxel(voxel);

		InstanceData instData;
		instData.MtxWorld = renderVoxel.MtxWorld;
		
		switch (renderVoxel.State) {
		case VoxelState::Static:
		case VoxelState::TerrainStatic:
			instData.Color = Vec4{ 1.f, 0.f, 0.f, 1.f };
			break;
		case VoxelState::Terrain:
			instData.Color = Vec4{ 0.f, 1.f, 0.f, 1.f };
			break;
		default:
			break;
		}

		switch (renderVoxel.Condition) {
		case VoxelCondition::Picked:
			instData.Color = Vec4{ 1.f, 0.f, 1.f, 1.f };
			break;
		case VoxelCondition::Closed:
			instData.Color = Vec4{ 0.f, 0.f, 1.f, 1.f };
			break;
		case VoxelCondition::Opened:
			instData.Color = Vec4{ 0.f, 1.f, 1.f, 1.f };
			break;
		case VoxelCondition::ReadyCreate:
			instData.Color = Vec4{ 1.f, 0.f, 0.f, 1.f };
			break;
		default:
			break;
		}

		mInstanceBuffers[CURR_FRAME_INDEX]->CopyData(buffIdx++, instData);
	}

	MeshRenderer::RenderInstancingBox(static_cast<UINT>(mRenderVoxels.size()));
}

void VoxelManager::PickTopVoxel(bool makePath)
{
	const Vec2& aimPos = InputMgr::I->GetMousePos();
	const Ray& ray = MAIN_CAMERA->ScreenToWorldRay(aimPos);

	Pos start = Scene::I->GetVoxelIndex(mPlayer->GetPosition());
	mSelectedVoxel = Pos{};
	// 추후 분할정복으로 변경 예정
	float minValue{ FLT_MAX };
	for (int i = 0; i < mRenderVoxels.size(); ++i) {
		Vec3 voxelPosW = Scene::I->GetVoxelPos(mRenderVoxels[i]);
		Scene::I->SetVoxelCondition(mRenderVoxels[i], VoxelCondition::None);
		if (Scene::I->GetVoxelState(mRenderVoxels[i]) == VoxelState::None) continue;
		BoundingBox bb{ voxelPosW, Grid::mkTileExtent };
		float dist{};
		if (ray.Intersects(bb, dist)) {
			if (minValue > dist) {
				minValue = dist;
				mSelectedVoxel = mRenderVoxels[i];
			}
		}
	}
	
	VoxelState selectedVoxelState = Scene::I->GetVoxelState(mSelectedVoxel);

	switch (mOption.CreateMode)
	{
	case CreateMode::None:
		UpdatePlanningPathMode(makePath, selectedVoxelState);
		break;
	case CreateMode::Create:
		UpdateCreateMode(selectedVoxelState);
		break;
	case CreateMode::Remove:
		UpdateRemoveMode(selectedVoxelState);
		break;
	default:
		break;
	}

	Scene::I->SetVoxelCondition(mSelectedVoxel, VoxelCondition::Picked);
}

void VoxelManager::UpdateCreateMode(VoxelState selectedVoxelState)
{
	Pos aboveVoxel = mSelectedVoxel.Up();
	if (Scene::I->GetVoxel(aboveVoxel).State == VoxelState::None) {
		Scene::I->SetVoxelCondition(aboveVoxel, VoxelCondition::ReadyCreate);
	}

	if (!mHoldingClick || mUsedCreateModeVoxels.count(mSelectedVoxel.XZ())) {
		return;
	}

	if (selectedVoxelState == VoxelState::Terrain || selectedVoxelState == VoxelState::TerrainStatic) {
		Scene::I->SetVoxelState(mSelectedVoxel, VoxelState::TerrainStatic);
	}
	else {
		Scene::I->SetVoxelState(mSelectedVoxel, VoxelState::Static);
	}
	Scene::I->SetVoxelState(aboveVoxel, VoxelState::Static);

	mUsedCreateModeVoxels.insert(mSelectedVoxel.XZ());
}

void VoxelManager::UpdateRemoveMode(VoxelState selectedVoxelState)
{
	if (!mHoldingClick || mUsedCreateModeVoxels.count(mSelectedVoxel.XZ())) {
		return;
	}

	if (selectedVoxelState == VoxelState::TerrainStatic) {
		Scene::I->SetVoxelState(mSelectedVoxel, VoxelState::Terrain);
	}
	else if (selectedVoxelState == VoxelState::Static) {
		Scene::I->SetVoxelState(mSelectedVoxel, VoxelState::None);
	}

	mUsedCreateModeVoxels.insert(mSelectedVoxel.XZ());
}

void VoxelManager::UpdatePlanningPathMode(bool makePath, VoxelState selectedVoxelState)
{
	if (!makePath) {
		return;
	}

	if (!mReadyMakePath) {
		if (!mPlayer->GetComponent<Agent>()->PathPlanningToAstar(mSelectedVoxel)) {
			return;
		}
	}
	else {
		mPlayer->GetComponent<Agent>()->ReadyPlanningToPath(mSelectedVoxel);
	}
	mReadyMakePath = !mReadyMakePath;
}

void VoxelManager::CalcRenderVoxelCount(int renderVoxelRows)
{
	const float ratio = static_cast<float>(DXGIMgr::I->GetWindowHeight()) / DXGIMgr::I->GetWindowWidth();
	mOption.RenderVoxelRows = renderVoxelRows;
	mOption.RenderVoxelCols = static_cast<int>(ratio * mOption.RenderVoxelRows);
}

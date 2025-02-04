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
#include "Imgui/ImGuiManager.h"

void VoxelManager::ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam)
{
	switch (messageID) {
	case WM_RBUTTONDOWN:
		mHoldingClick = true;
		PickTopVoxel(true);
		break;
	case WM_LBUTTONDOWN:
		AgentManager::I->PickAgent(&mPickedAgent);
		SetAgent(mPickedAgent);
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


void VoxelManager::ProcessKeyboardMsg(UINT messageID, WPARAM wParam, LPARAM lParam)
{
	switch (messageID) {
	case WM_KEYDOWN:
		switch (wParam)
		{
		case '1':
			mOption.CreateMode = CreateMode::None;
			break;
		case '2':
			mOption.CreateMode = CreateMode::Create;
			break;
		case '3':
			mOption.CreateMode = CreateMode::Remove;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

void VoxelManager::SetAgent(Agent* agent) {
	mPickedAgent = agent; 
	ImGuiManager::I->SetAgent(mPickedAgent);
}

void VoxelManager::Init()
{
	CalcRenderVoxelCount(50);
	mOption.RenderVoxelHeight = 10;

	mInstanceBuffers.resize(FrameResourceMgr::mkFrameResourceCount);
	for (auto& buffer : mInstanceBuffers) {
		buffer = std::make_unique<UploadBuffer<InstanceData>>(DEVICE.Get(), mkMaxRenderVoxelCount, false);
	}
}

void VoxelManager::UpdateRenderVoxels(const Pos& pos, bool checkCenterPos)
{
	if (pos == mCenterPos && checkCenterPos) {
		return;
	}

	mRenderVoxels.clear();

	int x = pos.X, y = pos.Z; // 중심 좌표
	int halfSizeX = mOption.RenderVoxelRows / 2; // 중심으로부터 확장 크기
	int halfSizeZ = mOption.RenderVoxelCols / 2;

	for (int i = x - halfSizeX; i < x + halfSizeX; ++i) {
		for (int j = y - halfSizeZ; j < y + halfSizeZ; ++j) {
			for (int k = 0; k < VoxelManager::mOption.RenderVoxelHeight; ++k) {
				const Pos& voxel = Pos{ j, i, k };
				if (Scene::I->GetVoxelState(voxel) != VoxelState::None || Scene::I->GetVoxelCondition(voxel) == VoxelCondition::ReadyCreate) {
					mRenderVoxels.insert(voxel);
				}
			}
		}
	}

	mCenterPos = pos;
}

void VoxelManager::Render()
{
	if (mOption.RenderMode == RenderMode::World) {
		return;
	}

	DXGIMgr::I->SetGraphicsRootShaderResourceView(RootParam::Instancing, FrameResourceMgr::GetBufferGpuAddr(0, mInstanceBuffers[CURR_FRAME_INDEX].get()));
	AgentManager::I->RenderPathList();

	int buffIdx{};
	for (auto& renderVoxel : mRenderVoxels) {
		Voxel voxel = Scene::I->GetVoxel(renderVoxel);

		InstanceData instData;
		instData.MtxWorld = voxel.MtxWorld;
		
		switch (voxel.State) {
		case VoxelState::Static:
		case VoxelState::CanWalk:
		case VoxelState::Dynamic:
			instData.Color = Vec4{ 1.f, 0.f, 0.f, 1.f };
			break;
		case VoxelState::Terrain:
			instData.Color = Vec4{ 0.f, 1.0f, 0.f, 1.f };
			break;
		default:
			break;
		}

		switch (voxel.Condition) {
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

	mSelectedVoxel = Pos{};
	// 추후 분할정복으로 변경 예정
	float minValue{ FLT_MAX };
	for (const auto& voxel : mRenderVoxels) {
		Vec3 voxelPosW = Scene::I->GetVoxelPos(voxel);
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
		if (Scene::I->GetVoxelState(voxel) == VoxelState::None) continue;
		BoundingBox bb{ voxelPosW, Grid::mkVoxelExtent };
		float dist{};
		if (ray.Intersects(bb, dist)) {
			if (minValue > dist) {
				minValue = dist;
				mSelectedVoxel = voxel;
			}
		}
	}

	VoxelState selectedVoxelState = Scene::I->GetVoxelState(mSelectedVoxel);
	mSelectedVoxelEdgeCost.first = Scene::I->GetEdgeCost(mSelectedVoxel, true);
	mSelectedVoxelEdgeCost.second = Scene::I->GetEdgeCost(mSelectedVoxel, false);
	mSelectedVoxelProximityCost = Scene::I->GetProximityCost(mSelectedVoxel);

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
	if (Scene::I->GetVoxelState(mAboveVoxel) == VoxelState::None) {
		mRenderVoxels.erase(mAboveVoxel);
	}

	if (Scene::I->GetVoxel(mSelectedVoxel.Up()).State == VoxelState::None) {
		Scene::I->SetVoxelCondition(mSelectedVoxel.Up(), VoxelCondition::ReadyCreate);
	}
	
	mAboveVoxel = mSelectedVoxel.Up();
	mRenderVoxels.insert(mAboveVoxel);

	if (!mHoldingClick || mUsedCreateModeVoxels.count(mSelectedVoxel.XZ())) {
		return;
	}

	Scene::I->RemoveCanWalkVoxel(mSelectedVoxel);
	Scene::I->SetVoxelState(mSelectedVoxel.Up(), VoxelState::Dynamic);

	mUsedCreateModeVoxels.insert(mSelectedVoxel.XZ());
	Scene::I->UpdateVoxelsProximityCost(mSelectedVoxel);
}

void VoxelManager::UpdateRemoveMode(VoxelState selectedVoxelState)
{
	if (!mHoldingClick || mUsedCreateModeVoxels.count(mSelectedVoxel.XZ())) {
		return;
	}

	if (selectedVoxelState != VoxelState::Terrain) {
		Scene::I->SetVoxelState(mSelectedVoxel, VoxelState::None);
		Scene::I->SetVoxelState(mSelectedVoxel.Down(), VoxelState::CanWalk);
		mRenderVoxels.erase(mSelectedVoxel);
	}

	mUsedCreateModeVoxels.insert(mSelectedVoxel.XZ());
	Scene::I->UpdateVoxelsProximityCost(mSelectedVoxel, true);
}

void VoxelManager::UpdatePlanningPathMode(bool makePath, VoxelState selectedVoxelState)
{
	if (!makePath) {
		return;
	}

	if (!mPickedAgent) {
		return;
	}

	if (!mReadyMakePath) {
		std::vector<Vec3> path = mPickedAgent->PathPlanningToAstar(mSelectedVoxel, {}, true);
		if (path.empty()) {
			return;
		}
		mPickedAgent->SetPath(path);
	}
	else {
		mPickedAgent->ReadyPlanningToPath(mSelectedVoxel);
	}
	mReadyMakePath = !mReadyMakePath;
}

void VoxelManager::CalcRenderVoxelCount(int renderVoxelRows)
{
	const float ratio = static_cast<float>(DXGIMgr::I->GetWindowHeight()) / DXGIMgr::I->GetWindowWidth();
	mOption.RenderVoxelRows = renderVoxelRows;
	mOption.RenderVoxelCols = static_cast<int>(ratio * mOption.RenderVoxelRows);
}

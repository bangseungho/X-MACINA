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


void VoxelManager::ClearClosedList()
{
	for (auto& voxel : mClosedList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::None);
	}

	mClosedList.clear();
}

void VoxelManager::ProcessMouseMsg(UINT messageID, WPARAM wParam, LPARAM lParam)
{
	switch (messageID) {
	case WM_RBUTTONDOWN:
		PickTopVoxel(true);
		break;
	case WM_RBUTTONUP:
		mHoldingClick = false;
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
	mReadyMakePath = true;
	mPlayer = player;
	mInstanceBuffers.resize(FrameResourceMgr::mkFrameResourceCount);
	CalcRenderVoxelCount(50);
	mClosedList.reserve(1000);
	mRenderVoxels.reserve(5000);
	for (auto& buffer : mInstanceBuffers) {
		buffer = std::make_unique<UploadBuffer<InstanceData>>(DEVICE.Get(), mkMaxRenderVoxels * mkMaxRenderVoxels * Grid::mTileHeightCount, false);
	}
}

void VoxelManager::Update()
{
	mRenderVoxels.clear();

	Pos pos = Scene::I->GetTileUniqueIndexFromPos(mPlayer->GetPosition());

	int x = pos.X, y = pos.Z; // 중심 좌표

	int halfSizeX = mOption.RenderVoxelRows / 2; // 중심으로부터 확장 크기
	int halfSizeZ = mOption.RenderVoxelCols / 2;

	// 정사각형 범위 순회
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

	for (auto& voxel : mClosedList) {
		Scene::I->SetVoxelCondition(voxel, VoxelCondition::Closed);
	}

	for (auto& voxel : mRenderVoxels) {
		RenderVoxel renderVoxel = Scene::I->GetVoxelFromUniqueIndex(voxel);

		InstanceData instData;
		instData.MtxWorld = renderVoxel.MtxWorld;
		
		switch (renderVoxel.State) {
		case VoxelState::Static:
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
	// 추후에 쿼드 트리 탐색으로 변경 예정
	if (mHoldingClick) {
		return;
	}

	const Vec2& aimPos = InputMgr::I->GetMousePos();
	const Ray& ray = MAIN_CAMERA->ScreenToWorldRay(aimPos);

	Pos start = Scene::I->GetTileUniqueIndexFromPos(mPlayer->GetPosition());
	mSelectedVoxel = Pos{};
	// 추후 분할정복으로 변경 예정
	float minValue{ FLT_MAX };
	for (int i = 0; i < mRenderVoxels.size(); ++i) {
		Vec3 voxelPosW = Scene::I->GetTilePosFromUniqueIndex(mRenderVoxels[i]);
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
	
	if (mOption.CreateMode == CreateMode::Create) {
		Pos aboveVoxel = mSelectedVoxel + Pos{ 0, 0, 1 };
		if (Scene::I->GetVoxelFromUniqueIndex(aboveVoxel).State == VoxelState::None) {
			Scene::I->SetVoxelCondition(aboveVoxel, VoxelCondition::ReadyCreate);
		}

		if (makePath) {
			Scene::I->SetVoxelState(mSelectedVoxel, VoxelState::Static);
			Scene::I->SetVoxelState(aboveVoxel, VoxelState::Static);
		}
	}
	else {
		Scene::I->SetVoxelCondition(mSelectedVoxel, VoxelCondition::Picked);
		if (makePath) {
			if (mOption.CreateMode == CreateMode::Remove) {
				if (Scene::I->GetVoxelState(mSelectedVoxel) == VoxelState::Static) {
					Scene::I->SetVoxelState(mSelectedVoxel, VoxelState::None);
				}
			}
			else {
				mPlayer->GetComponent<Agent>()->PathPlanningToAstar(mSelectedVoxel);
			}
		}
	}
}

void VoxelManager::CalcRenderVoxelCount(int renderVoxelRows)
{
	const float ratio = static_cast<float>(DXGIMgr::I->GetWindowHeight()) / DXGIMgr::I->GetWindowWidth();
	mOption.RenderVoxelRows = renderVoxelRows;
	mOption.RenderVoxelCols = static_cast<int>(ratio * mOption.RenderVoxelRows);
	mOption.RenderVoxelHeight = 10;
}

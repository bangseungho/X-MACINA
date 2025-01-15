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
	for (auto& buffer : mInstanceBuffers) {
		buffer = std::make_unique<UploadBuffer<InstanceData>>(DEVICE.Get(), mkMaxRenderVoxels * mkMaxRenderVoxels * Grid::mTileHeightCount, false);
	}
}

void VoxelManager::Update()
{
	mRenderVoxels.clear();

	Pos pos = Scene::I->GetTileUniqueIndexFromPos(mPlayer->GetPosition());

	int x = pos.X, y = pos.Z; // 중심 좌표

	int halfSize = mkMaxRenderVoxels / 2; // 중심으로부터 확장 크기

	// 정사각형 범위 순회
	for (int i = x - halfSize; i < x + halfSize; ++i) {
		for (int j = y - halfSize; j < y + halfSize; ++j) {
			for (int k = 0; k < Grid::mTileHeightCount; ++k) {
				if (Scene::I->GetTileFromUniqueIndex(Pos{ j, i, k }) != Tile::None) {
					mRenderVoxels.push_back(Pos{ j, i, k });
				}
			}
		}
	}
}

void VoxelManager::Render()
{
	Update();

	DXGIMgr::I->SetGraphicsRootShaderResourceView(RootParam::Instancing, FrameResourceMgr::GetBufferGpuAddr(0, mInstanceBuffers[CURR_FRAME_INDEX].get()));

	int buffIdx{};
	for (auto& voxel : mRenderVoxels) {
		InstanceData instData;
		instData.MtxWorld = Scene::I->GetVoxelFromUniqueIndex(voxel).MtxWorld;
		
		if (Scene::I->GetVoxelFromUniqueIndex(voxel).IsPicked) {
			instData.Color = Vec4{ 1.f, 0.f, 1.f, 1.f };
		}
		else {
			instData.Color = Scene::I->GetVoxelFromUniqueIndex(voxel).VColor;
		}

		mInstanceBuffers[CURR_FRAME_INDEX]->CopyData(buffIdx++, instData);
	}

	MeshRenderer::RenderInstancingBox(mRenderVoxels.size());
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
		Scene::I->SetPickingFlagFromUniqueIndex(mRenderVoxels[i], false);
		BoundingBox bb{ voxelPosW, Grid::mkTileExtent };

		float dist{};
		if (ray.Intersects(bb, dist)) {
			if (minValue > dist) {
				minValue = dist;
				mSelectedVoxel = mRenderVoxels[i];
			}
		}
	}
	
	Scene::I->SetPickingFlagFromUniqueIndex(mSelectedVoxel, true);

	if (makePath) {
		mPlayer->GetComponent<Agent>()->PathPlanningToAstar(mSelectedVoxel);
	}
}

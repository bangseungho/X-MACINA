#include "stdafx.h"
#include "Script_CameraTarget.h"

#include "InputMgr.h"
#include "Scene.h"
#include "Timer.h"
#include "Script_MainCamera.h"
#include "Component/Camera.h"
#include "VoxelManager.h"

void Script_CameraTarget::Start()
{
	mCamera = MainCamera::I->GetComponent<Script_MainCamera>().get();
}

void Script_CameraTarget::Update()
{
	VoxelManager::I->SetCenterPos(Scene::I->GetVoxelIndex(mObject->GetPosition()));
	ProcessInput();
}

void Script_CameraTarget::ProcessInput()
{
	float moveSpeed = mMoveSpeed;
	if (KEY_PRESSED(VK_SHIFT)) {
		moveSpeed *= 3.f;
	}

	if (KEY_PRESSED('W')) { 
		mObject->Translate(Vec3{ 0.f, 0.f, moveSpeed * DeltaTime() });
	}
	if (KEY_PRESSED('S')) {
		mObject->Translate(Vec3{ 0.f, 0.f, -moveSpeed * DeltaTime() });
	}
	if (KEY_PRESSED('A')) {
		mObject->Translate(Vec3{ -moveSpeed * DeltaTime(), 0.f, 0.f });
	}
	if (KEY_PRESSED('D')) {
		mObject->Translate(Vec3{ moveSpeed * DeltaTime(), 0.f, 0.f });
	}

	if (KEY_PRESSED('Q')) mCamera->ZoomIn();
	if (KEY_PRESSED('E')) mCamera->ZoomOut();
	if (KEY_PRESSED('I')) mCamera->ZoomReset();
}

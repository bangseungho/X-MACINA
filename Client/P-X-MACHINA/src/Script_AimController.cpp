#include "stdafx.h"
#include "Script_AimController.h"

#include "Component/UI.h"

#include "InputMgr.h"
#include "GameFramework.h"

void Script_AimController::Awake()
{
	base::Awake();

	//mUI = Canvas::I->CreateUI("Aim", Vec2(0, 0), 30, 30);

	//RESOLUTION resolution = GameFramework::I->GetWindowResolution();
	//mMaxPos.x = resolution.Width - 10.f;
	//mMaxPos.y = resolution.Height - 30.f;
}

void Script_AimController::Update()
{
	base::Update();

	//mMousePos = InputMgr::I->GetMousePos();
	//mMousePos.x = std::clamp(mMousePos.x, -mMaxPos.x, mMaxPos.x);
	//mMousePos.y = std::clamp(mMousePos.y, -mMaxPos.y, mMaxPos.y);

	//const Vec2 mouseDelta = InputMgr::I->GetMouseDelta() * mouseSensitivity;
	//mMousePos += mouseDelta;
	//mUI->SetPosition(mMousePos);
}
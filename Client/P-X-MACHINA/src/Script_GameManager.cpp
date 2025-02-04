#include "stdafx.h"
#include "Script_GameManager.h"

#include "Script_Ursacetus.h"
#include "Script_Onyscidus.h"
#include "Script_AdvancedCombatDroid_5.h"
#include "Script_MeleeBT.h"
#include "Script_MainCamera.h"
#include "Component/ParticleSystem.h"
#include "Component/Camera.h"
#include "Component/Agent.h"

#include "GameFramework.h"

#include "Scene.h"
#include "Object.h"
#include "VoxelManager.h"

void Script_GameManager::Awake()
{
	base::Awake();
	VoxelManager::I->SetAgent(Scene::I->Instantiate("EliteTrooper")->AddComponent<Agent>().get());
}

void Script_GameManager::Start()
{
	base::Start();

	mMainCamera = MainCamera::I->GetComponent<Script_MainCamera>();
}

void Script_GameManager::Update()
{
	base::Update();
}

void Script_GameManager::InitObjectScripts()
{

}

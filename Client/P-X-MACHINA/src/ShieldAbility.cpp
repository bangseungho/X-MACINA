#include "stdafx.h"
#include "ShieldAbility.h"

#include "Scene.h"
#include "GameFramework.h"

#include "Mesh.h"
#include "Shader.h"
#include "Object.h"
#include "ResourceMgr.h"

#include "Script_LiveObject.h"

ShieldAbility::ShieldAbility(float sheild)
	:
	RenderedAbility(2.f, 4.5f),
	mShield(sheild)
{
	mLayer = 1;
	mAbilityCB.Duration = 4.f;

	mRenderedObject = std::make_shared<GameObject>();
	mRenderedObject->SetModel("Shield");
	
	mShader = RESOURCE<Shader>("ShieldAbility");
}

void ShieldAbility::Update(float activeTime)
{
	base::Update(activeTime);

	const Vec3 playerPos = mObject->GetPosition() + Vec3{ 0.f, 0.85f, 0.f };
	mRenderedObject->SetPosition(playerPos);
}

void ShieldAbility::Activate()
{
	base::Activate();

	mObject->GetComponent<Script_LiveObject>()->SetShield(mShield);
}

void ShieldAbility::DeActivate()
{
	base::DeActivate();

	mObject->GetComponent<Script_LiveObject>()->SetShield(0.f);
}

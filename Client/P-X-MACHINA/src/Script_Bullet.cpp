#include "stdafx.h"
#include "Script_Bullet.h"

#include "Script_Enemy.h"
#include "Script_Weapon.h"

#include "Object.h"
#include "Timer.h"
#include "Scene.h"

#include "Component/Rigidbody.h"

void Script_Bullet::Update()
{
	mCurrLifeTime += DeltaTime();

	if (mCurrLifeTime >= mMaxLifeTime) {
		Reset();
		mGameObject->Return();
	}
	else if ((mObject->GetPosition().y < 0.f) || IntersectTerrain()) {
		Explode();
	}

}

void Script_Bullet::OnCollisionStay(Object& other)
{
	if (mCurrLifeTime <= FLT_EPSILON) {
		return;
	}
	if (IsOwner(&other)) {
		return;
	}

	switch (other.GetTag()) {
	case ObjectTag::Building:
	case ObjectTag::DissolveBuilding:
		Explode();
		break;

	case ObjectTag::Enemy:
	{
		auto enemy = other.GetComponent<Script_Enemy>();
		enemy->Hit(GetDamage());
		Explode();
	}

	break;
	default:
		break;
	}
}


void Script_Bullet::Init()
{
	mGameObject = mObject->GetObj<InstObject>();

	mRigid = mObject->GetComponent<Rigidbody>();
	mRigid->SetFriction(0.001f);
	mRigid->SetDrag(0.001f);

	Reset();
}

void Script_Bullet::Fire(const Vec3& pos, const Vec3& dir, const Vec3& up)
{
	mObject->SetPosition(pos);

	mRigid->Stop();
	mRigid->AddForce(dir, mSpeed, ForceMode::Impulse);

	SetDamage(mDamage);
	StartFire();
}

void Script_Bullet::Fire(const Transform& transform, const Vec2& err)
{
	mObject->SetLocalRotation(transform.GetRotation());
	Vec3 dir = transform.GetLook();
	dir = Vector3::Rotate(dir, err.y, err.x, 0.f);
	mObject->Rotate(err.y, err.x, 0.f);
	Fire(transform.GetPosition(), dir, transform.GetUp());
}

void Script_Bullet::Explode()
{
	Reset();

	mRigid->Stop();
	mGameObject->Return();
}

void Script_Bullet::Reset()
{
	mCurrLifeTime = 0.f;
}

bool Script_Bullet::IntersectTerrain()
{
	const Vec3 pos = mObject->GetPosition();
	const float terrainHeight = Scene::I->GetTerrainHeight(pos.x, pos.z);

	if (pos.y <= terrainHeight) {
		return true;
	}

	return false;
}
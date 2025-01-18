#pragma once


#pragma region Include
#include "Component/Component.h"
#pragma endregion


#pragma region ClassForwardDecl
class Script_MainCamera;
#pragma endregion


#pragma region Class
class Script_CameraTarget : public Component {
	COMPONENT(Script_CameraTarget, Component)

private:
	static constexpr float mMoveSpeed = 10;
	Script_MainCamera* mCamera{};

public:
	void Start() override;
	void Update() override;
	
private:
	void ProcessInput();
};
#pragma endregion
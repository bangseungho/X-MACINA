#pragma once

#pragma region Include
#include "Component/Component.h"
#pragma endregion


#pragma region Define
#pragma endregion


#pragma region Using
using namespace Path;
#pragma endregion


#pragma region ClassForwardDecl
#pragma endregion


#pragma region Struct
struct PQNode {
	bool operator<(const PQNode& rhs) const { return F < rhs.F; }
	bool operator>(const PQNode& rhs) const { return F > rhs.F; }

	int	F;
	int	G;
	Pos	Pos;
};
#pragma endregion


#pragma region Class

class Agent : public Component {
	COMPONENT(Agent, Component)

private:
	static constexpr int mkWeight = 10;
	
private:
	std::stack<Vec3> mPath{};
	float mWalkSpeed = 3.2f;

public:
	virtual void Update() override;

public:
	void PathPlanningToAstar(Pos dest);

private:
	void MoveToPath();
};
#pragma endregion
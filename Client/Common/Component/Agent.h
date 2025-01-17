#pragma once

#pragma region Include
#include "Component/Component.h"
#pragma endregion


#pragma region Define
#pragma endregion


#pragma region Using
#pragma endregion


#pragma region ClassForwardDecl
#pragma endregion


#pragma region Struct
struct PQNode {
	bool operator<(const PQNode& rhs) const { return F < rhs.F; }
	bool operator>(const PQNode& rhs) const { return F > rhs.F; }
	int	F{};
	int	G{};
	Pos	Pos{};
};
#pragma endregion


#pragma region Class
class PathOption : public Singleton<PathOption> {
	friend Singleton;
	
private:
	float mAgentSpeed = 4.2f;
	int mAllowedHeight = 1;
	int mMaxClosedListSize = 50000;

public:
	float GetAgentSpeed() const { return mAgentSpeed; }
	int GetAllowedHeight() const { return mAllowedHeight; }
	int GetMaxClosedListSize() const { return mMaxClosedListSize; }
	
	void SetAgentSpeed(float speed) { mAgentSpeed = speed; }
	void SetAllowedHeight(int height) { mAllowedHeight = height; }
	void SetMaxClosedListSize(int size) { mMaxClosedListSize = size; }
};


class Agent : public Component {
	COMPONENT(Agent, Component)

private:
	static constexpr int mkWeight = 10;
	
private:
	std::stack<Vec3> mPath{};
	Pos mStart{};

public:
	virtual void Update() override;

public:
	void PathPlanningToAstar(const Pos& dest);
	void ReadyPlanningToPath(const Pos& start);

private:
	void MoveToPath();
	int GetOnVoxelCount(const Pos& pos);
	void ClearPath() { while (!mPath.empty()) mPath.pop(); }
};
#pragma endregion
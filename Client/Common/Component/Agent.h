#pragma once

#pragma region Include
#include "Component/Component.h"
#pragma endregion


#pragma region Enum
enum class Heuristic {
	Manhattan = 0,
	Euclidean,
};
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
	float		mAgentSpeed = 5.2f;
	int			mAllowedHeight = 1;
	int			mMaxClosedListSize = 50000;
	int			mOnVoxelCost = 30;
	int			mHeuristicWeight = 10;
	int			mProximityWeight = 10;
	bool		mPathSmoothing = false;
	Heuristic	mHeuristic = Heuristic::Manhattan;

public:
	float		GetAgentSpeed() const { return mAgentSpeed; }
	int			GetAllowedHeight() const { return mAllowedHeight; }
	int			GetMaxClosedListSize() const { return mMaxClosedListSize; }
	int			GetOnVoxelCost() const { return mOnVoxelCost; }
	int			GetHeuristicWeight() const { return mHeuristicWeight; }
	int			GetProximityWeight() const { return mProximityWeight; }
	bool		GetPathSmoothing() const { return mPathSmoothing; }
	Heuristic	GetHeuristic() const { return mHeuristic; }
	
	void		SetAgentSpeed(float speed) { mAgentSpeed = speed; }
	void		SetAllowedHeight(int height) { mAllowedHeight = height; }
	void		SetMaxClosedListSize(int size) { mMaxClosedListSize = size; }
	void		SetOnVoxelCost(int cost) { mOnVoxelCost = cost; }
	void		SetHeuristicWeight(int weight) { mHeuristicWeight = weight; }
	void		SetProximityWeight(int weight) { mProximityWeight = weight; }
	void		SetPathSmoothing(bool smoothing) { mPathSmoothing = smoothing; }
	void		SetHeuristic(Heuristic heuristic) { mHeuristic = heuristic; }
};


class Agent : public Component {
	COMPONENT(Agent, Component)

private:
	
private:
	std::vector<Vec3> mPath{};
	std::vector<Vec3> mSplinePath{};
	Pos mStart{};

public:
	virtual void Update() override;

public:
	bool	PathPlanningToAstar(const Pos& dest);
	void	ReadyPlanningToPath(const Pos& start);

private:
	void	PathOptimize();
	void	MoveToPath();
	int		GetOnVoxelCount(const Pos& pos);
	void	ClearPath() { mPath.clear(); mSplinePath.clear(); /*while (!mPath.empty()) mPath.pop();*/ }
};
#pragma endregion
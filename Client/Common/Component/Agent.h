#pragma once

#pragma region Include
#include "Component/Component.h"
#pragma endregion


#pragma region Enum
enum class Heuristic : UINT8 {
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
	float		mAgentSpeed = 3.2f;
	int			mAllowedHeight = 1;
	int			mMaxOpenNodeCount = 50000;
	int			mOnVoxelCost = 30;
	int			mHeuristicWeight = 10;
	int			mProximityWeight = 10;
	bool		mDirPathOptimize = false;
	bool		mRayPathOptimize = false;
	bool		mSplinePath = false;
	Heuristic	mHeuristic = Heuristic::Manhattan;

public:
	float		GetAgentSpeed() const { return mAgentSpeed; }
	int			GetAllowedHeight() const { return mAllowedHeight; }
	int			GetMaxOpenNodeCount() const { return mMaxOpenNodeCount; }
	int			GetOnVoxelCost() const { return mOnVoxelCost; }
	int			GetHeuristicWeight() const { return mHeuristicWeight; }
	int			GetProximityWeight() const { return mProximityWeight; }
	bool		GetDirPathOptimize() const { return mDirPathOptimize; }
	bool		GetRayPathOptimize() const { return mRayPathOptimize; }
	bool		GetSplinePath() const { return mSplinePath; }
	Heuristic	GetHeuristic() const { return mHeuristic; }
	
	void		SetAgentSpeed(float speed) { mAgentSpeed = speed; }
	void		SetAllowedHeight(int height) { mAllowedHeight = height; }
	void		SetMaxOpenNodeCount(int count) { mMaxOpenNodeCount = count; }
	void		SetOnVoxelCost(int cost) { mOnVoxelCost = cost; }
	void		SetHeuristicWeight(int weight) { mHeuristicWeight = weight; }
	void		SetProximityWeight(int weight) { mProximityWeight = weight; }
	void		SetDirPathOptimize(bool optimize) { mDirPathOptimize = optimize; }
	void		SetRayPathOptimize(bool optimize) { mRayPathOptimize = optimize; if (optimize) SetDirPathOptimize(optimize); }
	void		SetSplinePath(bool spline) { mSplinePath = spline; }
	void		SetHeuristic(Heuristic heuristic) { mHeuristic = heuristic; }
};


class Agent : public Component {
	COMPONENT(Agent, Component)

private:
	std::vector<Vec3>	mFinalPath{};
	Pos					mStart{};

public:
	virtual void Update() override;

public:
	bool	PathPlanningToAstar(const Pos& dest);
	void	ReadyPlanningToPath(const Pos& start);

private:
	void	RayPathOptimize(std::stack<Pos>& path, const Pos& dest);
	void	MakeSplinePath();

private:
	void	MoveToPath();
	int		GetOnVoxelCount(const Pos& pos);
	void	ClearPath();
};
#pragma endregion
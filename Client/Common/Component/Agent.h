#pragma once

#pragma region Include
#include "Component/Component.h"
#include <Imgui/ImguiCode/imgui.h>
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
	float F{};
	float G{};
	Pos	Pos{};
};
#pragma endregion


#pragma region Class
class PathOption : public Singleton<PathOption> {
	friend Singleton;
	
private:
	float		mAgentSpeed = 2.2f;
	int			mAllowedHeight = 0;
	int			mMaxOpenNodeCount = 50000;
	int			mOnVoxelCost = 30;
	int			mHeuristicWeight = 10;
	int			mProximityWeight = 10;
	int			mEdgeWeight = 10;
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
	int			GetEdgeWeight() const { return mEdgeWeight; }
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
	void		SetEdgeWeight(int weight) { mEdgeWeight = weight; }
	void		SetDirPathOptimize(bool optimize) { mDirPathOptimize = optimize; }
	void		SetRayPathOptimize(bool optimize) { mRayPathOptimize = optimize; if (optimize) SetDirPathOptimize(optimize); }
	void		SetSplinePath(bool spline) { mSplinePath = spline; }
	void		SetHeuristic(Heuristic heuristic) { mHeuristic = heuristic; }
};


class Agent : public Component {
	COMPONENT(Agent, Component)

public:
	static constexpr int mkAvoidPathCount = 3;

private:
	std::vector<Vec3>	mGlobalPath{};
	std::vector<Vec3>	mLocalPath{};
	std::unordered_map<Pos, int> mGlobalPathCache{};

	Pos					mStart{};
	Pos					mDest{};

	std::vector<Pos>	mCloseList{};
	std::vector<Pos>	mOpenList{};
	Pos					mFirstCollisionVoxel{};


public:
	virtual void Start() override;
	virtual void Update() override;

public:
	std::vector<Vec3>	PathPlanningToAstar(const Pos& dest, bool clearPathList = true);
	void				ReadyPlanningToPath(const Pos& start);
	void				SetPath(std::vector<Vec3>& path) { mGlobalPath = path; }
	bool				PickAgent();
	void				RenderOpenList();
	void				RenderCloseList();
	void				ClearPathList();
	void				SetRimFactor(float factor) { mObject->mObjectCB.RimFactor = factor; }

private:
	void	RayPathOptimize(std::stack<Pos>& path, const Pos& dest);
	void	MakeSplinePath(std::vector<Vec3>& path);

private:
	void	MoveToPath();
	void	AvoidStaticVoxel(const Pos& crntPathIndex);
	int		GetOnVoxelCount(const Pos& pos);
	float	GetEdgeCost(const Pos& nextPos, const Pos& dir);
	void	ClearPath();
};


class AgentManager : public Singleton<AgentManager> {
	friend Singleton;

private:
	std::set<Agent*> mAgents{};

public:
	void AddAgent(Agent* agent) { mAgents.insert(agent); }
	void RemoveAgent(Agent* agent) { mAgents.erase(agent); }

public:
	void RenderPathList();
	void ClearPathList();

public:
	void PickAgent(Agent** agent);
};

#pragma endregion
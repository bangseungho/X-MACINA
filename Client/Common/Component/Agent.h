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
	int			mMaxOpenNodeCount = 50000;
	int			mOnVoxelCost = 30;
	int			mHeuristicWeight = 10;
	int			mProximityWeight = 10;
	int			mEdgeWeight = 10;
	bool		mDirPathOptimize = false;
	bool		mRayPathOptimize = false;
	bool		mSplinePath = false;
	bool		mStartFlag = false;
	bool		mMoveRandom = false;

public:

	int			GetMaxOpenNodeCount() const { return mMaxOpenNodeCount; }
	int			GetOnVoxelCost() const { return mOnVoxelCost; }
	int			GetHeuristicWeight() const { return mHeuristicWeight; }
	int			GetProximityWeight() const { return mProximityWeight; }
	int			GetEdgeWeight() const { return mEdgeWeight; }
	bool		GetDirPathOptimize() const { return mDirPathOptimize; }
	bool		GetRayPathOptimize() const { return mRayPathOptimize; }
	bool		GetSplinePath() const { return mSplinePath; }
	bool		GetStartFlag() const { return mStartFlag; }
	bool		GetMoveRandom() const { return mMoveRandom; }

	void		SetMaxOpenNodeCount(int count) { mMaxOpenNodeCount = count; }
	void		SetOnVoxelCost(int cost) { mOnVoxelCost = cost; }
	void		SetHeuristicWeight(int weight) { mHeuristicWeight = weight; }
	void		SetProximityWeight(int weight) { mProximityWeight = weight; }
	void		SetEdgeWeight(int weight) { mEdgeWeight = weight; }
	void		SetDirPathOptimize(bool optimize) { mDirPathOptimize = optimize; }
	void		SetRayPathOptimize(bool optimize) { mRayPathOptimize = optimize; if (optimize) SetDirPathOptimize(optimize); }
	void		SetSplinePath(bool spline) { mSplinePath = spline; }
	void		SetStartFlag(bool flag) { mStartFlag = flag; }
	void		SetMoveRandom(bool flag) { mMoveRandom = flag; }
};


struct AgentOption {
	float		AgentSpeed = 1.2f;
	int			AllowedHeight = 0;
	Heuristic	Heuri = Heuristic::Manhattan;
};


class Agent : public Component {
	COMPONENT(Agent, Component)

public:
	AgentOption mOption{};

private:
	std::vector<Vec3>	mGlobalPath{};
	std::vector<Vec3>	mLocalPath{};
	std::unordered_map<Pos, int> mGlobalPathCache{};
	std::unordered_map<Pos, int> mOpenListMinusCost{};
	std::unordered_map<Pos, int> mPrevPathMinusCost{};

	Pos					mVoxelIndex{};
	Pos					mStart{};
	Pos					mDest{};
	Vec3				mPathDir{};

	std::vector<Pos>	mCloseList{};
	std::vector<Pos>	mOpenList{};
	
	bool				mIsStart{};
	int					mSlowSpeedCount{};
	float				mAngleSpeedRatio{};
	int					mAgentID{};

private:
	static constexpr int mkAvoidForwardStaticObjectCount = 3;

public:
	virtual void Start() override;
	virtual void Update() override;
	
public:
	const Pos		GetPathIndex(int index) const;
	const Matrix	GetWorldMatrix() const { return mObject->GetWorldTransform(); }
	const Vec3		GetWorldPosition() const { return mObject->GetPosition(); }
	const Vec3		GetPathDirection() const { return mPathDir; }
	const Pos		GetVoxelIndex() const { return mVoxelIndex; }
	const bool		IsStart() const { return mIsStart; }

public:
	void SetWorldMatrix(const Matrix& mtxWorld) { return mObject->SetWorldTransform(mtxWorld); }
	void SetStartMoveToPath(bool isStart) { mIsStart = isStart; }
	void SetAngleSpeedRatio(float ratio) { mAngleSpeedRatio = ratio; }
	void SetAgentID(int id) { mAgentID = id; }

public:
	std::vector<Vec3>	PathPlanningToAstar(const Pos& dest, std::unordered_map<Pos, int> avoidCostMap, bool clearPathList = true, bool inputDest = true, bool avoidAgents = false);
	void				ReadyPlanningToPath(const Pos& start);
	void				SetPath(std::vector<Vec3>& path) { mGlobalPath = path; }
	bool				PickAgent();
	void				RenderOpenList();
	void				RenderCloseList();
	void				ClearPathList();
	void				SetRimFactor(float factor) { mObject->mObjectCB.RimFactor = factor; }

private:
	bool	CheckCurNodeContainPathCache(const Pos& curNode);
	void	RayPathOptimize(std::stack<Pos>& path, const Pos& dest);
	void	MakeSplinePath(std::vector<Vec3>& path);

private:
	void	MoveToPath();
	void	RePlanningToPathAvoidStatic(const Pos& crntPathIndex);
	void	RePlanningToPathAvoidDynamic();
	float	GetEdgeCost(const Pos& nextPos, const Pos& dir);
	void	ClearPath();
};


class AgentManager : public Singleton<AgentManager> {
	friend Singleton;

private:
	int mAgentIDs{};
	std::set<Agent*> mAgents{};
	bool mFinishAllAgentMoveToPath{};

public:
	const bool IsFinishAllAgentMoveToPath() const { return mFinishAllAgentMoveToPath; }

public:
	void AddAgent(Agent* agent) { agent->SetAgentID(++mAgentIDs); mAgents.insert(agent); }
	void RemoveAgent(Agent* agent) { mAgents.erase(agent); }
	void Update();

public:
	int CheckOtherAgent(const Pos& index, Agent* invoker);
	std::unordered_set<Pos> GetOtherAgentIndices(Agent* invoker);
	void StartMoveToPath();
	void RenderPathList();
	void ClearPathList();
	std::unordered_map<Pos, int> CheckAgentIndex(const Pos& index, Agent* invoker);
	void PickAgent(Agent** agent);

public:
	Pos RandomDest(int x, int z);
	void ShuffleMoveToPath();
};

#pragma endregion
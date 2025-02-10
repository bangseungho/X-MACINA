#pragma once

#pragma region ClassForwardDecl
class Agent;
#pragma endregion

struct AgentTreeNode {
public:
	int Begin{};
	int End{};
	int Left{};
	int Right{};
	Vec3 MaxCoord{};
	Vec3 MinCoord{};
};

class KdTree {
private:
	friend class AgentManager;
	friend class Agent;

	std::vector<Agent*> mAgents{};
	std::vector<AgentTreeNode> mAgentTree{};

public:
	KdTree() {}
	~KdTree() {}

private:
	void BuildAgentTree();
	void BuildAgentTreeRecursive(int Begin, int End, int Node);
	void ComputeAgentNeighbors(Agent* agent, float rangeSq) const;
	void QueryAgentTreeRecursive(Agent* agent, float& rangeSq, int Node) const;
};


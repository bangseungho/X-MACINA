#include "EnginePch.h"
#include "KdTree.h"

#include "Component/Agent.h"

namespace {
	const int gkKdTreeMaxLeafSize = 10;
}

void KdTree::BuildAgentTree()
{
	mAgents = AgentManager::I->mAgents;

	if (!mAgents.empty()) {
		mAgentTree.resize(2 * mAgents.size() - 1);
		BuildAgentTreeRecursive(0, mAgents.size(), 0);
	}
}

void KdTree::BuildAgentTreeRecursive(int Begin, int End, int Node)
{

	mAgentTree[Node].Begin = Begin;
	mAgentTree[Node].End = End;
	mAgentTree[Node].MinCoord = mAgents[Begin]->GetWorldPosition();
	mAgentTree[Node].MaxCoord = mAgents[Begin]->GetWorldPosition();


	for (int i = Begin + 1; i < End; ++i) {
		const Vec3& agentPosition = mAgents[i]->GetWorldPosition();
		mAgentTree[Node].MaxCoord.x = max(mAgentTree[Node].MaxCoord.x, agentPosition.x);
		mAgentTree[Node].MinCoord.x = min(mAgentTree[Node].MinCoord.x, agentPosition.x);

		mAgentTree[Node].MaxCoord.y = max(mAgentTree[Node].MaxCoord.y, agentPosition.y);
		mAgentTree[Node].MinCoord.y = min(mAgentTree[Node].MinCoord.y, agentPosition.y);

		mAgentTree[Node].MaxCoord.z = max(mAgentTree[Node].MaxCoord.z, agentPosition.z);
		mAgentTree[Node].MinCoord.z = min(mAgentTree[Node].MinCoord.z, agentPosition.z);
	}

	if (End - Begin > gkKdTreeMaxLeafSize) {
		int coord = 0;

		if (mAgentTree[Node].MaxCoord.x - mAgentTree[Node].MinCoord.x >
			mAgentTree[Node].MaxCoord.y - mAgentTree[Node].MinCoord.y &&
			mAgentTree[Node].MaxCoord.x - mAgentTree[Node].MinCoord.x >
			mAgentTree[Node].MaxCoord.z - mAgentTree[Node].MinCoord.z) {
			coord = 0;
		}
		else if (mAgentTree[Node].MaxCoord.y - mAgentTree[Node].MinCoord.y >
			mAgentTree[Node].MaxCoord.z - mAgentTree[Node].MinCoord.z) {
			coord = 1;
		}
		else {
			coord = 2;
		}

		const float splitValue = 0.5f * (mAgentTree[Node].MaxCoord[coord] + mAgentTree[Node].MinCoord[coord]);

		int Left = Begin;
		int Right = End;

		while (Left < Right) {
			while (Left < Right && mAgents[Left]->GetWorldPosition()[coord] < splitValue) {
				++Left;
			}

			while (Right > Left && mAgents[Right - 1]->GetWorldPosition()[coord] >= splitValue) {
				--Right;
			}

			if (Left < Right) {
				std::swap(mAgents[Left], mAgents[Right - 1]);
				++Left;
				--Right;
			}
		}

		int LeftSize = Left - Begin;
		if (LeftSize == 0) {
			++LeftSize;
			++Left;
		}

		mAgentTree[Node].Left = Node + 1;
		mAgentTree[Node].Right = Node + 2 * LeftSize;

		BuildAgentTreeRecursive(Begin, Left, mAgentTree[Node].Left);
		BuildAgentTreeRecursive(Left, End, mAgentTree[Node].Right);
	}
}

void KdTree::ComputeAgentNeighbors(Agent* agent, float rangeSq) const
{
	QueryAgentTreeRecursive(agent, rangeSq, 0);
}

void KdTree::QueryAgentTreeRecursive(Agent* agent, float& rangeSq, int Node) const
{
	Vec3 agentPosition = agent->GetWorldPosition();
	if (mAgentTree[Node].End - mAgentTree[Node].Begin <= gkKdTreeMaxLeafSize) {
		for (int i = mAgentTree[Node].Begin; i < mAgentTree[Node].End; ++i) {
			agent->InsertAgentNeightbor(mAgents[i], rangeSq);
		}
	}
	else {
		const float distSqLeftMinX = max(0.f, mAgentTree[mAgentTree[Node].Left].MinCoord.x - agentPosition.x);
		const float distSqLeftMaxX = max(0.f, agentPosition.x - mAgentTree[mAgentTree[Node].Left].MaxCoord.x);

		const float distSqLeftMinY = max(0.f, mAgentTree[mAgentTree[Node].Left].MinCoord.y - agentPosition.y);
		const float distSqLeftMaxY = max(0.f, agentPosition.y - mAgentTree[mAgentTree[Node].Left].MaxCoord.y);

		const float distSqLeftMinZ = max(0.f, mAgentTree[mAgentTree[Node].Left].MinCoord.z - agentPosition.z);
		const float distSqLeftMaxZ = max(0.f, agentPosition.z - mAgentTree[mAgentTree[Node].Left].MaxCoord.z);

		const float distSqLeft =
			distSqLeftMinX * distSqLeftMinX + distSqLeftMaxX * distSqLeftMaxX +
			distSqLeftMinY * distSqLeftMinY + distSqLeftMaxY * distSqLeftMaxY +
			distSqLeftMinZ * distSqLeftMinZ + distSqLeftMaxZ * distSqLeftMaxZ;

		const float distSqRightMinX = max(0.f, mAgentTree[mAgentTree[Node].Right].MinCoord.x - agentPosition.x);
		const float distSqRightMaxX = max(0.f, agentPosition.x - mAgentTree[mAgentTree[Node].Right].MaxCoord.x);

		const float distSqRightMinY = max(0.f, mAgentTree[mAgentTree[Node].Right].MinCoord.y - agentPosition.y);
		const float distSqRightMaxY = max(0.f, agentPosition.y - mAgentTree[mAgentTree[Node].Right].MaxCoord.y);

		const float distSqRightMinZ = max(0.f, mAgentTree[mAgentTree[Node].Right].MinCoord.z - agentPosition.z);
		const float distSqRightMaxZ = max(0.f, agentPosition.z - mAgentTree[mAgentTree[Node].Right].MaxCoord.z);

		const float distSqRight =
			distSqRightMinX * distSqRightMinX + distSqRightMaxX * distSqRightMaxX +
			distSqRightMinY * distSqRightMinY + distSqRightMaxY * distSqRightMaxY +
			distSqRightMinZ * distSqRightMinZ + distSqRightMaxZ * distSqRightMaxZ;

		if (distSqLeft < distSqRight) {
			if (distSqLeft < rangeSq) {
				QueryAgentTreeRecursive(agent, rangeSq, mAgentTree[Node].Left);

				if (distSqRight < rangeSq) {
					QueryAgentTreeRecursive(agent, rangeSq, mAgentTree[Node].Right);
				}
			}
		}
		else {
			if (distSqRight < rangeSq) {
				QueryAgentTreeRecursive(agent, rangeSq, mAgentTree[Node].Right);

				if (distSqLeft < rangeSq) {
					QueryAgentTreeRecursive(agent, rangeSq, mAgentTree[Node].Left);
				}
			}
		}
	}
}

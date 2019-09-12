#pragma once

#include <fstream>

#include "Config.h"
#include "Navigation/Obstacle.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	// FORWARD DECLARATIONS
	class NavMeshNode;
	class NavMesh;

	class FUSION_CROWD_API NavMeshObstacle : public Obstacle
	{
	public:
		static size_t NO_NEIGHBOR_OBST;

		NavMeshObstacle() : Obstacle(), _node(0x0)
		{
		}

		bool LoadFromAscii(std::ifstream& f, int id, DirectX::SimpleMath::Vector2* vertices);
		bool SetObstacle(int v0, int v1, int n, int nextObst, DirectX::SimpleMath::Vector2* vertices);
		inline const NavMeshNode* getNode() const { return _node; }

		~NavMeshObstacle();

		friend class NavMeshNode;
		friend class NavMesh;
	protected:
		NavMeshNode* _node;

		int _idObstacle;
		int _v0;
		int _v1;

		int _n;
	};
}

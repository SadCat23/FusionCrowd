#pragma once
#include "../../Config.h"
#include "../../NavComponents/Obstacle.h"
#include "../../Agent.h"
#include "../../NavComponents/NavMesh/NavMeshLocalizer.h"
#include "../../Math/vector.h"
#include "ProximityQuery.h"

#include <vector>

namespace FusionCrowd
{
	class FUSION_CROWD_API NavMeshSpatialQuery : public SpatialQuery
	{
	public:
		NavMeshSpatialQuery();
		virtual void SetAgents(const std::vector<Agent*> & agents);
		virtual void UpdateAgents() {} // ?
		virtual void AgentQuery(ProximityQuery *query) const;
		virtual void AgentQuery(ProximityQuery *query, float &rangeSq) const;
		virtual void ProcessObstacles();
		virtual void ObstacleQuery(ProximityQuery *query) const;
		virtual void ObstacleQuery(ProximityQuery *query, float rangeSq) const;
		virtual bool QueryVisibility(const Math::Vector2& q1, const Math::Vector2& q2, float radius) const;
		void SetNavMeshLocalizer(const NavMeshLocalizerPtr & nml) { _localizer = nml; }
		//virtual BFSM::Task * getTask();

		std::vector< Agent * > _agents;
		NavMeshLocalizerPtr _localizer;
	};
}


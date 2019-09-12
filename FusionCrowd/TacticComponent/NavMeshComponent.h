#pragma once

#include <string>

#include "ComponentId.h"
#include "ITacticComponent.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "StrategyComponent/Goal/Goal.h"
#include "Agent.h"
#include "Simulator.h"

namespace FusionCrowd
{
	class Simulator;
	class AgentSpatialInfo;

	class NavMeshComponent : public ITacticComponent
	{
	public:
		NavMeshComponent(std::shared_ptr<Simulator> simulator, std::shared_ptr<NavMeshLocalizer> localizer);

		void AddAgent(size_t id) override;
		bool DeleteAgent(size_t id) override;

		std::shared_ptr<NavMesh> GetNavMesh() const { return _localizer->getNavMesh(); };
		std::shared_ptr<NavMeshLocalizer> GetLocalizer() const;

		void Update(float timeStep) override;
		unsigned int getNodeId(size_t agentId) const;
		unsigned int getNodeId(size_t agentId, const std::string& grpName, bool searchAll = false);

		ComponentId GetId() override { return ComponentIds::NAVMESH_ID; }

		~NavMeshComponent();

	private:
		struct AgentStruct
		{
		public:
			unsigned int id;
			NavMeshLocation location;
		};

		void setPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct);
		unsigned int updateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, bool force) const;

		/*
		PortalPath * getPath(size_t id);
		void setPath(size_t agentID, PortalPath * path);
		void clearPath(size_t agentID);
		void setNode(size_t agentID, unsigned int nodeID);
		*/

		std::shared_ptr<Simulator> _simulator;

		float _headingDevCos;
		std::shared_ptr<NavMesh> _navMesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;
		std::vector<AgentStruct> _agents;
	};
}


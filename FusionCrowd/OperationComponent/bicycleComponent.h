#pragma once

#include "Agent.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"
#include "Export/ComponentId.h"
#include "Math/Line.h"

#include <map>

namespace FusionCrowd
{
	namespace Bicycle

	{

		


		struct AgentParamentrs
		{
			float _mass;
			float _theta;
			float _delta;
			float _length;
			float _orintX;
			float _orintY;
			float _acceleration;
			float _negativeAcceleration;
			bool _LookAt;
			DirectX::SimpleMath::Vector2 _curentTarget;

			AgentParamentrs() :_mass(80.0f), _theta(0.0f),_delta(0.0f),_length(2.0f),_orintX(1.0f),_orintY(0.0f), _acceleration(0.5f), _negativeAcceleration(0.5f), _LookAt(false), _curentTarget(DirectX::SimpleMath::Vector2(0.0f,0.0f))
			{
				
			}
			AgentParamentrs(float mass) : _mass(mass)
			{
			}
		};

		class BicycleComponent : public IOperationComponent
		{
		public:
			BicycleComponent(std::shared_ptr<NavSystem> navSystem);
			~BicycleComponent();
			float _timeHorizonObst = 0.15f;
			float _timeHorizon = 2.5f; 
			ComponentId GetId() override { return ComponentIds::BICYCLE; };

			void AddAgent(size_t id) override;
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id) override;
			void Update(float timeStep) override;
			size_t ComputeORCALines(std::vector<FusionCrowd::Math::Line>& _orcaLines, size_t agentId, float timeStep);
			void ObstacleLine(std::vector<FusionCrowd::Math::Line>& _orcaLines, Obstacle & obst, const float invTau, bool flip, size_t agentId);

		private:
			void ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep);



			std::shared_ptr<NavSystem> _navSystem;
			std::map<int, AgentParamentrs> _agents;

			
			float _agentScale;
			float _obstScale;
			float _reactionTime;
			float _bodyForse;
			float _friction;
			float _forceDistance;
		};
	}
}
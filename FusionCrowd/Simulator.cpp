#include "Simulator.h"

#include "Navigation/NavSystem.h"
#include "Navigation/AgentSpatialInfo.h"
#include "TacticComponent/NavMeshComponent.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class FUSION_CROWD_API Simulator::SimulatorImpl
	{
	public:
		SimulatorImpl()
		{
		}

		~SimulatorImpl() = default;

		void Initialize(Simulator & simulator, const char* navMeshPath)
		{
			_navMeshTactic = std::make_shared<NavMeshComponent>(simulator, navMeshPath);

			AddTacticComponent(_navMeshTactic);

			_navSystem = std::make_shared<NavSystem>(_navMeshTactic);
		}

		bool DoStep()
		{
			const float timeStep = 0.1f;
			for (auto strategy : _strategyComponents)
			{
				strategy->Update(timeStep);
			}

			for (auto tactic : _tacticComponents)
			{
				tactic->Update(timeStep);
			}

			for (auto oper : _operComponents)
			{
				oper->Update(timeStep);
			}

			_navSystem->Update(timeStep);

			return true;
		}

		size_t GetAgentCount() const { return _agents.size(); }

		NavSystem & GetNavSystem()
		{
			return *_navSystem;
		}

		Agent & GetById(size_t agentId)
		{
			return _agents[agentId];
		}

		std::shared_ptr<Goal> GetAgentGoal(size_t agentId) {
			return _agents[agentId].currentGoal;
		}

	    size_t AddAgent(
			float maxAngleVel,
			float radius,
			float prefSpeed,
			float maxSpeed,
			float maxAccel,
			DirectX::SimpleMath::Vector2 pos,
			std::shared_ptr<Goal> goal
		)
		{
			size_t id = _agents.size();

			AgentSpatialInfo info;
			info.id = id;
			info.pos = pos;
			info.radius = radius;
			info.maxAngVel = maxAngleVel;
			info.prefSpeed = prefSpeed;
			info.maxSpeed = maxSpeed;
			info.maxAccel = maxAccel;

			_navSystem->AddAgent(info);
			Agent a(id);
			a.currentGoal = goal;
			_agents.push_back(a);

			//TEMPORARY
			_navMeshTactic->AddAgent(id);

			return id;
		}

		bool SetOperationComponent(size_t agentId, std::string newOperationComponent)
		{
			for(std::shared_ptr<IOperationComponent> c : _operComponents)
			{
				if(c->GetName() == newOperationComponent) {
					std::shared_ptr<IOperationComponent> old = _agents[agentId].opComponent;
					if(old != nullptr)
					{
						old->DeleteAgent(agentId);
					}

					c->AddAgent(agentId);
					_agents[agentId].opComponent = c;

					return true;
				}
			}
			return false;
		}
		bool SetStrategyComponent(size_t agentId, std::string newStrategyComponent)
		{
			for(std::shared_ptr<IStrategyComponent> c : _strategyComponents)
			{
				if(c->GetName() == newStrategyComponent) {
					std::shared_ptr<IStrategyComponent> old = _agents[agentId].stratComponent;
					if(old != nullptr)
					{
						old->RemoveAgent(agentId);
					}

					c->AddAgent(agentId);
					_agents[agentId].stratComponent = c;

					return true;
				}
			}
			return false;
		}

		void AddOperComponent(std::shared_ptr<IOperationComponent> operComponent)
		{
			_operComponents.push_back(operComponent);
		}

		void AddTacticComponent(std::shared_ptr<ITacticComponent> tacticComponent)
		{
			_tacticComponents.push_back(tacticComponent);
		}

		void AddStrategyComponent(std::shared_ptr<IStrategyComponent> strategyComponent)
		{
			_strategyComponents.push_back(strategyComponent);
		}

		void InitSimulator() {
			_navSystem->Init();
		}

		void UpdateNav(float x, float y)
		{
			DirectX::SimpleMath::Vector2 point1(-20.2780991, -22.8689995);
			_navMeshTactic->UpdateNavMesh(point1);
			for (int i = 0; i < _agents.size(); i++)
			{
				_navMeshTactic->AddAgent(_agents[i].id);
			}
		}

		bool IsPointInPolygonNavMesh(int idNode, float x, float y) {
			DirectX::SimpleMath::Vector2 point(x, y);
			std::shared_ptr<NavMesh>  navMesh = _navMeshTactic->GetNavMesh();
			return navMesh->IsPointInPolygon(&navMesh->GetNode(idNode), point);
		}

		int CheckObstacle(std::vector<DirectX::SimpleMath::Vector2> contour) {
			int idNode = -1;
			std::shared_ptr<NavMesh>  navMesh = _navMeshTactic->GetNavMesh();
			idNode = navMesh->CheckObstacle(contour);
			return idNode;
		}

		void GetNewNode(std::vector<DirectX::SimpleMath::Vector2> outer, std::vector<DirectX::SimpleMath::Vector2> contour,
			std::vector<int>& triangles, std::vector<DirectX::SimpleMath::Vector2>& vertex)
		{
			std::shared_ptr<NavMesh>  navMesh = _navMeshTactic->GetNavMesh();
			navMesh->TriangulateCounturAndObstale(outer, contour, triangles, vertex);
		}

		void AddNavMeshNode(std::vector<int> idNodes, std::vector<DirectX::SimpleMath::Vector2> contour)
		{
			std::shared_ptr<NavMesh>  navMesh = _navMeshTactic->GetNavMesh();

			std::vector<DirectX::SimpleMath::Vector2> outerTest;
			DirectX::SimpleMath::Vector2* v = navMesh->GetVertices();

			for (int i = 0; i < idNodes.size(); i++) {
				int  vCount = navMesh->GetNode(idNodes[i]).getVertexCount();
				for (int j = 0; j < vCount; j++) {
					outerTest.push_back(v[navMesh->GetNode(idNodes[i]).getVertexID(j)]);
				}
			}

			std::vector<DirectX::SimpleMath::Vector2> contourTest;
			contourTest.push_back(DirectX::SimpleMath::Vector2(-81.5f, 20.0f));
			contourTest.push_back(DirectX::SimpleMath::Vector2(-83.5f, 22.0f));
			contourTest.push_back(DirectX::SimpleMath::Vector2(-81.1f, 22.0f));
			contourTest.push_back(DirectX::SimpleMath::Vector2(-81.0f, 21.5f));

			//idNodes.push_back(494);
			//idNodes.push_back(495);
			//idNodes.push_back(496);
			//std::vector<int> res;
			//res = _navMesh->GetCountur(idNodes);
			//_primaryNavMesh = _navMesh;

			std::vector<int> triangles;
			std::vector<DirectX::SimpleMath::Vector2> vertex;
			navMesh->TriangulateCounturAndObstale(outerTest, contourTest, triangles, vertex);

			//std::vector<int> v12;

			NavMeshInfo::NavMeshInfo newNodes = navMesh->GetNewNode(triangles, vertex, contourTest);
			std::shared_ptr<NavMesh> m;
			std::unique_ptr<NavMesh> upt(new NavMesh("Test"));
			navMesh->AddNode(newNodes, idNodes, m);


			_navMeshTactic->SetNavMesh(m);
		}

		std::vector<int> GetIntersectionNode(std::vector<DirectX::SimpleMath::Vector2> contour)
		{
			std::vector<int> idNodes;
			std::shared_ptr<NavMesh>  navMesh = _navMeshTactic->GetNavMesh();
			idNodes = navMesh->GetListIncomingNodes(contour);
			return idNodes;
		}


		// TEMPORARY SOLUTION
		std::shared_ptr<NavMeshComponent> _navMeshTactic;
	private:

		size_t GetNextId() const { return GetAgentCount(); }

		std::shared_ptr<NavSystem> _navSystem;

		std::vector<FusionCrowd::Agent> _agents;
		std::vector<std::shared_ptr<IStrategyComponent>> _strategyComponents;
		std::vector<std::shared_ptr<ITacticComponent>> _tacticComponents;
		std::vector<std::shared_ptr<IOperationComponent>> _operComponents;
	};

	Simulator::~Simulator() = default;

	Simulator::Simulator(const char* navMeshPath)
		: pimpl(std::make_unique<SimulatorImpl>())
	{
		pimpl->Initialize(*this, navMeshPath);
	}

	bool Simulator::DoStep()
	{
		return pimpl->DoStep();
	}

	size_t Simulator::GetAgentCount() const
	{
		return pimpl->GetAgentCount();
	}

	NavSystem & Simulator::GetNavSystem()
	{
		return pimpl->GetNavSystem();
	}

	Agent & Simulator::GetById(size_t agentId)
	{
		return pimpl->GetById(agentId);
	}

	std::shared_ptr<Goal> Simulator::GetAgentGoal(size_t agentId) {
		return pimpl->GetAgentGoal(agentId);
	}

	size_t Simulator::AddAgent(float maxAngleVel, float radius, float prefSpeed, float maxSpeed, float maxAccel, Vector2 pos, std::shared_ptr<Goal> goal)
	{
		return pimpl->AddAgent(maxAngleVel, radius, prefSpeed, maxSpeed, maxAccel, pos, goal);
	}

	bool Simulator::SetOperationComponent(size_t agentId, std::string newOperationComponent)
	{
		return pimpl->SetOperationComponent(agentId, newOperationComponent);
	}

	bool Simulator::SetStrategyComponent(size_t agentId, std::string newStrategyComponent)
	{
		return pimpl->SetStrategyComponent(agentId, newStrategyComponent);
	}

	void Simulator::AddOperComponent(std::shared_ptr<IOperationComponent> component)
	{
		pimpl->AddOperComponent(component);
	}

	void Simulator::AddTacticComponent(std::shared_ptr<ITacticComponent> component)
	{
		pimpl->AddTacticComponent(component);
	}

	void Simulator::AddStrategyComponent(std::shared_ptr<IStrategyComponent> component)
	{
		pimpl->AddStrategyComponent(component);
	}

	void Simulator::InitSimulator()
	{
		pimpl->InitSimulator();
	}

	void Simulator::UpdateNav(float x, float y)
	{
		pimpl->UpdateNav(x, y);
	}

	int  Simulator::CheckObstacle(std::vector<DirectX::SimpleMath::Vector2> contour) {
		return pimpl->CheckObstacle(contour);
	}

	void Simulator::GetNewNode(std::vector<DirectX::SimpleMath::Vector2> outer, std::vector<DirectX::SimpleMath::Vector2> contour,
		std::vector<int>& triangles, std::vector<DirectX::SimpleMath::Vector2>& vertex)
	{
		pimpl->GetNewNode(outer, contour, triangles, vertex);
	}

	void Simulator::AddNavMeshNode(std::vector<int> idNodes, std::vector<DirectX::SimpleMath::Vector2> contour)
	{
		pimpl->AddNavMeshNode(idNodes, contour);
	}

	void Simulator::GetIntersectionNode(std::vector<DirectX::SimpleMath::Vector2> contour, int*& idNodes, int size)
	{
		std::vector<int> r = pimpl->GetIntersectionNode(contour);
		idNodes = new int[contour.size()];

		for (int i = 0; i < r.size(); i++) {
			idNodes[i] = r[i];
		}
	}
}

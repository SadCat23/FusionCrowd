#include "FusionCrowdLinkUE4.h"

#include "Simulator.h"

#include "Math/consts.h"
#include "Math/Util.h"

#include "StrategyComponent/Goal/GoalSet.h"
#include "StrategyComponent/Goal/Goal.h"
#include "StrategyComponent/Goal/PointGoal.h"

#include "OperationComponent/IOperationComponent.h"
#include "OperationComponent/KaramouzasComponent.h"
#include "OperationComponent/ORCAComponent.h"
#include "OperationComponent/PedVOComponent.h"
#include "OperationComponent/HelbingComponent.h"

#include "TacticComponent/NavMeshComponent.h"

#include <algorithm>
#include <memory>

using namespace DirectX::SimpleMath;

FusionCrowdLinkUE4::FusionCrowdLinkUE4(): agentsCount(0)
{
}

FusionCrowdLinkUE4::~FusionCrowdLinkUE4()
{
}

void FusionCrowdLinkUE4::StartFusionCrowd(char* navMeshDir)
{
	navMeshPath = (char*)malloc(strlen(navMeshDir) + 1);
	strcpy(navMeshPath, navMeshDir);

	sim = new FusionCrowd::Simulator(navMeshPath);

	_strategy = std::make_shared<UE4StrategyProxy>(*sim);

//	navMeshTactic = new FusionCrowd::NavMeshComponent(*sim, );
	kComponent = std::make_shared<FusionCrowd::Karamouzas::KaramouzasComponent>(*sim);
	orcaComponent = std::make_shared<FusionCrowd::ORCA::ORCAComponent>(*sim);
	pedvoComponent = std::make_shared<FusionCrowd::PedVO::PedVOComponent>(*sim);
	helbingComponent = std::make_shared<FusionCrowd::Helbing::HelbingComponent>(*sim);

	sim->AddOperComponent(kComponent);
	//sim->AddTacticComponent(*navMeshTactic);
	sim->AddOperComponent(orcaComponent);
	sim->AddOperComponent(pedvoComponent);
	sim->AddOperComponent(helbingComponent);

	sim->AddStrategyComponent(_strategy);
}

int FusionCrowdLinkUE4::GetAgentCount()
{
	return sim->GetAgentCount();
}

size_t FusionCrowdLinkUE4::AddAgent(const float * agentPos, const float * goalPos, const char * opComponent)
{
	Vector2 position(agentPos[0], agentPos[1]);
	auto goal = std::make_shared<FusionCrowd::PointGoal>(goalPos[0], goalPos[1]);
	std::string compName(opComponent);

	size_t id = sim->AddAgent(360, 0.19f, 0.3f, 0.5f, 5, position, goal);

	sim->SetOperationComponent(id, compName);
	sim->SetStrategyComponent(id, _strategy->GetName());

	return id;
}

void FusionCrowdLinkUE4::SetGoal(size_t agentId, const float * goalPos)
{
	_strategy->SetGoal(agentId, goalPos);
}

void FusionCrowdLinkUE4::AddAgents(int agentsCount)
{
	std::vector<Vector2> positions;
	positions.push_back(Vector2(-0.55f, 4.0f));
	positions.push_back(Vector2(-0.50f, -1.5f));
	positions.push_back(Vector2(-0.1f, -1.5f));
	positions.push_back(Vector2(-0.1f, -1.1f));
	positions.push_back(Vector2(-0.5f, -1.1f));
	positions.push_back(Vector2(0.3f, -1.1f));
	positions.push_back(Vector2(0.3f, -1.5f));


	auto goal = std::make_shared<FusionCrowd::PointGoal>(-3.0f, 5.0f);
	for(Vector2 pos : positions)
	{
		size_t id = sim->AddAgent(360, 0.19f, 0.05f, 0.2f, 5, pos, goal);
		sim->SetOperationComponent(id, pedvoComponent->GetName());

		/*
		if(id % 2 == 0)
			sim->SetOperationComponent(id, kComponent->GetName());
		else
			sim->SetOperationComponent(id, orcaComponent->GetName());
		*/
		//navMeshTactic->AddAgent(id);

		sim->SetStrategyComponent(id, _strategy->GetName());
	}

	sim->InitSimulator();
}

void FusionCrowdLinkUE4::SetOperationModel(size_t agentId, const char * name)
{
	sim->SetOperationComponent(agentId, std::string(name));
}


void FusionCrowdLinkUE4::GetPositionAgents(agentInfo* ueAgentInfo)
{
	FusionCrowd::NavSystem & nav = sim->GetNavSystem();
	bool info = sim->DoStep();
	agentsCount = sim->GetAgentCount();

	for (int i = 0; i < agentsCount; i++)
	{
		auto spatialInfo = nav.GetPublicSpatialInfo(i);

		ueAgentInfo[i].id = spatialInfo.id;

		ueAgentInfo[i].pos = new float[2];
		ueAgentInfo[i].pos[0] = spatialInfo.posX;
		ueAgentInfo[i].pos[1] = spatialInfo.posY;

		ueAgentInfo[i].vel = new float[2];
		ueAgentInfo[i].vel[0] = spatialInfo.velX;
		ueAgentInfo[i].vel[1] = spatialInfo.velY;

		ueAgentInfo[i].orient = new float[2];
		ueAgentInfo[i].orient[0] = spatialInfo.orientX;
		ueAgentInfo[i].orient[1] = spatialInfo.orientY;

		ueAgentInfo[i].radius = spatialInfo.radius;
		auto opComp = sim->GetById(i).opComponent;
		auto name = opComp != nullptr ? opComp->GetName() : std::string("NO_OP_COMPONENT");

		ueAgentInfo[i].opCompName = new char [name.length() + 1];
		std::strcpy (ueAgentInfo[i].opCompName, name.c_str());
	}
}

void FusionCrowdLinkUE4::UpdateNav(float x, float y)
{
	sim->UpdateNav(0, 0);
}

void FusionCrowdLinkUE4::GetTrinagles()
{

}
void FusionCrowdLinkUE4::GetNewNode(float* outer, float* contour, int outCount, int conturCount, int*& triangels, int& triangelsCount, float*& vertexs, int& vertexsCount)
{
	std::vector<Vector2> outerV;
	std::vector<Vector2> contourV;

	std::vector<int> triangel;
	std::vector<Vector2> vertex;

	for (int i = 0; i < outCount * 2; i+=2) {
		outerV.push_back(Vector2(outer[i], outer[i+1]));
	}

	for (int i = 0; i < conturCount * 2; i+=2) {
		contourV.push_back(Vector2(contour[i], contour[i + 1]));
	}

	sim->GetNewNode(outerV, contourV, triangel, vertex);

	triangelsCount = triangel.size();
	vertexsCount = vertex.size() * 2;

	triangels = new int[triangelsCount];
	vertexs = new float[vertexsCount];

	for (int i = 0; i < triangelsCount; i++) {
		triangels[i] = triangel[i];
	}

	for (int i = 0; i < vertexsCount; i+=2) {
		vertexs[i] = vertex[i/2].x;
		vertexs[i+1] = vertex[i/2].y;
	}
}

void FusionCrowdLinkUE4::AddNode(int* idNodes, int idNodesCount,float* contour, int conturCount)
{
	std::vector<Vector2> contourV;
	for (int i = 0; i < conturCount * 2; i += 2) {
		contourV.push_back(Vector2(contour[i], contour[i + 1]));
	}

	std::vector<int> idNodesV;

	for (int i = 0; i < idNodesCount; i++) {
		idNodesV.push_back(idNodes[i]);
	}

	sim->AddNavMeshNode(idNodesV,contourV);
}

void FusionCrowdLinkUE4::GetIntersectionNode(float* contour, int conturCount, int*& idNodes, int& idNodesCount)
{
	std::vector<Vector2> contourV;
	for (int i = 0; i < conturCount * 2; i += 2) {
		contourV.push_back(Vector2(contour[i], contour[i + 1]));
	}
	sim->GetIntersectionNode(contourV, idNodes, idNodesCount);
}


//int FusionCrowdLinkUE4::CheckObstacle(float** countur, int counterVertex)
//{
//
//	return 0;
//}
#include "BicycleComponent.h"

#include "Math/consts.h"
#include "Math/geomQuery.h"
#include "Math/Util.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"

#include <algorithm>
#include <list>
#include <iostream>
#include <cmath>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Bicycle
	{
		BicycleComponent::BicycleComponent(std::shared_ptr<NavSystem> navSystem) : _navSystem(navSystem)
		{
		}

		BicycleComponent::~BicycleComponent()
		{
		}

		void BicycleComponent::Update(float timeStep)
		{
			for (auto p : _agents)
			{
				auto id = p.first;
				AgentSpatialInfo & agent = _navSystem->GetSpatialInfo(id);
				ComputeNewVelocity(agent, timeStep);
			}
		}

		Vector2 rotateVector(Vector2 vector, float angle)
		{
			Vector2 rotatedVector;
			rotatedVector.x = vector.x * cos(angle) - vector.y * sin(angle);
			rotatedVector.y = vector.x * sin(angle) + vector.y * cos(angle);
			return rotatedVector;
		}

		double ToDegrees(double angel)
		{
			return angel * 180 / 3.1415926;
		}
		double ToRadian(float angel)
		{
			return angel / 180 * 3.1415926;
		}
		
		

		float LengthVector(Vector2 vect)
		{
			float l= sqrt(pow(vect.x, 2) + pow(vect.y, 2));
			
			return l;
		}

		Vector2 normalizeVector(Vector2 vect)
		{
			float inverseLength = 1 / LengthVector(vect);
			Vector2 norm = Vector2(vect.x*inverseLength, vect.y*inverseLength);
			return norm;
		}


		//������� ����������� ��������������� ���� (�� �������� ���������� ������� ������) ����� �������� ������� �������� �������� � �������� ����������� ������ 
		//��� ������� ������������ �������� ������ (���� ���� delta � ����) 
		//������� ��������� ��������������� ���� ����� ���������� ������������� ����� ����� ������ 
		//��� ��� ������� �������������� ���������������� ���� ������� 

		float PredictionAngel(Vector2 normalizePrefVel, AgentParamentrs _agentParam,float VelBike, float timeStep)
		{
			int _maxStepForPredicted = 10;
			
			float maxDeltaDeltaFromStep = 0.5;

			float theta = _agentParam._theta;
			float delta = _agentParam._delta;
			float orintY = 0, orintX = 0;
			float Angel = 0;
			
			for (int i = 0; i < _maxStepForPredicted; i++)
			{
				theta += VelBike * tan(delta) / _agentParam._length;
				orintX = cos(theta);
				orintY = sin(theta);
				
					if (delta < 0)
					{
						delta += maxDeltaDeltaFromStep* timeStep;
					}
					else if (delta > 0)
					{
						delta -= maxDeltaDeltaFromStep* timeStep;
					}
				
				Angel = (float)atan2(orintX * normalizePrefVel.y - orintY * normalizePrefVel.x, orintX * normalizePrefVel.x + orintY * normalizePrefVel.y);
				Angel = Angel * 180 / PI;
				if (abs(Angel) <= 5)
				{
					return Angel;
				}
			}
			return Angel;
			

			
		}

		void BicycleComponent::ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep)
		{
		
			
			float anglePredict=0;
			float angle=0;
			
			//������������ ���� �������� ����
			float maxDelta = 0.5f;


			float _accuracyLookAt = 10;
			float _accuracyPredictedAngle = 5;
			float _accuracyAngleRotated = 1;


			Vector2 prefVel;
			Vector2 normalizePrefVel;
			Vector2 orint;

			//������������ �������� �������� ���� 
			float maxDeltaDelta = 0.5f;
			
			AgentParamentrs _agentParam = _agents[agent.id];
			prefVel = agent.prefVelocity.getPreferredVel();

		

				//��������� ����������� �� �������� �������� 
				if (LengthVector(prefVel) != 0.0f)
				{
					//��������� ���� �� ������ �������� 
					if (LengthVector(agent.vel) != 0.0f)
					{


						normalizePrefVel = normalizeVector(prefVel);

						orint.x = agent.orient.x;
						orint.y = agent.orient.y;

						//���� ����� �������� ������� �������� �������� � ����������� ������
						angle = (float)atan2(orint.x * normalizePrefVel.y - orint.y * normalizePrefVel.x, orint.x * normalizePrefVel.x + orint.y * normalizePrefVel.y);
						angle = angle * 180 / PI;

						//���������  ���������� �� ���� ������������ �� ����
						if (_agentParam._LookAt)
						{   				
							//�������� �� ��������� �� ����
							if (_agentParam._curentTarget != agent.prefVelocity.getTarget())
							{
								_agentParam._LookAt = false;		
							}
							//���� ����� ���������� ������ ���� - ����������� ���� � ������ ���� ��� ����� �����
							if (abs(angle) < _accuracyLookAt)
							{
								_agentParam._delta = 0.0f;
								_agentParam._theta = atan2(normalizePrefVel.y, normalizePrefVel.x);					
								_agentParam._LookAt = false;
							}
							else
							{  //��������� ���� ����
								if (_agentParam._delta > 0)
								{
									_agentParam._delta -= maxDeltaDelta *timeStep;
								}
								else if (_agentParam._delta < 0)
								{
									_agentParam._delta += maxDeltaDelta *timeStep;
								}
							}
						}
						//���� ���� ���������� �� ���� �� ��������� 
						else
						{
							//�������� ������������� �������� 
							if (abs(angle) > _accuracyAngleRotated)
							{
								//��������� ��������������� ���� ����� �������� ������� �������� �������� � ������������ ������ 
								//����� ��������� �����, ��� ������� ������������ (���� �������� ���� � ����)
								anglePredict = PredictionAngel(normalizePrefVel, _agentParam, LengthVector(agent.vel), timeStep);
								
								//�������� ������������� ����������� ��������������� ���� 
								if (abs(anglePredict)< _accuracyPredictedAngle)
								{	
									//���� ������������ � ����������� ���� 
									_agentParam._LookAt = true;
									_agentParam._curentTarget = agent.prefVelocity.getTarget();
								}
								else
								{

									//�������� ������� �������� � ������������� �������� 
									if (angle > _accuracyAngleRotated)
									{	//������ ������������ ���� �������� ���� 
										float deltaDelta = atan(angle / (LengthVector(agent.vel) / _agentParam._length));
										
										//��������� ������������ ���� �������� ���� � ������������
										if (deltaDelta >= maxDeltaDelta)
										{
											if (_agentParam._delta < maxDelta)
											{
												_agentParam._delta += maxDeltaDelta *timeStep;
											}
										}
										else
										{
											if (_agentParam._delta < maxDelta)
											{
												_agentParam._delta += deltaDelta * timeStep;
											}
										}

									}
									else if (angle < -_accuracyAngleRotated)
									{
										float deltaDelta = atan(angle / (LengthVector(agent.vel) / _agentParam._length));
										if (deltaDelta <= -maxDeltaDelta)
										{
											if (_agentParam._delta > -maxDelta)
											{
												_agentParam._delta -= maxDeltaDelta * timeStep;
											}
										}
										else
										{
											if (_agentParam._delta > -maxDelta)
											{
												_agentParam._delta -= deltaDelta * timeStep;
											}
										}
									}
								}
							}
						}



						for (auto const & obst : _navSystem->GetClosestObstacles(agent.id)) {

							Vector2 nearPt;
							float sqDist;
							float SAFE_DIST2 = 0.01;
							if (obst.distanceSqToPoint(agent.pos, nearPt, sqDist) == Obstacle::LAST) continue;
							if (SAFE_DIST2 > sqDist)
							{
								_agentParam._delta = 0.0f;
								_agentParam._theta = atan2(normalizePrefVel.y, normalizePrefVel.x);				
							}
						}


						//������ ��������� ���� ����������� ���������� 
						_agentParam._theta +=(LengthVector(agent.vel) * tan(_agentParam._delta) / _agentParam._length);

						//������ �������� �������� 
						agent.velNew.x = LengthVector(agent.vel) * cos(_agentParam._theta);
						agent.velNew.y = LengthVector(agent.vel) * sin(_agentParam._theta);

						//���������� ���������� 
						_agentParam._orintX = cos(_agentParam._theta);
						_agentParam._orintY = sin(_agentParam._theta);

					}

					//������ ��������� �������� �� ����������� ���������� ���� ��� ���� �������
					else
					{
						agent.velNew.x = agent.orient.x;
						agent.velNew.y = agent.orient.y;
					}


				}

				_agents[agent.id] = _agentParam;				
			
		}
	


		void BicycleComponent::AddAgent(size_t id, float mass)
		{
			_agents[id] = AgentParamentrs();
			_navSystem->GetSpatialInfo(id).inertiaEnabled = true;
		}

		void BicycleComponent::AddAgent(size_t id)
		{
			AddAgent(id, 80.0f);
		}

		bool BicycleComponent::DeleteAgent(size_t id)
		{
			_agents.erase(id);
			return true;
		}
	}
}
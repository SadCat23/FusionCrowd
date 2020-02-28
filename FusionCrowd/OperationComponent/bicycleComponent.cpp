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


		//функция расчитывает прогнозируеммый угол (на некторое количество времени вперед) между нормалью вектора желаемой скорости и вектором направления агента 
		//При условии выравнивания рулевого колеса (угол руля delta к нулю) 
		//Функция возращает прогназируеммый угол после достижения максимального числа шагов вперед 
		//или при условии удволетворения прогназируеммого угла условию 

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
			
			//максимальный угол поворота руля
			float maxDelta = 0.5f;


			float _accuracyLookAt = 10;
			float _accuracyPredictedAngle = 5;
			float _accuracyAngleRotated = 1;


			Vector2 prefVel;
			Vector2 normalizePrefVel;
			Vector2 orint;

			//максимальная скорость поворота руля 
			float maxDeltaDelta = 0.5f;
			
			AgentParamentrs _agentParam = _agents[agent.id];
			prefVel = agent.prefVelocity.getPreferredVel();

		

				//Проверяем выставленна ли желаемая скорость 
				if (LengthVector(prefVel) != 0.0f)
				{
					//Проверяем есть ли агента скорость 
					if (LengthVector(agent.vel) != 0.0f)
					{


						normalizePrefVel = normalizeVector(prefVel);

						orint.x = agent.orient.x;
						orint.y = agent.orient.y;

						//Угол между нормалью вектора желаемой скорости и ориентацией агента
						angle = (float)atan2(orint.x * normalizePrefVel.y - orint.y * normalizePrefVel.x, orint.x * normalizePrefVel.x + orint.y * normalizePrefVel.y);
						angle = angle * 180 / PI;

						//Проверяем  установлен ли флаг выравнивания на цель
						if (_agentParam._LookAt)
						{   				
							//Проверка не сменилась ли цель
							if (_agentParam._curentTarget != agent.prefVelocity.getTarget())
							{
								_agentParam._LookAt = false;		
							}
							//Если прият достаточно точный угол - выравниваем руль и задаем угол еще более точно
							if (abs(angle) < _accuracyLookAt)
							{
								_agentParam._delta = 0.0f;
								_agentParam._theta = atan2(normalizePrefVel.y, normalizePrefVel.x);					
								_agentParam._LookAt = false;
							}
							else
							{  //Изменение угла руля
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
						//Если флаг выравнивая на цель не выставлен 
						else
						{
							//проверка необходимости поворота 
							if (abs(angle) > _accuracyAngleRotated)
							{
								//получение прогнозируемого угла между нормалью вектора желаемой скорости и направлением агента 
								//через несколько тиков, при условии выравнивания (угол поворота руля к нулю)
								anglePredict = PredictionAngel(normalizePrefVel, _agentParam, LengthVector(agent.vel), timeStep);
								
								//Проверка достаточности полученного прогназируемого угла 
								if (abs(anglePredict)< _accuracyPredictedAngle)
								{	
									//Флаг выравнивания и запоминание цели 
									_agentParam._LookAt = true;
									_agentParam._curentTarget = agent.prefVelocity.getTarget();
								}
								else
								{

									//Проверка стороны поворота и необходимости поворота 
									if (angle > _accuracyAngleRotated)
									{	//Расчет необходимого угла поворота руля 
										float deltaDelta = atan(angle / (LengthVector(agent.vel) / _agentParam._length));
										
										//Сравнение необходимого угла поворота руля с максимальным
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


						//Расчет кочечного угла направления велосипеда 
						_agentParam._theta +=(LengthVector(agent.vel) * tan(_agentParam._delta) / _agentParam._length);

						//Расчет конечной скорости 
						agent.velNew.x = LengthVector(agent.vel) * cos(_agentParam._theta);
						agent.velNew.y = LengthVector(agent.vel) * sin(_agentParam._theta);

						//Обновление ориентации 
						_agentParam._orintX = cos(_agentParam._theta);
						_agentParam._orintY = sin(_agentParam._theta);

					}

					//Задаем начальную скорость по направлению велосипеда если она была нулевой
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
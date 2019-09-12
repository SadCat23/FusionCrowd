#pragma once

#include <string>
#include <map>
#include <vector>

#include "Config.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h"
#include "NavMeshNode.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	namespace NavMeshInfo
	{
		using namespace DirectX::SimpleMath;

		struct ObstacleInfo
		{
			int _v0;
			int _v1;
			int _node;
			int _nextObs;

			ObstacleInfo() : _v0(-1), _v1(-1), _node(-1), _nextObs(-1) {}

			ObstacleInfo(int v0, int v1, int node, int nextObs) : _v0(v0), _v1(v1), _node(node), _nextObs(nextObs) {}
		};

		struct EdgeInfo
		{
			int _v0 = -1;
			int _v1 = -1;
			int _n0 = -1;
			int _n1 = -1;

			EdgeInfo() : _v0(-1), _v1(-1), _n0(-1), _n1(-1) {}

			EdgeInfo(int v0, int v1, int n0, int n1) : _v0(v0), _v1(v1), _n0(n0), _n1(n1) {}
		};

		struct NodeInfo
		{
			Vector2 _centre;
			std::vector<int> _vertexs;
			DirectX::SimpleMath::Vector3 _elevation;
			std::vector<int> _edges;
			std::vector<int> _obstacles;

			NodeInfo() : _centre(Vector2(0, 0)), _elevation(Vector3(0, 0, 0)), _edges(), _obstacles()
			{
				_vertexs.clear();
				_edges.clear();
				_obstacles.clear();
			}

			NodeInfo(Vector2 centre, std::vector<int> vertexs, Vector3 elevation, std::vector<int> edges, std::vector<int> obstacles) :
				_centre(centre), _vertexs(vertexs), _elevation(elevation), _edges(edges), _obstacles(obstacles) {}
		};

		struct NavMeshInfo
		{
			std::vector<Vector2>  _vertex;
			std::vector<EdgeInfo> _edge;
			std::vector<ObstacleInfo> _obstacle;
			std::vector<NodeInfo> _nodes;
		};
	};

	// forward declarations
	class NavMesh;
	class NavMeshNode;
	class NavMeshEdge;
	class NavMeshLocalizer;
	class PathPlanner;

	class NMNodeGroup
	{
	public:
		NMNodeGroup() : _first(0), _last(0)
		{
		}

		NMNodeGroup(size_t first, size_t last) : _first(first), _last(last)
		{
		}

		inline size_t getFirst() const { return _first; }
		inline size_t getLast() const { return _last; }
		inline size_t getGlobalId(unsigned int i) const { return _first + i; }
		inline size_t groupSize() const { return static_cast<size_t>(_last - _first + 1); }

	private:
		size_t _first;
		size_t _last;
	};

	class FUSION_CROWD_API NavMesh
	{
	public:
		NavMesh(const std::string& name);
		~NavMesh();
		//Load
		static std::shared_ptr<NavMesh> Load(const std::string& FileName);


		bool IsPointInPolygon(NavMeshNode* nodes, DirectX::SimpleMath::Vector2 point);
		int CheckObstacle(std::vector<DirectX::SimpleMath::Vector2> contour);
		std::vector<int> GetCountur(std::vector<int> idNodes);
		void TriangulateCounturAndObstale(std::vector<DirectX::SimpleMath::Vector2> outer, std::vector<DirectX::SimpleMath::Vector2> contour, std::vector<int>& triangles, std::vector<DirectX::SimpleMath::Vector2>& vertex);
		NavMeshInfo::NavMeshInfo GetNewNode(std::vector<int> triangles, std::vector<DirectX::SimpleMath::Vector2> vertex, std::vector<DirectX::SimpleMath::Vector2> contour);
		void AddNode(NavMeshInfo::NavMeshInfo navMeshInfo, std::vector<int> idNodes, std::shared_ptr<NavMesh>& mesh);
		std::vector<int> GetListIncomingNodes(std::vector<DirectX::SimpleMath::Vector2> contour);

		void clear();
		bool finalize();
		//Vertex
		void SetVertexCount(size_t count);
		void SetVertex(unsigned int i, float x, float y);
		inline DirectX::SimpleMath::Vector2* GetVertices() { return & vertices[0]; }
		//Edge
		void SetEdgeCount(size_t count);
		NavMeshEdge& GetEdge(unsigned int i);
		//Obstacle
		void SetObstacleCount(size_t count);
		NavMeshObstacle& GetObstacle(unsigned int i);
		int getObstacleCount() { return obstCount; }
		//Node
		bool AddGroup(const std::string& grpName, size_t grpSize);
		NavMeshNode& GetNode(unsigned int i);
		inline size_t getNodeCount() const { return nCount; }
		const NMNodeGroup* getNodeGroup(const std::string& grpName) const;

	protected:
		std::string fileName;
		size_t vCount;
		DirectX::SimpleMath::Vector2* vertices;
		int eCount;
		NavMeshEdge* edges;
		int obstCount;
		NavMeshObstacle* obstacles;
		int nCount;
		NavMeshNode* nodes;
		std::map<const std::string, NMNodeGroup> nodeGroups;

		DirectX::SimpleMath::Vector2 ÑalculateNodeCenter(std::vector<DirectX::SimpleMath::Vector2> vertexs);
	};
}

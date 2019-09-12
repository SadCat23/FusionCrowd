#include <fstream>
#include "NavMesh.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h";
#include "triangle/triangulate.h"
//const std::string NavMesh::LABEL = "navmesh";

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavMesh::NavMesh(const std::string& name) : vCount(0), vertices(0x0),
	                                            nCount(0), nodes(0x0),
												eCount(0), edges(0x0),
												obstCount(0), obstacles(0x0),
	                                            nodeGroups()
	{
	}

#ifdef _WIN32
	// This disables a 64-bit compatibility warning - pushing a 64-bit value into a 32-bit value.
	//	In this case, I know the value in the pointers that are being re-interpreted as
	//  unsigned ints are REALLY just unsigned ints, so it is safe.
#pragma warning( disable : 4311 )
#endif
	bool NavMesh::finalize()
	{
		// All of the edge indices in the nodes need to be replaced with pointers
		// All of the obstacle indices in the nodes need to be replaced with pointers.
		for (size_t n = 0; n < nCount; ++n)
		{
			NavMeshNode& node = nodes[n];
			for (size_t e = 0; e < node._edgeCount; ++e)
			{
				// TODO: This might not work in building 64-bit code.
				//		The pointer will be larger than the unsigned int.  But as I'm pushing an
				//		unsigned int into a pointer slot, it'll probably be safe.  Needs to be
				//		tested.

				size_t eID = reinterpret_cast<size_t>(node._edges[e]);
				node._edges[e] = &edges[eID];
			}
			for (size_t o = 0; o < node._obstCount; ++o)
			{
				size_t oID = reinterpret_cast<size_t>(node._obstacles[o]);
				node._obstacles[o] = &obstacles[oID];
			}
			node._id = static_cast<unsigned int>(n);
			node._poly.setBB(vertices);
		}

		// All of the node indices in the edges need to be replaced with pointers
		for (size_t e = 0; e < eCount; ++e)
		{
			NavMeshEdge& edge = edges[e];
			size_t nID = reinterpret_cast<size_t>(edge._node0);
			edge._node0 = &nodes[nID];

			nID = reinterpret_cast<size_t>(edge._node1);
			edge._node1 = &nodes[nID];
			// compute edge distance
			edge._distance = (edge._node0->getCenter() - edge._node1->getCenter()).Length();

			// Confirm that point is on the left when looking from node0
			if (FusionCrowd::MathUtil::det(edge._dir, edge._node0->_center - edge._point) > 0.f)
			{
				NavMeshNode* tmp = edge._node0;
				edge._node0 = edge._node1;
				edge._node1 = tmp;
			}
		}

		std::vector<bool> processed(obstCount, false);
		for (size_t o = 0; o < obstCount; ++o)
		{
			obstacles[o]._id = o;
			if (processed[o]) continue;
			const size_t START = o;
			size_t curr = o;
			while (curr != NavMeshObstacle::NO_NEIGHBOR_OBST && !processed[curr])
			{
				processed[curr] = true;
				NavMeshObstacle& obst = obstacles[curr];
				size_t nID = reinterpret_cast<size_t>(obst._node);
				obst._node = &nodes[nID];

				nID = reinterpret_cast<size_t>(obst._nextObstacle);
				if (nID == NavMeshObstacle::NO_NEIGHBOR_OBST)
				{
					obst._nextObstacle = 0x0;
				}
				else
				{
					obst._nextObstacle = &obstacles[nID];
					//	Wire up "_prevObstacle" with the previous obstacle
					obstacles[nID]._prevObstacle = &obstacles[curr];
				}
				curr = nID;
			}
			// set open/closed
			if (curr == NavMeshObstacle::NO_NEIGHBOR_OBST ||
				curr != START)
			{
				// set open
				Obstacle* obst = &obstacles[START];
				obst->setClosedState(false);
				while (obst->_nextObstacle != 0x0)
				{
					obst = obst->_nextObstacle;
					obst->setClosedState(false);
				}
			}
		}

		return true;
	}
#ifdef _WIN32
#pragma warning( default : 4311 )
#endif

	std::shared_ptr<NavMesh> NavMesh::Load(const std::string& FileName)
	{
		// TODO: Change this to support comments.
		std::ifstream f;
		f.open(FileName.c_str(), std::ios::in);

		if (!f.is_open())
		{
			return NULL;
		}

		// load vertices
		unsigned int vertCount;
		if (!(f >> vertCount))
		{
			return NULL;
		}

		std::shared_ptr<NavMesh> mesh = std::make_shared<NavMesh>(FileName);
		mesh->SetVertexCount(vertCount);
		float x, y;
		for (unsigned int v = 0; v < vertCount; ++v)
		{
			if (!(f >> x >> y))
			{
				return NULL;
			}
			mesh->SetVertex(v, x, y);
		}

		// load edges
		unsigned int edgeCount;
		if (!(f >> edgeCount))
		{
			return NULL;
		}
		mesh->SetEdgeCount(edgeCount);
		for (unsigned int e = 0; e < edgeCount; ++e)
		{
			NavMeshEdge& edge = mesh->GetEdge(e);
			if (!edge.loadFromAscii(f, e, mesh->vertices))
			{
				return NULL;
			}
		}

		// load obstacles
		unsigned int obstCount;
		if (!(f >> obstCount))
		{
			return NULL;
		}
		mesh->SetObstacleCount(obstCount);
		for (unsigned int o = 0; o < obstCount; ++o)
		{
			NavMeshObstacle& obst = mesh->GetObstacle(o);
			if (!obst.LoadFromAscii(f, o, mesh->vertices))
			{
				return NULL;
			}
		}

		unsigned int totalN = 0;
		unsigned int n = 0;
		// load node group
		while (!f.eof())
		{
			std::string grpName;
			if (!(f >> grpName))
			{
				if (f.eof())
				{
					break;
				}
				else
				{
					return 0x0;
				}
			}
			// load nodes
			unsigned int nCount;
			if (!(f >> nCount))
			{
				return 0x0;
			}

			totalN += nCount;
			mesh->AddGroup(grpName, nCount);
			assert(totalN == mesh->getNodeCount() &&
				"Data management problem -- tracked node count does not match "
				"in-memory node count");

			for (; n < totalN; ++n)
			{
				NavMeshNode& node = mesh->GetNode(n);
				if (!node.loadFromAscii(f))
				{
					return 0x0;
				}

				node.setID(n);
				node.setVertices(mesh->GetVertices());
			}
		}

		if (!mesh->finalize())
		{
			return NULL;
		}

		return mesh;
	}

#pragma region  Vertex
	void NavMesh::SetVertexCount(size_t count)
	{
		if (vCount)
		{
			delete[] vertices;
		}
		vCount = count;
		vertices = new Vector2[vCount];
	}

	void NavMesh::SetVertex(unsigned int i, float x, float y)
	{
		vertices[i] = Vector2(x, y);
	}
#pragma endregion

#pragma region  Edge
	void NavMesh::SetEdgeCount(size_t count)
	{
		if (eCount)
		{
			delete[] edges;
		}
		eCount = count;
		edges = new NavMeshEdge[eCount];
	}

	NavMeshEdge& NavMesh::GetEdge(unsigned int i)
	{
		return edges[i];
	}
#pragma endregion

#pragma region Obstacle
	void NavMesh::SetObstacleCount(size_t count)
	{
		if (obstCount)
		{
			delete[] obstacles;
		}
		obstCount = count;
		obstacles = new NavMeshObstacle[obstCount];
	}

	NavMeshObstacle& NavMesh::GetObstacle(unsigned int i)
	{
		return obstacles[i];
	}
#pragma endregion

#pragma region Node
	bool NavMesh::AddGroup(const std::string& grpName, size_t grpSize)
	{
		if (nodeGroups.find(grpName) != nodeGroups.end())
		{
			return false;
		}
		size_t first = nCount;
		size_t last = first + grpSize - 1;
		nodeGroups[grpName] = NMNodeGroup(static_cast<unsigned int>(first),
		                                  static_cast<unsigned int>(last));

		// Now extend the node memory
		NavMeshNode* tmpNodes = new NavMeshNode[nCount + grpSize];
		if (nCount)
		{
			for (size_t i = 0; i < nCount; ++i)
			{
				tmpNodes[i] = nodes[i];
			}
			delete[] nodes;
		}
		nCount += grpSize;
		nodes = tmpNodes;
		return true;
	}

	NavMeshNode& NavMesh::GetNode(unsigned int i)
	{
		return nodes[i];
	}

	const NMNodeGroup* NavMesh::getNodeGroup(const std::string& grpName) const
	{
		std::map<const std::string, NMNodeGroup>::const_iterator itr = nodeGroups.find(grpName);
		NMNodeGroup* grp = 0x0;
		if (itr != nodeGroups.end())
		{
			return &(itr->second);
		}
		return grp;
	}

#pragma endregion

	void NavMesh::clear()
	{
		if (vCount)
		{
			vCount = 0;
			delete[] vertices;
			vertices = 0x0;
		}

		if (nCount)
		{
			nCount = 0;
			delete[] nodes;
			nodes = 0x0;
		}

		if (eCount)
		{
			eCount = 0;
			delete[] edges;
			edges = 0x0;
		}
	}


	bool NavMesh::IsPointInPolygon(NavMeshNode* nodes, DirectX::SimpleMath::Vector2 point)
	{
		int nodeCount = nodes->getVertexCount();
		int j = nodeCount - 1;
		bool oddNodes = false;
		for (int i = 0; i < nodeCount; i++) {
			int idV = nodes->getVertexID(i);
			int idV1 = nodes->getVertexID(j);
			DirectX::SimpleMath::Vector2 v0 = vertices[idV];
			DirectX::SimpleMath::Vector2 v1 = vertices[idV1];
			if ((v0.y < point.y && v1.y >= point.y || v1.y < point.y && v0.y >= point.y) && (v0.x <= point.x || v1.x <= point.x))
			{
				oddNodes ^= (v0.x + (point.y - v0.y) / (v1.y - v0.y) * (v1.x - v0.x) < point.x);
			}
			j = i;
		}
		return oddNodes;
	}

	int NavMesh::CheckObstacle(std::vector<DirectX::SimpleMath::Vector2> contour)
	{
		for (int i = 0; i < nCount; i++)
		{
			bool isConsist = true;
			for (int j = 0; j < contour.size(); j++)
			{
				isConsist = IsPointInPolygon(&nodes[i], contour[j]);
				if (!isConsist) {
					break;
				}
			}
			if (isConsist) {
				return i;
			}
		}
		return -1;
	}

	std::vector<int> NavMesh::GetCountur(std::vector<int> idNodes)
	{
		struct EdgeNode
		{
			int _v0;
			int _v1;
			bool _twin;
			int idNode;
			int idEdge;
			EdgeNode(int v0, int v1) {
				_v0 = v0;
				_v1 = v1;
				_twin = false;
				idNode = -1;
				idEdge = -1;
			}

			EdgeNode() {
				_v0 = -1;
				_v1 = -1;
				_twin = false;
				idNode = -1;
				idEdge = -1;
			}
		};

		struct NeighborNodes
		{
			int idNode;
			std::vector<int> idNodes;
			std::vector<EdgeNode> edgeNode;
			std::vector<int> vertex;
		};

		std::vector<Vector2> points;
		std::vector<int> idVertexCounter;
		if (idNodes.size() == 1) {
			int vertexCount = nodes[idNodes[0]].getVertexCount();
			for (int i = 0; i < vertexCount; i++) {
				nodes[idNodes[0]].getVertexID(i);
			}
		}
		else {
			std::vector<NeighborNodes> neighborNodes;
			for (int i = 0; i < idNodes.size(); i++) {
				NeighborNodes test;
				test.idNode = idNodes[i];
				std::vector<int> neighbor;
				int edgeCount = nodes[idNodes[i]].getEdgeCount();
				for (int j = 0; j < edgeCount; j++) {
					int idN0 = nodes[idNodes[i]].getEdge(j)->getFirstNode()->getID();
					int idN1 = nodes[idNodes[i]].getEdge(j)->getSecondNode()->getID();
					if (idN0 != idNodes[i]) {
						neighbor.push_back(idN0);
					}
					if (idN1 != idNodes[i]) {
						neighbor.push_back(idN1);
					}
				}
				for (int j = 0; j < idNodes.size(); j++) {
					for (int g = 0; g < neighbor.size(); g++) {
						if (idNodes[i] != idNodes[j])
						{
							if (neighbor[g] == idNodes[j]) {
								test.idNodes.push_back(neighbor[g]);
							}
						}
					}
				}

				int g = nodes[idNodes[i]].getVertexCount() - 1;;
				for (int j = 0; j < nodes[idNodes[i]].getVertexCount(); j++)
				{
					test.edgeNode.push_back(EdgeNode(nodes[idNodes[i]].getVertexID(g), nodes[idNodes[i]].getVertexID(j)));
					test.vertex.push_back(nodes[idNodes[i]].getVertexID(j));
					g = j;
				}
				neighborNodes.push_back(test);
			}

			for (int i = 0; i < neighborNodes.size(); i++) {
				for (int e = 0; e < neighborNodes[i].edgeNode.size(); e++) {
					for (int j = 0; j < neighborNodes[i].idNodes.size(); j++) {
						for (int g = 0; g < neighborNodes.size(); g++) {
							if ((neighborNodes[i].idNodes[j] == neighborNodes[g].idNode) && (neighborNodes[i].idNode != neighborNodes[g].idNode))
							{
								for (int e1 = 0; e1 < neighborNodes[g].edgeNode.size(); e1++) {
									if (((neighborNodes[g].edgeNode[e1]._v0 == neighborNodes[i].edgeNode[e]._v0)&& (neighborNodes[g].edgeNode[e1]._v1 == neighborNodes[i].edgeNode[e]._v1))||
									((neighborNodes[g].edgeNode[e1]._v0 == neighborNodes[i].edgeNode[e]._v1) && (neighborNodes[g].edgeNode[e1]._v1 == neighborNodes[i].edgeNode[e]._v0))){
										neighborNodes[i].edgeNode[e]._twin = true;
										neighborNodes[i].edgeNode[e].idNode = g;
										if (e1 == neighborNodes[g].edgeNode.size() - 1) {
											neighborNodes[i].edgeNode[e].idEdge = 0;
										}
										else {
											neighborNodes[i].edgeNode[e].idEdge = e1 + 1;
										}
									}
								}
							}
						}
					}
				}
			}

			int idNode = 0;
			int idEdge = 0;
			bool isClosed = false;
			while (!isClosed) {
				if (!neighborNodes[idNode].edgeNode[idEdge]._twin) {
					if (idVertexCounter.size() > 0) {
						if (idVertexCounter[0] == neighborNodes[idNode].edgeNode[idEdge]._v0)
						{
							isClosed = true;
							break;
						}
					}
					idVertexCounter.push_back(neighborNodes[idNode].edgeNode[idEdge]._v0);
					if (idEdge == neighborNodes[idNode].edgeNode.size() - 1)
					{
						idEdge = 0;
					}
					else {
						idEdge++;
					}
				}
				else {
					int _idNode = neighborNodes[idNode].edgeNode[idEdge].idNode;
					int _idEdge = neighborNodes[idNode].edgeNode[idEdge].idEdge;
					idNode = _idNode;
					idEdge = _idEdge;
				}
			}
		}
		return idVertexCounter;
	}

	void NavMesh::TriangulateCounturAndObstale(std::vector<DirectX::SimpleMath::Vector2> outer, std::vector<DirectX::SimpleMath::Vector2> contour, std::vector<int>& triangles, std::vector<DirectX::SimpleMath::Vector2>& vertex) {
		Eigen::MatrixXd V;
		Eigen::MatrixXi E;
		Eigen::MatrixXd H;

		Eigen::MatrixXd V2;
		Eigen::MatrixXi F2;

		std::vector<float> inPoints;
		std::vector<int> inEdges;
		std::vector<float> holes;

		int prevId = -1;
		for (int i = 0; i < outer.size(); i++) {
			inPoints.push_back(outer[i].x);
			inPoints.push_back(outer[i].y);
			vertex.push_back(outer[i]);
			if (i > 0) {
				inEdges.push_back(prevId);
				inEdges.push_back(i);
			}
			prevId = i;
		}
		inEdges.push_back(prevId);
		inEdges.push_back(0);

		prevId = outer.size();
		int c_nodes = outer.size();
		int idMinXCountur = 0;
		float minXCountur = contour[0].x;
		for (int i = 0; i < contour.size(); i++) {

			inPoints.push_back(contour[i].x);
			inPoints.push_back(contour[i].y);
			vertex.push_back(contour[i]);

			if (i > 0) {
				inEdges.push_back(prevId);
				inEdges.push_back(c_nodes + i);
			}
			prevId = i + c_nodes;
			if (minXCountur > contour[i].x) {
				idMinXCountur = i;
				minXCountur = contour[i].x;
			}
		}
		inEdges.push_back(prevId);
		inEdges.push_back(c_nodes);

		int indMinus = idMinXCountur - 1 < 0 ? contour.size() - 1 : idMinXCountur - 1;
		int indPlus = idMinXCountur + 1 >= contour.size() ? 0 : idMinXCountur + 1;

		DirectX::SimpleMath::Vector2 delta = contour[indMinus] + contour[indPlus] - 2 * contour[idMinXCountur];
		delta.Normalize();
		delta = 0.02f * delta;

		DirectX::SimpleMath::Vector2 p = contour[idMinXCountur] + delta;
		holes.push_back(p.x);
		holes.push_back(p.y);

		V.resize(inPoints.size() / 2, 2);
		E.resize(inEdges.size() / 2, 2);
		H.resize(holes.size() / 2, 2);

		for (int i = 0; i < inPoints.size(); i += 2) {
			V(i / 2, 0) = inPoints[i];
			V(i / 2, 1) = inPoints[i + 1];
		}
		for (int i = 0; i < inEdges.size(); i += 2) {
			E(i / 2, 0) = inEdges[i];
			E(i / 2, 1) = inEdges[i + 1];
		}
		for (int i = 0; i < holes.size(); i += 2) {
			H(i / 2, 0) = holes[i];
			H(i / 2, 1) = holes[i + 1];
		}

		igl::triangle::triangulate(V, E, H, "", V2, F2);

		for (int i = 0; i < F2.rows(); i++)
		{
			triangles.push_back(F2(i, 2));
			triangles.push_back(F2(i, 1));
			triangles.push_back(F2(i, 0));
		}
	}

	NavMeshInfo::NavMeshInfo NavMesh::GetNewNode(std::vector<int> triangles, std::vector<DirectX::SimpleMath::Vector2>  vertex, std::vector<DirectX::SimpleMath::Vector2> contour)
	{
		NavMeshInfo::NavMeshInfo _navMeshInfo;
		std::vector<int> cId;
		std::vector<NavMeshInfo::ObstacleInfo> obstacle;
		for (int i = 0; i < vertex.size(); i++) {
			for (int j = 0; j < contour.size(); j++) {
				if (vertex[i] == contour[j]) {
					cId.push_back(i);
				}
			}
		}

		for (int i = 0; i < cId.size(); i++) {
			NavMeshInfo::ObstacleInfo info;
			if (i + 1 >= cId.size() ) {
				info._v0 = cId[i];
				info._v1 = cId[0];
			}
			else {
				info._v0 = cId[i];
				info._v1 = cId[i+1];
			}
			obstacle.push_back(info);
		}

		std::vector<NavMeshInfo::NodeInfo> nodes;

		for (int i = 0; i < triangles.size(); i+=3) {
			NavMeshInfo::NodeInfo info;
			info._vertexs.push_back(triangles[i]);
			info._vertexs.push_back(triangles[i + 1]);
			info._vertexs.push_back(triangles[i + 2]);

			nodes.push_back(info);
		}

		for (int i = 0; i < nodes.size(); i++) {
			for (int j = 0; j < obstacle.size(); j++) {
				if (((nodes[i]._vertexs[0] == obstacle[j]._v0) && (nodes[i]._vertexs[1] == obstacle[j]._v1)) ||
					((nodes[i]._vertexs[0] == obstacle[j]._v1) && (nodes[i]._vertexs[1] == obstacle[j]._v0)))
				{
					obstacle[j]._node = i;

				}
				if (((nodes[i]._vertexs[1] == obstacle[j]._v0) && (nodes[i]._vertexs[2] == obstacle[j]._v1)) ||
					((nodes[i]._vertexs[1] == obstacle[j]._v1) && (nodes[i]._vertexs[2] == obstacle[j]._v0)))
				{
					obstacle[j]._node = i;
				}
				if (((nodes[i]._vertexs[2] == obstacle[j]._v0) && (nodes[i]._vertexs[0] == obstacle[j]._v1)) ||
					((nodes[i]._vertexs[2] == obstacle[j]._v1) && (nodes[i]._vertexs[0] == obstacle[j]._v0)))
				{
					obstacle[j]._node = i;
				}

				if ((nodes[i]._vertexs[0] == obstacle[j]._v0)||(nodes[i]._vertexs[1] == obstacle[j]._v0)||(nodes[i]._vertexs[2] == obstacle[j]._v0))
				{
					nodes[i]._obstacles.push_back(j);
				}
				else {
					if ((nodes[i]._vertexs[0] == obstacle[j]._v1) || (nodes[i]._vertexs[1] == obstacle[j]._v1) || (nodes[i]._vertexs[2] == obstacle[j]._v1))
					{
						nodes[i]._obstacles.push_back(j);
					}
				}
			}
		}

		std::vector<NavMeshInfo::EdgeInfo> edge;
		for (int i = 0; i < nodes.size(); i++) {
			bool isObstacle = false;
			for (int j = 0; j < obstacle.size(); j++) {
				if (((nodes[i]._vertexs[0] == obstacle[j]._v0) && (nodes[i]._vertexs[1] == obstacle[j]._v1)) ||
					((nodes[i]._vertexs[0] == obstacle[j]._v1) && (nodes[i]._vertexs[1] == obstacle[j]._v0)))
				{
					isObstacle =true;
				}
			}
			if (!isObstacle) {
				NavMeshInfo::EdgeInfo edgeInfo;
				edgeInfo._v0 = nodes[i]._vertexs[0];
				edgeInfo._v1 = nodes[i]._vertexs[1];
				for (int j = 0; j < edge.size(); j++) {
					if (((edgeInfo._v0 == edge[j]._v0)&&(edgeInfo._v1 == edge[j]._v1)) ||
					((edgeInfo._v0 == edge[j]._v1) && (edgeInfo._v1 == edge[j]._v0))){
						isObstacle = true;
					}
				}
				if (!isObstacle) {
					edgeInfo._v0 = nodes[i]._vertexs[0];
					edgeInfo._v1 = nodes[i]._vertexs[1];
					edge.push_back(edgeInfo);
				}
			}

			isObstacle = false;
			for (int j = 0; j < obstacle.size(); j++) {
				if (((nodes[i]._vertexs[1] == obstacle[j]._v0) && (nodes[i]._vertexs[2] == obstacle[j]._v1)) ||
					((nodes[i]._vertexs[1] == obstacle[j]._v1) && (nodes[i]._vertexs[2] == obstacle[j]._v0)))
				{
					isObstacle = true;
				}
			}
			if (!isObstacle) {
				NavMeshInfo::EdgeInfo edgeInfo;
				edgeInfo._v0 = nodes[i]._vertexs[1];
				edgeInfo._v1 = nodes[i]._vertexs[2];
				for (int j = 0; j < edge.size(); j++) {
					if (((edgeInfo._v0 == edge[j]._v0) && (edgeInfo._v1 == edge[j]._v1)) ||
						((edgeInfo._v0 == edge[j]._v1) && (edgeInfo._v1 == edge[j]._v0))) {
						isObstacle = true;
					}
				}
				if (!isObstacle) {
					edgeInfo._v0 = nodes[i]._vertexs[1];
					edgeInfo._v1 = nodes[i]._vertexs[2];
					edge.push_back(edgeInfo);
				}
			}

			isObstacle = false;
			for (int j = 0; j < obstacle.size(); j++) {
				if (((nodes[i]._vertexs[2] == obstacle[j]._v0) && (nodes[i]._vertexs[0] == obstacle[j]._v1)) ||
					((nodes[i]._vertexs[2] == obstacle[j]._v1) && (nodes[i]._vertexs[0] == obstacle[j]._v0)))
				{
					isObstacle = true;
				}
			}
			if (!isObstacle) {
				NavMeshInfo::EdgeInfo edgeInfo;
				edgeInfo._v0 = nodes[i]._vertexs[2];
				edgeInfo._v1 = nodes[i]._vertexs[0];
				for (int j = 0; j < edge.size(); j++) {
					if (((edgeInfo._v0 == edge[j]._v0) && (edgeInfo._v1 == edge[j]._v1)) ||
						((edgeInfo._v0 == edge[j]._v1) && (edgeInfo._v1 == edge[j]._v0))) {
						isObstacle = true;
					}
				}
				if (!isObstacle) {
					edgeInfo._v0 = nodes[i]._vertexs[2];
					edgeInfo._v1 = nodes[i]._vertexs[0];
					edge.push_back(edgeInfo);
				}
			}

		}

		for (int i = 0; i < nodes.size(); i++) {
			for (int j = 0; j < edge.size(); j++) {
				if (((nodes[i]._vertexs[0] == edge[j]._v0) && (nodes[i]._vertexs[1] == edge[j]._v1)) ||
					((nodes[i]._vertexs[0] == edge[j]._v1) && (nodes[i]._vertexs[1] == edge[j]._v0)))
				{
					if (edge[j]._n0 == -1) {
						edge[j]._n0 = i;
					}
					else {
						if ((edge[j]._n1 == -1) && (edge[j]._n0 != -1))
						{
							edge[j]._n1 = i;
						}
					}
				}

				if (((nodes[i]._vertexs[1] == edge[j]._v0) && (nodes[i]._vertexs[2] == edge[j]._v1)) ||
					((nodes[i]._vertexs[1] == edge[j]._v1) && (nodes[i]._vertexs[2] == edge[j]._v0)))
				{
					if (edge[j]._n0 == -1) {
						edge[j]._n0 = i;
					}
					else {
						if ((edge[j]._n1 == -1) && (edge[j]._n0 != -1))
						{
							edge[j]._n1 = i;
						}
					}
				}

				if (((nodes[i]._vertexs[2] == edge[j]._v0) && (nodes[i]._vertexs[0] == edge[j]._v1)) ||
					((nodes[i]._vertexs[2] == edge[j]._v1) && (nodes[i]._vertexs[0] == edge[j]._v0)))
				{
					if (edge[j]._n0 == -1) {
						edge[j]._n0 = i;
					}
					else {
						if ((edge[j]._n1 == -1) && (edge[j]._n0 != -1))
						{
							edge[j]._n1 = i;
						}
					}
				}
			}
		}

		for (int i = 0; i < edge.size(); i++) {
			if (edge[i]._n0 != -1) {
				nodes[edge[i]._n0]._edges.push_back(i);
			}
			if (edge[i]._n1 != -1) {
				nodes[edge[i]._n1]._edges.push_back(i);
			}
		}

		for (int i = 0; i < nodes.size(); i++) {
			std::vector<Vector2> vertexsNode;
			for (int j = 0; j < nodes[i]._vertexs.size(); j++) {
				vertexsNode.push_back(vertex[nodes[i]._vertexs[j]]);
			}
			nodes[i]._centre = ÑalculateNodeCenter(vertexsNode);
		}
		_navMeshInfo._vertex = vertex;
		_navMeshInfo._edge = edge;
		_navMeshInfo._obstacle = obstacle;
		_navMeshInfo._nodes = nodes;
		return _navMeshInfo;
	}

	void NavMesh::AddNode(NavMeshInfo::NavMeshInfo navMeshInfo, std::vector<int> idNodes, std::shared_ptr<NavMesh>& mesh)
	{
		struct OldNew {
			int _old;
			int _new;
			OldNew() : _old(-1), _new(-1)
			{
			}
			OldNew(int oldID, int newId) : _old(oldID), _new(newId)
			{
			}
		};

		struct OldNewEdge {
			int _old;
			int _new;
			int _oldNode;
			NavMeshInfo::EdgeInfo _edge;

			OldNewEdge(): _old(-1), _new(-1), _oldNode(-1),_edge(NavMeshInfo::EdgeInfo())
			{
			}

			OldNewEdge(int oldID, int newId, int oldNew, NavMeshInfo::EdgeInfo edge) : _old(oldID), _new(newId), _oldNode(oldNew),_edge(edge)
			{
			}
		};

		struct OldNewObstacle
		{
			int _old;
			int _new;
			int _oldNode;
			NavMeshInfo::ObstacleInfo _obstacle;

			OldNewObstacle() : _old(-1), _new(-1), _oldNode(-1), _obstacle(NavMeshInfo::ObstacleInfo())
			{
			}

			OldNewObstacle(int oldID, int newId, int oldNode, NavMeshInfo::ObstacleInfo obstacle) : _old(oldID), _new(newId), _oldNode(oldNode), _obstacle(obstacle)
			{
			}
		};

		std::vector<int> idOldVertex;
		for (int i = 0; i < idNodes.size(); i++) {
			for (int j = 0; j < nodes[idNodes[i]].getVertexCount(); j++) {
				idOldVertex.push_back(nodes[idNodes[i]].getVertexID(j));
			}
		}

		//std::vector<int> idOldVertex;
		//for (int i = 0; i < nodesChange.size(); i++) {
		//	for (int j = 0; j < nodesChange[i].getVertexCount(); j++) {
		//		idOldVertex.push_back(nodesChange[i].getVertexID(j));
		//	}
		//}
		std::vector<Vector2> oldVertex;
		for (int i = 0; i < idOldVertex.size(); i++) {
			oldVertex.push_back(vertices[idOldVertex[i]]);
		}

		std::vector<Vector2> newVertex;
		std::vector<OldNew> st1;
		for (int i = 0; i < navMeshInfo._vertex.size(); i++) {
			bool isEv = true;
			for (int j = 0; j < vCount; j++) {
				if (navMeshInfo._vertex[i] == vertices[j]) {
					isEv = false;

					st1.push_back(OldNew(j, i));
				}
			}
			if (isEv) {
				newVertex.push_back(navMeshInfo._vertex[i]);
			}
		}
		//std::vector<int> d;

		int vertexsCount = vCount + newVertex.size();

		for (int i = 0; i < navMeshInfo._vertex.size(); i++) {
			for (int j = 0; j < newVertex.size(); j++) {
				if (navMeshInfo._vertex[i] == newVertex[j]) {
					st1.push_back(OldNew(vCount + j, i));
				}
			}
		}

		for (int i = 0; i < navMeshInfo._edge.size(); i++) {
			for (int j = 0; j < st1.size(); j++) {
				if (navMeshInfo._edge[i]._v0 == st1[j]._new) {
					navMeshInfo._edge[i]._v0 = st1[j]._old;
				}
				if (navMeshInfo._edge[i]._v1 == st1[j]._new) {
					navMeshInfo._edge[i]._v1 = st1[j]._old;
				}
			}
		}

		for (int i = 0; i < navMeshInfo._edge.size(); i++) {
			for (int j = 0; j < st1.size(); j++) {
				if (navMeshInfo._edge[i]._v0 == st1[j]._new) {
					navMeshInfo._edge[i]._v0 = st1[j]._old;
				}
				if (navMeshInfo._edge[i]._v1 == st1[j]._new) {
					navMeshInfo._edge[i]._v1 = st1[j]._old;
				}
			}
		}

		for (int i = 0; i < navMeshInfo._obstacle.size(); i++) {
			for (int j = 0; j < st1.size(); j++) {
				if (navMeshInfo._obstacle[i]._v0 == st1[j]._new) {
					navMeshInfo._obstacle[i]._v0 = st1[j]._old;
				}
				if (navMeshInfo._obstacle[i]._v1 == st1[j]._new) {
					navMeshInfo._obstacle[i]._v1 = st1[j]._old;
				}
			}
		}

		for (int i = 0; i < navMeshInfo._nodes.size(); i++) {
			for (int j = 0; j < navMeshInfo._nodes[i]._vertexs.size(); j++) {
				for (int k = 0; k < st1.size(); k++) {
					if (navMeshInfo._nodes[i]._vertexs[j] == st1[k]._new) {
						navMeshInfo._nodes[i]._vertexs[j] = st1[k]._old;
					}
				}
			}
		}
		std::vector<OldNewEdge> newEdge;
		std::vector<OldNewEdge> oldEdge;
		std::vector<OldNewObstacle> oldObstacle;
		for (int i = 0; i < navMeshInfo._edge.size(); i++) {
			bool isOld = false;
			for (int j = 0; j < eCount; j++) {
				int idV0 = edges[j]._v0;
				int idV1 = edges[j]._v1;
				if (((navMeshInfo._edge[i]._v0 == idV0) && (navMeshInfo._edge[i]._v1 == idV1)) || ((navMeshInfo._edge[i]._v0 == idV1) && (navMeshInfo._edge[i]._v1 == idV0))) {
					int idN0 = edges[j]._n0;
					int idN1 = edges[j]._n1;
					int idOldNode = -1;
					for (int k = 0; k < idNodes.size(); k++) {
						if (idN0 == idNodes[k]) {
							idN0 = -1;
							idOldNode = idNodes[k];
						}
						if (idN1 == idNodes[k]) {
							idN1 = -1;
							idOldNode = idNodes[k];
						}
					}

					if (navMeshInfo._edge[i]._n0 == -1){
						if (idN0 != -1) {
							navMeshInfo._edge[i]._n0 = idN0;
						}
						else {
							navMeshInfo._edge[i]._n0 = idN1;
						}
					}
					if (navMeshInfo._edge[i]._n1 == -1) {
						if (idN0 != -1) {
							navMeshInfo._edge[i]._n1 = idN0;
						}
						else {
							navMeshInfo._edge[i]._n1 = idN1;
						}
					}

					oldEdge.push_back(OldNewEdge(j, i, idOldNode, navMeshInfo._edge[i]));
					isOld = true;
				}
			}
			if ((navMeshInfo._edge[i]._n0 == -1) || (navMeshInfo._edge[i]._n1 == -1)) {
				if (navMeshInfo._edge[i]._v0 != -1) {
					oldObstacle.push_back(OldNewObstacle(-1, i, -1, NavMeshInfo::ObstacleInfo(navMeshInfo._edge[i]._v0, navMeshInfo._edge[i]._v1, nCount - idNodes.size() + navMeshInfo._edge[i]._n0, -1)));
				}
				else {
					oldObstacle.push_back(OldNewObstacle (-1, i, -1,NavMeshInfo::ObstacleInfo(navMeshInfo._edge[i]._v0, navMeshInfo._edge[i]._v1, nCount - idNodes.size() + navMeshInfo._edge[i]._n1, -1)));
				}
			}
			else {
				if (!isOld) {
					navMeshInfo._edge[i]._n0 = nCount - idNodes.size() + navMeshInfo._edge[i]._n0;
					navMeshInfo._edge[i]._n1 = nCount - idNodes.size() + navMeshInfo._edge[i]._n1;
					newEdge.push_back(OldNewEdge(-1, i, -1, navMeshInfo._edge[i]));
				}
			}

		}

		for (int i = 0; i < oldObstacle.size(); i++) {
			for (int j = 0; j < obstCount; j++) {
				obstacles[j]._v0;
				obstacles[j]._v1;
				if (((oldObstacle[i]._obstacle._v0 == obstacles[j]._v0) && (oldObstacle[i]._obstacle._v1 == obstacles[j]._v1))
					|| ((oldObstacle[i]._obstacle._v0 == obstacles[j]._v1) && (oldObstacle[i]._obstacle._v1 == obstacles[j]._v0))) {
					oldObstacle[i]._old = j;
					oldObstacle[i]._oldNode = obstacles[j]._n;

				}
			}
		}

		for (int i = 0; i < navMeshInfo._nodes.size(); i++) {
			for (int j = 0; j < navMeshInfo._nodes[i]._obstacles.size(); j++) {
				navMeshInfo._nodes[i]._obstacles[j] = obstCount + navMeshInfo._nodes[i]._obstacles[j];
			}
		}

		for (int i = 0; i < oldObstacle.size(); i++) {
			for (int j = 0; j < navMeshInfo._nodes.size(); j++) {
				std::vector<int> bufferEdge;
				for (int k = 0; k < navMeshInfo._nodes[j]._edges.size(); k++) {
					if (navMeshInfo._nodes[j]._edges[k] != oldObstacle[i]._new) {
						bufferEdge.push_back(navMeshInfo._nodes[j]._edges[k]);
					}
				}
				navMeshInfo._nodes[j]._edges = bufferEdge;
				if ((oldObstacle[i]._obstacle._v0 == navMeshInfo._nodes[j]._vertexs[0])||(oldObstacle[i]._obstacle._v0 == navMeshInfo._nodes[j]._vertexs[1])||(oldObstacle[i]._obstacle._v0 == navMeshInfo._nodes[j]._vertexs[2])||
					(oldObstacle[i]._obstacle._v1 == navMeshInfo._nodes[j]._vertexs[0])||(oldObstacle[i]._obstacle._v1 == navMeshInfo._nodes[j]._vertexs[1])||(oldObstacle[i]._obstacle._v1 == navMeshInfo._nodes[j]._vertexs[2])) {
					navMeshInfo._nodes[j]._obstacles.push_back(oldObstacle[i]._old);
				}
			}
		}

		for (int i = 0; i < oldEdge.size(); i++) {
			for (int j = 0; j < navMeshInfo._nodes.size(); j++) {
				for (int k = 0; k < navMeshInfo._nodes[j]._edges.size(); k++) {
					if (navMeshInfo._nodes[j]._edges[k] == oldEdge[i]._new) {
						navMeshInfo._nodes[j]._edges[k] = oldEdge[i]._old;
					}
				}
			}
		}

		for (int i = 0; i < newEdge.size(); i++) {
			for (int j = 0; j < navMeshInfo._nodes.size(); j++) {
				for (int k = 0; k < navMeshInfo._nodes[j]._edges.size(); k++) {
					if (navMeshInfo._nodes[j]._edges[k] == newEdge[i]._new) {
						navMeshInfo._nodes[j]._edges[k] =i + eCount;
					}
				}
			}
		}

		for (int i = 0; i < navMeshInfo._obstacle.size(); i++) {
			navMeshInfo._obstacle[i]._node = nCount - idNodes.size() + navMeshInfo._obstacle[i]._node;
		}

		std::sort(idNodes.begin(), idNodes.end(), std::greater<int>());

		NavMeshInfo::NavMeshInfo newNavMeshInfo;
		float x, y;
		for (unsigned int v = 0; v < vCount; ++v) {
			newNavMeshInfo._vertex.push_back(vertices[v]);
		}

		for (unsigned int v = 0; v < newVertex.size(); ++v) {
			newNavMeshInfo._vertex.push_back(newVertex[v]);
		}

		for (unsigned int e = 0; e < eCount; ++e) {
			NavMeshInfo::EdgeInfo ed;
			ed._v0 = edges[e]._v0;
			ed._v1 = edges[e]._v1;
			ed._n0 = -1;
			ed._n1 = -1;
			newNavMeshInfo._edge.push_back(ed);
		}

		for (unsigned int e = 0; e < newEdge.size(); ++e) {

			newNavMeshInfo._edge.push_back(newEdge[e]._edge);
		}

		for (unsigned int o = 0; o < obstCount; ++o) {
			NavMeshInfo::ObstacleInfo ob;
			ob._v0 = obstacles[o]._v0;
			ob._v1 = obstacles[o]._v1;
			ob._node = -1;
			ob._nextObs = -1;
			newNavMeshInfo._obstacle.push_back(ob);
		}

		for (unsigned int o = 0; o < navMeshInfo._obstacle.size(); ++o) {
			newNavMeshInfo._obstacle.push_back(navMeshInfo._obstacle[o]);
		}

		for (unsigned int n = 0; n < nCount; ++n) {
			bool isDelete = false;
			for (unsigned int j = 0; j < idNodes.size(); j++) {
				if (n == idNodes[j]) {
					isDelete = true;
				}
			}
			if (!isDelete) {
				NavMeshInfo::NodeInfo no;
				no._centre = nodes[n].getCenter();
				for (int i = 0; i < nodes[n].getVertexCount(); i++) {
					no._vertexs.push_back(nodes[n].getVertexID(i));
				}
				newNavMeshInfo._nodes.push_back(no);
			}
		}

		for (unsigned int n = 0; n < navMeshInfo._nodes.size(); ++n) {
			navMeshInfo._nodes[n]._obstacles.clear();
			navMeshInfo._nodes[n]._edges.clear();
			newNavMeshInfo._nodes.push_back(navMeshInfo._nodes[n]);
		}

		for (unsigned int e = 0; e < newNavMeshInfo._edge.size(); ++e) {

			for (unsigned int n = 0; n < newNavMeshInfo._nodes.size(); ++n) {
				bool isIncludeV0 = false;
				bool isIncludeV1 = false;
				for (unsigned int v = 0; v < newNavMeshInfo._nodes[n]._vertexs.size(); ++v) {
					if (newNavMeshInfo._nodes[n]._vertexs[v] == newNavMeshInfo._edge[e]._v0) {
						isIncludeV0 = true;
					}
					if (newNavMeshInfo._nodes[n]._vertexs[v] == newNavMeshInfo._edge[e]._v1) {
						isIncludeV1 = true;
					}
				}

				if (isIncludeV0 && isIncludeV1) {
					if (newNavMeshInfo._edge[e]._n0 == -1)
					{
						newNavMeshInfo._edge[e]._n0 = n;
					}
					else {
						if (n != newNavMeshInfo._edge[e]._n0) {
							newNavMeshInfo._edge[e]._n1 = n;

							newNavMeshInfo._nodes[newNavMeshInfo._edge[e]._n0]._edges.push_back(e);
							newNavMeshInfo._nodes[newNavMeshInfo._edge[e]._n1]._edges.push_back(e);
						}
					}
				}
			}
		}

		for (unsigned int o = 0; o < newNavMeshInfo._obstacle.size(); ++o) {
			for (unsigned int n = 0; n < newNavMeshInfo._nodes.size(); ++n) {
				bool isIncludeV0 = false;
				bool isIncludeV1 = false;
				for (unsigned int v = 0; v < newNavMeshInfo._nodes[n]._vertexs.size(); ++v) {
					if (newNavMeshInfo._nodes[n]._vertexs[v] == newNavMeshInfo._obstacle[o]._v0) {
						isIncludeV0 = true;
					}
					if (newNavMeshInfo._nodes[n]._vertexs[v] == newNavMeshInfo._obstacle[o]._v1) {
						isIncludeV1 = true;
					}
				}

				if (isIncludeV0 && isIncludeV1) {
					newNavMeshInfo._obstacle[o]._node = n;
				}

				if (isIncludeV0 || isIncludeV1) {
					newNavMeshInfo._nodes[n]._obstacles.push_back(o);
				}
			}

		}

		//final navMesh
		std::string f = "test";
		mesh = std::make_shared<NavMesh>(f);
		mesh->SetVertexCount(newNavMeshInfo._vertex.size());

		//float x, y;
		for (unsigned int v = 0; v < newNavMeshInfo._vertex.size(); ++v){
			mesh->SetVertex(v, newNavMeshInfo._vertex[v].x, newNavMeshInfo._vertex[v].y);
		}

		mesh->SetEdgeCount(newNavMeshInfo._edge.size());
		for (unsigned int e = 0; e < newNavMeshInfo._edge.size(); ++e)
		{
			NavMeshEdge& edge = mesh->GetEdge(e);
			edge.setEdge(newNavMeshInfo._edge[e]._v0, newNavMeshInfo._edge[e]._v1, newNavMeshInfo._edge[e]._n0, newNavMeshInfo._edge[e]._n1, mesh->vertices);
		}
		mesh->SetObstacleCount(newNavMeshInfo._obstacle.size());
		for (unsigned int o = 0; o < newNavMeshInfo._obstacle.size(); ++o)
		{
			NavMeshObstacle& obst = mesh->GetObstacle(o);
			obst.SetObstacle(newNavMeshInfo._obstacle[o]._v0, newNavMeshInfo._obstacle[o]._v1, newNavMeshInfo._obstacle[o]._node, newNavMeshInfo._obstacle[o]._nextObs, mesh->vertices);
		}
		mesh->AddGroup("Test", newNavMeshInfo._nodes.size());
		for (int n =0 ; n < newNavMeshInfo._nodes.size(); ++n)
		{
			NavMeshNode& node = mesh->GetNode(n);
			node.setCenter(newNavMeshInfo._nodes[n]._centre);
			node.setPolygon(newNavMeshInfo._nodes[n]._vertexs);
			node.setEdge(newNavMeshInfo._nodes[n]._edges);
			node.setObstacle(newNavMeshInfo._nodes[n]._obstacles);
			node.setID(n);
			node.setVertices(mesh->GetVertices());
		}

		bool r = mesh->finalize();
	}

	DirectX::SimpleMath::Vector2 NavMesh::ÑalculateNodeCenter(std::vector<Vector2> vertexs)
	{
		DirectX::SimpleMath::Vector2 center(0, 0);
		float square = 0;
		for (int i = 0; i < vertexs.size(); i++) {
			if (i + 1 >= vertexs.size()) {
				square = square + (vertexs[i].x * vertexs[0].y - vertexs[0].x * vertexs[i].y);
			}
			else {
				square = square + (vertexs[i].x * vertexs[i + 1].y - vertexs[i + 1].x * vertexs[i].y);
			}
		}
		square = 0.5 * square;

		float gx = 0;
		for (int i = 0; i < vertexs.size(); i++) {
			if (i + 1 >= vertexs.size()) {
				gx = gx + (vertexs[i].x + vertexs[0].x)*(vertexs[i].x * vertexs[0].y - vertexs[0].x *vertexs[i].y);
			}
			else {
				gx = gx + (vertexs[i].x + vertexs[i + 1].x)*(vertexs[i].x * vertexs[i + 1].y - vertexs[i + 1].x * vertexs[i].y);
			}
		}
		gx = gx / (6 * square);


		float gy = 0;
		for (int i = 0; i < vertexs.size(); i++) {
			if (i + 1 >= vertexs.size()) {
				gy = gy + (vertexs[i].y + vertexs[0].y)*(vertexs[i].x * vertexs[0].y - vertexs[0].x * vertexs[i].y);
			}
			else {
				gy = gy + (vertexs[i].y + vertexs[i + 1].y)*(vertexs[i].x * vertexs[i + 1].y - vertexs[i + 1].x * vertexs[i].y);
			}
		}
		gy = gy / (6 * square);

		return Vector2(gx, gy);
	}

	std::vector<int> NavMesh::GetListIncomingNodes(std::vector<DirectX::SimpleMath::Vector2> contour)
	{
		std::vector<int> idNodes;
		std::vector<Vector2> contourVector;

		for (int i = 0; i < contour.size(); i++) {
			Vector2 v;
			if (i + 1 >= contour.size()) {
				v.x = contour[0].x - contour[i].x;
				v.y = contour[0].y - contour[i].y;
			}
			else {
				v.x = contour[i + 1].x - contour[i].x;
				v.y = contour[i + 1].y - contour[i].y;
			}
			contourVector.push_back(v);
		}

		for (int i = 0; i < nCount; i++) {
			int countourSize = contour.size();
			int cV = 0;

			std::vector<Vector2> countourNode;
			std::vector<Vector2> vertexNode;

			for (int j = 0; j < nodes[i].getVertexCount(); j++) {
				vertexNode.push_back(vertices[nodes[i].getVertexID(j)]);
			}


			while (cV < countourSize) {
				for (int j = 0; j < vertexNode.size(); j++) {
					Vector2 v1;
					Vector2 v2;

					if (j + 1 >= vertexNode.size()) {
						v1.x = vertexNode[j].x - contour[cV].x;
						v1.y = vertexNode[j].y - contour[cV].y;

						v2.x = vertexNode[0].x - contour[cV].x;
						v2.y = vertexNode[0].y - contour[cV].y;
					}
					else {
						v1.x = vertexNode[j+1].x - contour[cV].x;
						v1.y = vertexNode[j+1].y - contour[cV].y;

						v2.x = vertexNode[j].x - contour[cV].x;
						v2.y = vertexNode[j].y - contour[cV].y;
					}

					float z1 = contourVector[cV].x * v1.y - contourVector[cV].y * v1.x;
					float z2 = contourVector[cV].x * v2.y - contourVector[cV].y * v2.x;

					if (((z1>0)&&(z2<0))||((z1 < 0)&&(z2 > 0))) {
						Vector2 v;
						if (j + 1 >= vertexNode.size()) {
							v.x = vertexNode[0].x - vertexNode[j].x;
							v.y = vertexNode[0].y - vertexNode[j].y;
						}
						else {
							v.x = vertexNode[j + 1].x - vertexNode[j].x;
							v.y = vertexNode[j + 1].y - vertexNode[j].y;
						}

						if (cV + 1 >= countourSize){
							v1.x = contour[cV].x - vertexNode[j].x;
							v1.y = contour[cV].y - vertexNode[j].y;

							v2.x = contour[0].x - vertexNode[j].x;
							v2.y = contour[0].y - vertexNode[j].y;
						}
						else {
							v1.x = contour[cV + 1].x - vertexNode[j].x;
							v1.y = contour[cV + 1].y - vertexNode[j].y;

							v2.x = contour[cV].x - vertexNode[j].x;
							v2.y = contour[cV].y - vertexNode[j].y;
						}
						z1 = v.x * v1.y - v.y * v1.x;
						z2 = v.x * v2.y - v.y * v2.x;
						if (((z1 > 0) && (z2 < 0)) || ((z1 < 0) && (z2 > 0))) {
							if (std::find(idNodes.begin(), idNodes.end(), i) == idNodes.end()) {
								idNodes.push_back(i);
							}
						}
					}
				}
				cV++;
			}
		}

		return idNodes;
	}

	NavMesh::~NavMesh()
	{
		clear();
	}
}

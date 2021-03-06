#pragma once

#include <istream>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "Math/Util.h"

namespace FusionCrowd
{
	struct NavGraphNode
	{
		size_t id;
		DirectX::SimpleMath::Vector2 position;

		NavGraphNode() : id(-1), position(DirectX::SimpleMath::Vector2(0, 0))
		{}

		NavGraphNode(size_t id, DirectX::SimpleMath::Vector2 pos) : id(id), position(pos)
		{}

		NavGraphNode(const NavGraphNode& other) : id(other.id), position(std::move(other.position))
		{}

		NavGraphNode(NavGraphNode&& other) noexcept : id(other.id), position(std::move(other.position))
		{}

		NavGraphNode& operator=(const NavGraphNode& other)
		{
			if (this == &other) return *this;
			id = other.id;
			position = other.position;
			return *this;
		}

		NavGraphNode& operator=(NavGraphNode&& other) noexcept
		{
			if (this == &other) return *this;

			id = other.id;
			position = std::move(other.position);
			return *this;
		}

		friend bool operator==(const NavGraphNode& lhs, const NavGraphNode& rhs)
		{
			return lhs.id == rhs.id && lhs.position == rhs.position;
		}

		friend bool operator!=(const NavGraphNode& lhs, const NavGraphNode& rhs)
		{
			return !(lhs == rhs);
		}
	};

	struct NavGraphEdge
	{
		size_t id;
		size_t nodeFrom;
		size_t nodeTo;
		float weight;
		float width;

		NavGraphEdge() : id(-1), nodeFrom(-1), nodeTo(-1), weight(0), width(0)
		{}

		NavGraphEdge(size_t id, size_t nodeFrom, size_t nodeTo, float weight, float width) :
			id(id),
		    nodeFrom(nodeFrom),
		    nodeTo(nodeTo),
			weight(weight),
			width(width)
		{}

		NavGraphEdge(const NavGraphEdge& other)
			: id(other.id),
			  nodeFrom(other.nodeFrom),
			  nodeTo(other.nodeTo),
			  weight(other.weight),
			  width(other.width)
		{}

		NavGraphEdge(NavGraphEdge&& other) noexcept
			: id(other.id),
			  nodeFrom(other.nodeFrom),
			  nodeTo(other.nodeTo),
			  weight(other.weight),
			  width(other.width)
		{}

		NavGraphEdge& operator=(const NavGraphEdge& other)
		{
			if (this == &other)
				return *this;
			id = other.id;
			nodeFrom = other.nodeFrom;
			nodeTo = other.nodeTo;
			weight = other.weight;
			width = other.width;
			return *this;
		}

		NavGraphEdge& operator=(NavGraphEdge&& other) noexcept
		{
			if (this == &other)
				return *this;
			id = other.id;
			nodeFrom = other.nodeFrom;
			nodeTo = other.nodeTo;
			weight = other.weight;
			width = other.width;
			return *this;
		}

		friend bool operator==(const NavGraphEdge& lhs, const NavGraphEdge& rhs)
		{
			return lhs.id == rhs.id
				&& lhs.nodeFrom == rhs.nodeFrom
				&& lhs.nodeTo == rhs.nodeTo
				&& lhs.weight == rhs.weight
				&& lhs.width == rhs.width;
		}

		friend bool operator!=(const NavGraphEdge& lhs, const NavGraphEdge& rhs)
		{
			return !(lhs == rhs);
		}
	};

	class NavGraph
	{
	public:
		static std::unique_ptr <NavGraph> LoadFromStream(std::istream & istream);
	public:
		NavGraph(std::vector<NavGraphNode> nodes, std::vector<NavGraphEdge> edges);

		const NavGraphNode & GetNode(size_t id) const;

		const size_t GetClosestNodeIdByPosition(DirectX::SimpleMath::Vector2 p, std::unordered_set<NavGraphNode> nodes);
		const DirectX::SimpleMath::Vector2 GetClosiestPointAndNodeId(DirectX::SimpleMath::Vector2 p, size_t& node_id);

		std::vector<NavGraphEdge> GetOutEdges(size_t fromNodeId) const;
		std::vector<NavGraphEdge> GetInEdges(size_t toNodeId) const;

		std::unordered_set<NavGraphNode> GetOutNeighbours(size_t fromNodeId) const;
		std::unordered_set<NavGraphNode> GetInNeighbours(size_t toNodeId) const;

		std::unordered_set<NavGraphNode> GetAllNodes() const;
	private:
		std::unordered_map<size_t, NavGraphNode> _nodes;

		std::unordered_map<size_t, std::vector<NavGraphEdge>> _outEdges;
		std::unordered_map<size_t, std::vector<NavGraphEdge>> _inEdges;
	};
}

namespace std {
    template<> struct hash<FusionCrowd::NavGraphNode>
    {
        size_t operator()(const FusionCrowd::NavGraphNode& node) const noexcept
        {
            return std::hash<size_t>()(node.id) + 31 * (std::hash<float>()(node.position.x) + 31 * std::hash<float>()(node.position.y));
        }
    };

	template<> struct hash<FusionCrowd::NavGraphEdge>
    {
        size_t operator()(const FusionCrowd::NavGraphEdge& edge) const noexcept
        {
        	auto sizetHasher = std::hash<size_t>();
        	auto floatHasher = std::hash<float>();

        	size_t result = sizetHasher(edge.id);
        	result = 31 * result + sizetHasher(edge.nodeFrom);
        	result = 31 * result + sizetHasher(edge.nodeTo);
        	result = 31 * result + floatHasher(edge.weight);
        	result = 31 * result + floatHasher(edge.width);

            return result;
        }
    };
}

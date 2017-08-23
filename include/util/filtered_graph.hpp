#ifndef OSRM_UTIL_FILTERED_GRAPH_HPP
#define OSRM_UTIL_FILTERED_GRAPH_HPP

#include "storage/shared_memory_ownership.hpp"

#include "util/dynamic_graph.hpp"
#include "util/static_graph.hpp"
#include "util/vector_view.hpp"

#include <boost/range/adaptor/filtered.hpp>

namespace osrm
{
namespace util
{
namespace detail
{
template <typename GraphT, storage::Ownership Ownership> class FilteredGraphImpl;

// For static graphs we can save the filters as a static vector since
// we don't modify the structure of the graph. This also makes it easy to
// swap out the filter.
template <typename EdgeDataT, storage::Ownership Ownership>
class FilteredGraphImpl<util::StaticGraph<EdgeDataT, Ownership>, Ownership>
{
  public:
    using Graph = util::StaticGraph<EdgeDataT, Ownership>;
    using EdgeIterator = typename Graph::EdgeIterator;
    using NodeIterator = typename Graph::NodeIterator;
    template <typename T> using Vector = util::ViewOrVector<T, Ownership>;

    unsigned GetNumberOfNodes() const { return graph.GetNumberOfNodes(); }

    unsigned GetNumberOfEdges() const { return graph.GetNumberOfEdges(); }

    unsigned GetOutDegree(const NodeIterator n) const
    {
        auto range = graph.GetAdjacentEdgeRange(n);
        return std::count_if(range.begin(), range.end(), [this](const EdgeIterator edge) {
            return edge_filter[edge];
        });
    }

    inline NodeIterator GetTarget(const EdgeIterator e) const
    {
        BOOST_ASSERT(edge_filter[e]);
        return graph.GetTarget(e);
    }

    auto &GetEdgeData(const EdgeIterator e)
    {
        BOOST_ASSERT(edge_filter[e]);
        return graph.GetEdgeData(e);
    }

    const auto &GetEdgeData(const EdgeIterator e) const
    {
        BOOST_ASSERT(edge_filter[e]);
        return graph.GetEdgeData(e);
    }

    auto GetAdjacentEdgeRange(const NodeIterator n) const
    {
        return graph.GetAdjacentEdgeRange(n) |
               boost::adaptors::filtered(
                   [this](const EdgeIterator edge) { return edge_filter[edge]; });
    }

    // searches for a specific edge
    EdgeIterator FindEdge(const NodeIterator from, const NodeIterator to) const
    {
        for (const auto edge : GetAdjacentEdgeRange(from))
        {
            if (to == GetTarge(edge))
            {
                return edge;
            }
        }
        return SPECIAL_EDGEID;
    }

    template <typename FilterFunction>
    EdgeIterator
    FindSmallestEdge(const NodeIterator from, const NodeIterator to, FilterFunction &&filter) const
    {
        static_assert(traits::HasDataMember<typename Graph::EdgeArrayEntry>::value,
                      "Filtering on .data not possible without .data member attribute");

        EdgeIterator smallest_edge = SPECIAL_EDGEID;
        EdgeWeight smallest_weight = INVALID_EDGE_WEIGHT;
        for (auto edge : GetAdjacentEdgeRange(from))
        {
            const NodeID target = GetTarget(edge);
            const auto &data = GetEdgeData(edge);
            if (target == to && data.weight < smallest_weight &&
                std::forward<FilterFunction>(filter)(data))
            {
                smallest_edge = edge;
                smallest_weight = data.weight;
            }
        }
        return smallest_edge;
    }

    EdgeIterator FindEdgeInEitherDirection(const NodeIterator from, const NodeIterator to) const
    {
        EdgeIterator tmp = FindEdge(from, to);
        return (SPECIAL_NODEID != tmp ? tmp : FindEdge(to, from));
    }

    EdgeIterator
    FindEdgeIndicateIfReverse(const NodeIterator from, const NodeIterator to, bool &result) const
    {
        EdgeIterator current_iterator = FindEdge(from, to);
        if (SPECIAL_NODEID == current_iterator)
        {
            current_iterator = FindEdge(to, from);
            if (SPECIAL_NODEID != current_iterator)
            {
                result = true;
            }
        }
        return current_iterator;
    }

    FilteredGraphImpl(Graph graph, Vector<bool> edge_filter_)
        : graph(std::move(graph)), edge_filter(std::move(edge_filter_))
    {
        BOOST_ASSERT(edge_filter.size() == graph.GetNumberOfEdges());
    }

    // Takes a graph and a function that maps EdgeID to true
    // if the edge should be included in the graph.
    template <typename Pred>
    FilteredGraphImpl(Graph graph, Pred filter)
        : graph(std::move(graph)), edge_filter(graph.GetNumberOfEdges())
    {
        auto edge_ids = util::irange<EdgeID>(0, graph.GetNumberOfEdges());
        std::transform(edge_ids.begin(), edge_ids.end(), edge_filter.begin(), filter);
    }

    void Renumber(const std::vector<NodeID> &old_to_new_node)
    {
        graph.Renumber(old_to_new_node);
        // FIXME the edge filter needs to be renumbered with a different permutation
        // util::inplacePermutation(edge_filter.begin(), edge_filter.end(), old_to_new_node);
    }

  private:
    Graph graph;
    Vector<bool> edge_filter;
};

// For dynamic graphs we need to save the filter in the EdgeData, since edges
// get removed/added.
template <typename EdgeDataT>
class FilteredGraphImpl<util::DynamicGraph<EdgeDataT>, storage::Ownership::Container>
{
    struct EdgeDataWithFilter
    {
        EdgeDataWithFilter() = default;
        EdgeDataWithFilter(const EdgeDataT &data) : data(data), filter(false) {}

        EdgeDataWithFilter(EdgeDataT &&data) : data(std::move(data)), filter(false) {}

        EdgeDataT data;
        bool filter : 1;
    };

  public:
    using Graph = util::DynamicGraph<EdgeDataT>;
    using FilterGraph = util::DynamicGraph<EdgeDataWithFilter>;
    using EdgeIterator = typename FilterGraph::EdgeIterator;
    using NodeIterator = typename FilterGraph::NodeIterator;

    unsigned GetNumberOfNodes() const { return graph.GetNumberOfNodes(); }

    unsigned GetNumberOfEdges() const { return graph.GetNumberOfEdges(); }

    inline NodeIterator GetTarget(const EdgeIterator e) const
    {
        BOOST_ASSERT(edge_filter[e]);
        return graph.GetTarget(e);
    }

    auto &GetEdgeData(const EdgeIterator e)
    {
        BOOST_ASSERT(edge_filter[e]);
        return graph.GetEdgeData(e).data;
    }

    const auto &GetEdgeData(const EdgeIterator e) const
    {
        BOOST_ASSERT(edge_filter[e]);
        return graph.GetEdgeData(e).data;
    }

    auto GetAdjacentEdgeRange(const NodeIterator n) const
    {
        return graph.GetAdjacentEdgeRange(n) |
               boost::adaptors::filtered(
                   [this](const EdgeIterator edge) { return GetEdgeData(edge).filter; });
    }

    // searches for a specific edge
    EdgeIterator FindEdge(const NodeIterator from, const NodeIterator to) const
    {
        for (const auto edge : GetAdjacentEdgeRange(from))
        {
            if (to == GetTarge(edge))
            {
                return edge;
            }
        }
        return SPECIAL_EDGEID;
    }

    template <typename FilterFunction>
    EdgeIterator
    FindSmallestEdge(const NodeIterator from, const NodeIterator to, FilterFunction &&filter) const
    {
        static_assert(traits::HasDataMember<typename FilterGraph::EdgeArrayEntry>::value,
                      "Filtering on .data not possible without .data member attribute");

        EdgeIterator smallest_edge = SPECIAL_EDGEID;
        EdgeWeight smallest_weight = INVALID_EDGE_WEIGHT;
        for (auto edge : GetAdjacentEdgeRange(from))
        {
            const NodeID target = GetTarget(edge);
            const auto &data = GetEdgeData(edge);
            if (target == to && data.weight < smallest_weight &&
                std::forward<FilterFunction>(filter)(data))
            {
                smallest_edge = edge;
                smallest_weight = data.weight;
            }
        }
        return smallest_edge;
    }

    EdgeIterator FindEdgeInEitherDirection(const NodeIterator from, const NodeIterator to) const
    {
        EdgeIterator tmp = FindEdge(from, to);
        return (SPECIAL_NODEID != tmp ? tmp : FindEdge(to, from));
    }

    EdgeIterator
    FindEdgeIndicateIfReverse(const NodeIterator from, const NodeIterator to, bool &result) const
    {
        EdgeIterator current_iterator = FindEdge(from, to);
        if (SPECIAL_NODEID == current_iterator)
        {
            current_iterator = FindEdge(to, from);
            if (SPECIAL_NODEID != current_iterator)
            {
                result = true;
            }
        }
        return current_iterator;
    }

    template <typename Pred>
    FilteredGraphImpl(Graph graph_, Pred filter)
        : graph(std::move(graph_).template Transform<EdgeDataWithFilter>())
    {
        for (const auto node : irange<NodeID>(0, graph.GetNumberOfNodes()))
        {
            auto valid_start = filter(node);
            for (const auto edge : graph.GetAdjacentEdgeRange(node))
            {
                auto &data = graph.GetEdgeData(edge);
                auto valid_target = filter(graph.GetTarget(edge));
                data.filter = valid_start && valid_target;
            }
        }
    }

    void Renumber(const std::vector<NodeID> &old_to_new_node) { graph.Renumber(old_to_new_node); }

  private:
    FilterGraph graph;
};
}

template <typename GraphT>
using FilteredGraphContainer = detail::FilteredGraphImpl<GraphT, storage::Ownership::Container>;
template <typename GraphT>
using FilteredGraphView = detail::FilteredGraphImpl<GraphT, storage::Ownership::View>;
}
}

#endif

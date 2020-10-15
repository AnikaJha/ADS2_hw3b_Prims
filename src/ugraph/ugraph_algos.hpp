////////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief      Contains implementations of some algorithms for undirected graph.
/// \author     Sergey Shershakov
/// \version    0.1.0
/// \date       21.09.2020
/// \copyright  © Sergey Shershakov 2020.
///             This code is for educational purposes of the course "Algorithms
///             and Data Structures" provided by the Faculty of Computer Science
///             at the Higher School of Economics.
///
/// When altering code, a copyright line must be preserved.
///
////////////////////////////////////////////////////////////////////////////////


#ifndef UGRAPH_ALGOS_HPP
#define UGRAPH_ALGOS_HPP

#include <set>
#include <map>

#include "lbl_ugraph.hpp"

template<typename Vertex, typename EdgeLbl>
class PriorityQ
{
public:
    PriorityQ()
    : _verticesByWeights(), _mapByVertices()
    { }

    bool isEmpty() const
    {
        return _verticesByWeights.empty() && _mapByVertices.empty();
    }

    void insert(const Vertex& v, const EdgeLbl& w)
    {
        _verticesByWeights.insert(std::make_pair(w, v));
        _mapByVertices.insert(std::make_pair(v, w));
    }
    Vertex getMin()
    {
        return _verticesByWeights.begin() -> second;
    }

    void set(const Vertex& v, const EdgeLbl& w)
    {
            remove(v);
            insert(v, w);
    }
    void remove(const Vertex& v) {
        if (!isEmpty())
        {
            auto iter = _mapByVertices.find(v);
            if(iter != _mapByVertices.end())
            {
                _mapByVertices.erase(v);
                _verticesByWeights.erase({iter->second, v});
            }
        }

    }

    EdgeLbl getCost(const Vertex& v)
    {
        return _mapByVertices.find(v) -> second;
    }


private:
    std::set<std::pair<EdgeLbl,Vertex>> _verticesByWeights;
    std::multimap<Vertex, EdgeLbl> _mapByVertices;

};


/// Finds a MST for the given graph \a g using Prim's algorithm.
template<typename Vertex, typename EdgeLbl>
std::set<typename EdgeLblUGraph<Vertex, EdgeLbl>::Edge>
    findMSTPrim(const EdgeLblUGraph<Vertex, EdgeLbl>& g)
{
    PriorityQ<Vertex, EdgeLbl> pQ;

    std::set<Vertex> visited;

    for(auto itV = g.getVertices().first; itV != g.getVertices().second; ++itV)
    {
        pQ.insert(*itV, INT_MAX);
    }

    auto edgeIter = g.getEdges().first;

    pQ.set(edgeIter->first, 0);

    std::map<Vertex,Vertex> MST;

    while(!pQ.isEmpty())
    {
        Vertex minVertex = pQ.getMin();
        visited.insert(minVertex);
        auto itAdjPairEdges = g.getAdjEdges(minVertex);

        pQ.remove(minVertex);

        for(auto mapOfEdgCIt = itAdjPairEdges.first; mapOfEdgCIt != itAdjPairEdges.second; ++mapOfEdgCIt)
        {
            Vertex adjVert = mapOfEdgCIt -> second;
            EdgeLbl lbl;
            g.getLabel(minVertex, adjVert, lbl);

            if(pQ.getCost(adjVert) > lbl && visited.find(adjVert) == visited.end())
            {
                pQ.set(adjVert, lbl);

                if(MST.find(adjVert) == MST.end())
                    MST.insert(std::make_pair(adjVert, minVertex));

                else
                    MST.find(adjVert) -> second = minVertex;
            }
        }

    }

    std::set<typename EdgeLblUGraph<Vertex, EdgeLbl>::Edge> res;

    for(auto mstIt = MST.begin(); mstIt != MST.end(); ++mstIt)
    {
        res.insert({mstIt->first, mstIt->second});
    }


    return res;
}

/// Finds a MST for the given graph \a g using Kruskal's algorithm.
template<typename Vertex, typename EdgeLbl>
std::set<typename EdgeLblUGraph<Vertex, EdgeLbl>::Edge>
    findMSTKruskal(const EdgeLblUGraph<Vertex, EdgeLbl>& g)
{
    // TODO: implement this!

    std::set<typename EdgeLblUGraph<Vertex, EdgeLbl>::Edge> res;
    return res;
}


#endif // UGRAPH_ALGOS_HPP

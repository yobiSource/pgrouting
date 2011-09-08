/*
 * Shortest path algorithm for PostgreSQL
 *
 * Copyright (c) 2005 Sylvain Pasche
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include <boost/config.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <algorithm> //copy
#include <vector>

#include "dijkstra.h" //path_type_t

using namespace std;
using namespace boost;


struct Edge
{
  int id;
  float8 cost;
};

struct Vertex
{
  int id;
  int edge_id;
};


template <class G>
static void
graph_add_edge(G &graph, int id, int source, int target, float8 cost)
{
  typedef typename graph_traits<G>::edge_descriptor E;
  E e;
  bool inserted;

  if (cost < 0) // edges are not inserted in the graph if cost is negative
    return;

  tie(e, inserted) = add_edge(source, target, graph);

  graph[e].cost = cost;
  graph[e].id = id;
}

template <class G>
static int
get_edge_id(G &graph, int source, int target)
{
  typedef typename graph_traits<G>::edge_descriptor E;
  E e;
  bool found;

  tie(e, found) = boost::edge(source, target, graph);

  return found ? graph[e].id : -1;
}

template <class G>
static float8
get_edge_cost(G &graph, int source, int target)
{
  typedef typename graph_traits<G>::edge_descriptor E;
  E e;
  bool found;

  tie(e, found) = boost::edge(source, target, graph);

  return found ? graph[e].cost : -1;
}

int
boost_dijkstra_nodes(edge_t *edges, unsigned int count, int source_vertex_id,
                    double rdistance, bool directed, bool has_reverse_cost,
                    path_element_t **path, int *path_count, char **err_msg)
{
  typedef adjacency_list < listS, vecS, directedS, Vertex, Edge > graph_t;
  typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
  typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;

  graph_t graph;

  for (std::size_t j = 0; j < count; ++j)
  {
    edge_t& edge = edges[j];

    graph_add_edge(graph, edge.id, edge.source, edge.target, edge.cost);    

    if (!directed || (directed && has_reverse_cost))
    {
      if (has_reverse_cost)
      {
        graph_add_edge(graph, edge.id, edge.target, edge.source, edge.rcost);
      }
      else
      {
        graph_add_edge(graph, edge.id, edge.target, edge.source, edge.cost);
      }
    }
  }

  std::vector<vertex_descriptor> predecessors(num_vertices(graph));
  std::vector<float8> distances(num_vertices(graph));

  dijkstra_shortest_paths(graph, source_vertex_id,
                          predecessor_map(&predecessors[0])
                          .weight_map(get(&Edge::cost, graph))
                          .distance_map(&distances[0]));


  graph_traits < graph_t >::vertex_iterator vi, vend;
  vector<path_element_t> path_vector;

  for(tie(vi, vend) = vertices(graph); vi != vend; vi++) {

    if( (double)distances[*vi] <= rdistance ) {

      path_element_t pe;
      vertex_descriptor p;

      p = predecessors[*vi];

      pe.vertex_id = *vi;
      pe.parent_id = p;
      pe.edge_id   = get_edge_id(graph, p, *vi);
      pe.cost      = distances[*vi];

      path_vector.push_back( pe );
    }
  }

  if( path_vector.size() == 0 ) {
    *err_msg = (char *)"No path found";
    return 0;
  }

  *path = (path_element_t *) malloc( sizeof(path_element_t) *
                                     path_vector.size() );
  *path_count = path_vector.size();
  std::copy(path_vector.begin(), path_vector.end(), *path);

  return EXIT_SUCCESS;
}

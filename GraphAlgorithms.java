import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;

/**
 * Your implementation of various different graph algorithms.
 *
 * @author Aron Silberwasser
 * @userid asilberwasser3
 * @GTID 903683147
 * @version 1.0
 */
public class GraphAlgorithms {

    /**
     * Performs a breadth first search (bfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * You may import/use java.util.Set, java.util.List, java.util.Queue, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for BFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the bfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> bfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null || !graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("inputs cannot be null, and start must be a vertex in the graph");
        }

        Set<Vertex<T>> visited = new HashSet<>();
        Queue<Vertex<T>> q = new LinkedList<>();
        List<Vertex<T>> traversalList = new ArrayList<>();

        q.add(start);
        visited.add(start);

        while (!q.isEmpty()) {
            Vertex<T> v = q.remove();
            traversalList.add(v);

            for (VertexDistance<T> u : graph.getAdjList().get(v)) {
                if (!visited.contains(u.getVertex())) {
                    q.add(u.getVertex());
                    visited.add(u.getVertex());
                }
            }
        }

        return traversalList;
    }

    /**
     * Performs a depth first search (dfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * *NOTE* You MUST implement this method recursively, or else you will lose
     * all points for this method.
     *
     * You may import/use java.util.Set, java.util.List, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for DFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the dfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> dfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null || !graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("inputs cannot be null, and start must be a vertex in the graph");
        }

        Set<Vertex<T>> visited = new HashSet<>();
        List<Vertex<T>> traversalList = new ArrayList<>();

        dfsHelper(start, graph, visited, traversalList);

        return traversalList;
    }

    /**
     * recursive helper method to edit traversalList
     * @param currV current vertex
     * @param graph the graph being traversed
     * @param visited list of
     * @param traversalList final list of vertices
     * @param <T> the generic typing of the data
     */
    private static <T> void dfsHelper(Vertex<T> currV, Graph<T> graph,
                                      Set<Vertex<T>> visited, List<Vertex<T>> traversalList) {
        visited.add(currV);
        traversalList.add(currV);

        for (VertexDistance<T> u : graph.getAdjList().get(currV)) {
            if (!visited.contains(u.getVertex())) {
                dfsHelper(u.getVertex(), graph, visited, traversalList);
            }
        }
    }

    /**
     * Finds the single-source shortest distance between the start vertex and
     * all vertices given a weighted graph (you may assume non-negative edge
     * weights).
     *
     * Return a map of the shortest distances such that the key of each entry
     * is a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing
     * infinity) if no path exists.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Map, and java.util.Set and any class that
     * implements the aforementioned interfaces, as long as your use of it
     * is efficient as possible.
     *
     * You should implement the version of Dijkstra's where you use two
     * termination conditions in conjunction.
     *
     * 1) Check if all of the vertices have been visited.
     * 2) Check if the PQ is empty.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the Dijkstra's on (source)
     * @param graph the graph we are applying Dijkstra's to
     * @return a map of the shortest distances from start to every
     * other node in the graph
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("inputs cannot be null");
        }

        Set<Vertex<T>> visited = new HashSet<>();
        Map<Vertex<T>, Integer> distanceMap = new HashMap<>();
        Queue<VertexDistance<T>> pq = new PriorityQueue<>();

        int i = 0;
        for (Vertex<T> v : graph.getVertices()) {
            distanceMap.put(v, Integer.MAX_VALUE);
            if (!start.equals(v)) {
                i++; //if no vertex is equal to start, then i will be equal to size of vertex set: so throw exception
            }
        }
        if (i == graph.getVertices().size()) {
            throw new IllegalArgumentException("start must be a vertex in the graph");
        }

        pq.add(new VertexDistance<>(start, 0));
        distanceMap.put(start, 0);

        while (!pq.isEmpty() && visited.size() != graph.getVertices().size()) {
            VertexDistance<T> vd = pq.remove();

            if (!visited.contains(vd.getVertex())) {
                visited.add(vd.getVertex());
                distanceMap.put(vd.getVertex(), vd.getDistance());

                for (VertexDistance<T> u : graph.getAdjList().get(vd.getVertex())) {
                    if (!visited.contains(u.getVertex())) {
                        pq.add(new VertexDistance<>(u.getVertex(),
                                u.getDistance() + distanceMap.get(vd.getVertex())));
                    }
                }
            }
        }

        return distanceMap;
    }

    /**
     * Runs Prim's algorithm on the given graph and returns the Minimum
     * Spanning Tree (mst) in the form of a set of Edges. If the graph is
     * disconnected and therefore no valid mst exists, return null.
     *
     * You may assume that the passed in graph is undirected. In this framework,
     * this means that if (u, v, 3) is in the graph, then the opposite edge
     * (v, u, 3) will also be in the graph, though as a separate Edge object.
     *
     * The returned set of edges should form an undirected graph. This means
     * that every time you add an edge to your return set, you should add the
     * reverse edge to the set as well. This is for testing purposes. This
     * reverse edge does not need to be the one from the graph itself; you can
     * just make a new edge object representing the reverse edge.
     *
     * You may assume that there will only be one valid mst that can be formed.
     *
     * You should NOT allow self-loops or parallel edges in the mst.
     *
     * You may import/use PriorityQueue, java.util.Set, and any class that 
     * implements the aforementioned interface.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for this method (storing the adjacency list in a variable is fine).
     *
     * @param <T> the generic typing of the data
     * @param start the vertex to begin Prims on
     * @param graph the graph we are applying Prims to
     * @return the mst of the graph or null if there is no valid mst
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Set<Edge<T>> prims(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null || !graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("inputs cannot be null, and start must be a vertex in the graph");
        }

        Set<Vertex<T>> visited = new HashSet<>();
        Set<Edge<T>> mst = new HashSet<>();
        Queue<Edge<T>> pq = new PriorityQueue<>();

        visited.add(start);
        for (VertexDistance<T> vd : graph.getAdjList().get(start)) {
            pq.add(new Edge<>(start, vd.getVertex(), vd.getDistance()));
        }

        while (!pq.isEmpty() && visited.size() != graph.getVertices().size()) {
            Edge<T> e = pq.remove();
            if (!visited.contains(e.getV()) || !visited.contains(e.getU())) {
                mst.add(e);
                mst.add(new Edge<>(e.getV(), e.getU(), e.getWeight()));
                visited.add(e.getV());

                for (VertexDistance<T> vd : graph.getAdjList().get(e.getV())) {
                    if (!visited.contains(vd.getVertex())) {
                        pq.add(new Edge<>(e.getV(), vd.getVertex(), vd.getDistance()));
                    }
                }
            }
        }

        return mst;
    }
}
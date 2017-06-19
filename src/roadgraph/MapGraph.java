/**
 * @author UCSD MOOC development team and Eric Bruce
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

 
public class MapGraph {
	
	// member variables
	
	private HashMap <GeographicPoint, MapNode> mapGraphVertices;
			
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		mapGraphVertices = new HashMap<GeographicPoint, MapNode>();
	}
	
	// getters
	
	public HashMap <GeographicPoint, MapNode> getMapGraphVertices() {
		return mapGraphVertices;
	}

	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {

		return mapGraphVertices.values().size();
		
	}
	
	public void setMapGraphVertices(HashMap <GeographicPoint, MapNode> mapGraphVertices) {
		this.mapGraphVertices = mapGraphVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return mapGraphVertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		
		int totalNumEdges = 0;
		
		Iterator<MapNode> iterator = mapGraphVertices.values().iterator();
		while (iterator.hasNext()){
			MapNode mapNode = iterator.next();
			//System.out.println(mapNode.getLocation());
			//System.out.println("hey edges ="+mapNode.getEdges());
			if(null!=mapNode.getEdges()){
				//System.out.println("size="+mapNode.getEdges().size());
				totalNumEdges += mapNode.getEdges().size();
			} else {
				totalNumEdges +=0;
			}
		}
		return totalNumEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		boolean addVertexStatus = false;
		if (null!=location && !mapGraphVertices.containsKey(location)){
			MapNode mapNode = new MapNode();
			mapNode.setLocation(location);
			mapGraphVertices.put(location, mapNode);
			addVertexStatus=true;
		} else {
			addVertexStatus=false;
		}
		return addVertexStatus;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param roadDist The length of the road, in mi
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		MapEdge edge = new MapEdge();
		if (null != from && null != to && null != roadName && null != roadType) {
			edge.setStartLoc(from);
			edge.setGoalLoc(to);
			edge.setRoadName(roadName);
			edge.setRoadType(roadType);
			edge.setRoadDist(length);
			if (mapGraphVertices.containsKey(from)) {
				Set<MapEdge> edges = mapGraphVertices.get(from).getEdges();
				if (null!=edges){
					edges.add(edge);
				} else {
					edges = new HashSet<MapEdge>();
					edges.add(edge);
				}
				mapGraphVertices.get(from).setMapEdges(edges);
			}
		}
	}
	
	
	public List<GeographicPoint> getNeighbors(GeographicPoint location) {
		List <GeographicPoint> neighborGeoLocs = new ArrayList<GeographicPoint>();
		if (mapGraphVertices.containsKey(location)){
			if (null!=mapGraphVertices.get(location).getEdges()&& mapGraphVertices.get(location).getEdges().size()>0){
				for (MapEdge mapEdge : mapGraphVertices.get(location).getEdges()) {
					neighborGeoLocs.add(mapEdge.getGoalLoc());
				}
			}
		}
		return neighborGeoLocs;
	}
	
	
	
	/**********************************  BFS  ***********************************************/

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        //System.out.println("calling bfs now");
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)	{
		
		if (null != start && null != goal && mapGraphVertices.containsKey(start)
				&& mapGraphVertices.containsKey(goal)) {
		Map <GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		Set	<GeographicPoint> visitedNodeSet = new HashSet<GeographicPoint>();

		LinkedList<GeographicPoint> nodeQueueToExplore = new LinkedList<GeographicPoint>();
		List<GeographicPoint> neighborGeoLocs = new ArrayList<GeographicPoint>();
		//add start to queue
		nodeQueueToExplore.addLast(start);
		//adding start to visited nodes
		visitedNodeSet.add(start);	
		GeographicPoint currentLocation = null;
		
		while (!nodeQueueToExplore.isEmpty()){
			//queue loop starts
			//initialize currentLocation to start
			currentLocation = nodeQueueToExplore.poll();
			// Hook for visualization.
			nodeSearched.accept(currentLocation);
			if(currentLocation.equals(goal)){
				//System.out.println("i m going to break;");
				break;
			}
			//getting neighbors of currentLocation
			neighborGeoLocs = getNeighbors(currentLocation);
			//System.out.println(" Let's print the neigbors of "+ currentLocation);
			//for (GeographicPoint geographicPoint : neighborGeoLocs) {
				//System.out.println(geographicPoint.getX() + " " + geographicPoint.getY());
			//}

			//check if neighborNode n is not in the visitedSet then add n to queue
			for (GeographicPoint geographicPointNeighbor : neighborGeoLocs) {
				if(!visitedNodeSet.contains(geographicPointNeighbor)){
					// add new neighbor to visitedSet
					visitedNodeSet.add(geographicPointNeighbor);
					// add currentNode as new neighbor's parent in parentMap
					//System.out.println("Adding neighbor "+geographicPointNeighbor + " to parentMap");
					parentMap.put(geographicPointNeighbor, currentLocation);
					//enqueue new neighbor to the queue
					nodeQueueToExplore.addLast(geographicPointNeighbor);
				}
			}
			//System.out.println("reached the end of the queue loop....");
			/*for(Entry<GeographicPoint, GeographicPoint> geoPoint : parentMap.entrySet()){
				System.out.println(geoPoint.getKey() + " "+ geoPoint.getValue());
			}	*/		
			//queue loop ends
		}
		
		if (!currentLocation.equals(goal)){
			//System.out.println("returning null");
			return null;
		}
		//System.out.println("currentLocation!=goal="+(currentLocation!=goal));
		/*for(Entry<GeographicPoint, GeographicPoint> entry: parentMap.entrySet()){
			System.out.println("entry key ="+entry.getKey() + " entry value ="+entry.getValue());
		}*/
		 return buildRoutePath(parentMap, goal, start);
		} else {
			return null;
		}
	}
	/**
	 * This method searches for the shortest route on the bfs algorithm
	 * @param parentMapReceived
	 * @param goal
	 * @param start
	 * @return List <GeographicPoint>
	 */
	private List<GeographicPoint> buildRoutePath (Map<GeographicPoint, GeographicPoint> parentMapReceived,GeographicPoint goal, GeographicPoint start){
		//preparing a linked list of geographic locations
		//System.out.println("Did i reach here..");
		List <GeographicPoint> bfsRouteGeoLocList = new LinkedList <GeographicPoint>();
		//add goal to the route
		bfsRouteGeoLocList.add(goal);
		boolean startGeoLocFound = false;
		// setting the current location as the destination location to trace back to the source location
		GeographicPoint currentGeoLoc = goal;
		//System.out.println("startGeoLocFound ="+startGeoLocFound);
		//run the loop until the start location is found in the parentMapReceived
		while (startGeoLocFound != true) {
			//System.out.println(" I enter the dragon");
			//iterating through each of the elements of the parentMap
			for (Entry<GeographicPoint, GeographicPoint> entry : parentMapReceived.entrySet()) {
				//System.out.println("Enter the map...");
				// the current location is exactly whose parent I am tracing back right upto the start location
				if (currentGeoLoc.equals(entry.getKey())) {
					//System.out.println("TRUE entry.getKey() " + entry.getKey() + " currentGeoLoc ="+ currentGeoLoc );
					//setting the parent found as the current location to trace it's parent in next iteration of for loop
					currentGeoLoc = entry.getValue();
					//adding the parent found to the route list
					bfsRouteGeoLocList.add(currentGeoLoc);
				}
				if (currentGeoLoc.equals(start)) {
					// the current location is actually the start location so hurrah break
					//System.out.println(" currentGeoLoc == start =" + currentGeoLoc + " start =" +start);
					startGeoLocFound = true;
				}
				if(startGeoLocFound) break;
			}
		}
		//System.out.println(bfsRouteGeoLocList);
		Collections.reverse(bfsRouteGeoLocList);
		return bfsRouteGeoLocList;
	}
	
	/**********************************  BFS  ***********************************************/
		
	/** WEEK 4 BELOW **/
	
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}

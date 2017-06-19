/**
 * 
 * @author Eric Bruce
 * A class that represents a node (aka intersection where multiple roads meet) on the map
 * This class has two attributes 
 *   - location, latitude, longitude coordinates of the node/intersection
 *   - mapedges, roads/streets from/to the location
 *  
 */

package roadgraph;

import java.util.HashSet;
import java.util.Set;
import geography.GeographicPoint;


public class MapNode implements Comparable<MapNode> {

	// member variables initialization
	
	private GeographicPoint location;
	private Set<MapEdge> mapedges;
	private double DistFrmEnd = Double.MAX_VALUE;
	private double DistFrmStart = Double.MAX_VALUE;
	
	
	public MapNode() {
		location = new GeographicPoint(0,0);
		mapedges = new HashSet<MapEdge>();
	}
	
	
	// getters
	public GeographicPoint getLocation() {
		return location;
	}
	
	public Set<MapEdge> getEdges() {
		return mapedges;
	}
	
	public double getDistFrmEnd() {
		return DistFrmEnd;
	}
	
	public double getDistFrmStart() {
		return DistFrmStart;
	}
	
	// setters
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}
	
	public void setMapEdges(Set<MapEdge> mapedges) {
		this.mapedges = mapedges;
	}
	
	// Overrides
	@Override
	public int compareTo(MapNode node) {
		return ((Double)(this.DistFrmStart)).compareTo((Double)(node.getDistFrmStart()));
	}

	@Override
	public String toString() {
		return "MapNode [location=" + location + ", mapedges=" + mapedges + ", actualDurFrmEnd=" + DistFrmEnd
				+ ", DistFrmStart=" + DistFrmStart + "]";
	}
	
	
	
	
	
	
}

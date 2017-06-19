/**
 * 
 * @author Eric Bruce
 * A class that represents a MapEdge on the map
 * This class has these attributes 
 *   - startloc, latitude, longitude coordinates of the start node
 *   - goalloc, latitude, longitude coordinates of the goal node
 *   - roadname, name of the road
 *   - roadtype, ave, street, road
 *   - roaddist, in miles
 *  
 */
package roadgraph;

import geography.GeographicPoint;


public class MapEdge {
	
	// variables initialization
	private GeographicPoint startLoc;
	private GeographicPoint goalLoc;
	private String roadName = "";
	private String roadType ="";
	private double roadDist = 0;

	// initial constructor
	public MapEdge() {
		startLoc = new GeographicPoint(0, 0);
		goalLoc = new GeographicPoint(0, 0);
		roadName="";
		roadType="";
		roadDist=0;
	}
	
	// getters
	public GeographicPoint getStartLoc() {
		return startLoc;
	}
	
	public GeographicPoint getGoalLoc() {
		return goalLoc;
	}
	
	public String getRoadName() {
		return roadName;
	}
	
	public String getRoadType() {
		return roadType;
	}
	
	public Double getRoadDist() {
		return roadDist;
	}
	
	// setters
	public void setStartLoc(GeographicPoint startLoc) {
		this.startLoc = startLoc;
	}
	
	public void setGoalLoc(GeographicPoint goalLoc) {
		this.goalLoc = goalLoc;
	}
	
	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}
	
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	
	public void setRoadDist(double roadDist) {
		this.roadDist = roadDist;
	}	

}

package org.marssa.pathplanning.web_services.path_planning;
import java.nio.Buffer;

import mise.marssa.footprint.datatypes.composite.Coordinate;

import org.json.JSONObject;

public class Waypoint {

private String id = null;
private String name = null;
private Coordinate coordinate = null;

public Waypoint (String _id, String _name, Coordinate _coor)
{
	this.id = _id;
	this.name = _name;
	this.coordinate =_coor;
}

public String getId() {
	return id;
}

public void setId(String id) {
	this.id = id;
}

public Coordinate getCoordinate() {
	return coordinate;
}

public void setCoordinate(Coordinate coordinate) {
	this.coordinate = coordinate;
}

/**
 * @return Returns the name.
 */
public String getName() {
 return name;
}

/**
 * @param name
 *            The name to set.
 */
public void setName(String name) {
 this.name = name;
}

/**
 * Convert this object to a JSON object for representation
 */
public JSONObject toJSON() {
try{
 JSONObject jsonobj = new JSONObject();
 jsonobj.put("id", this.id);
 jsonobj.put("coordinate", this.coordinate);
 jsonobj.put("name", this.name);
 return jsonobj;
}catch(Exception e){
 return null;
}
}

/**
 * Convert this object to a string for representation
 */
public String toString() {
 StringBuffer sb = new StringBuffer();
 sb.append("id:");
 sb.append(this.id);
 sb.append("coordinate:");
 sb.append(this.coordinate);
 sb.append(",name:");
 sb.append(this.name);
 return sb.toString();
}
}
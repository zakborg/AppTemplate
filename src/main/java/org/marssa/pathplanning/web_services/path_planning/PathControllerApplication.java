package org.marssa.pathplanning.web_services.path_planning;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

import mise.marssa.footprint.datatypes.decimal.MDecimal;
import mise.marssa.footprint.exceptions.ConfigurationError;
import mise.marssa.footprint.exceptions.NoConnection;
import mise.marssa.footprint.exceptions.OutOfRange;
import mise.marssa.services.navigation.GpsReceiver;

import org.marssa.pathplanning.constants.Constants;
import org.marssa.pathplanning.control.electrical_motor.MotorController;
import org.marssa.pathplanning.control.path_planning.PathPlanningController;
import org.marssa.pathplanning.control.rudder.RudderController;
import org.restlet.Application;
import org.restlet.Request;
import org.restlet.Response;
import org.restlet.Restlet;
import org.restlet.data.CacheDirective;
import org.restlet.data.MediaType;
import org.restlet.data.Status;
import org.restlet.routing.Router;

public class PathControllerApplication extends Application {

	private ArrayList<CacheDirective> cacheDirectives;
	private MotorController motorController;
	private RudderController rudderController;
	private GpsReceiver gpsReceiver;
	private PathPlanningController pathPlanningController;
		private final ConcurrentMap<String, Waypoint> waypoints =   
            new ConcurrentHashMap<String, Waypoint>();  
	
	public PathControllerApplication(ArrayList<CacheDirective> cacheDirectives, MotorController motorController, 
			RudderController rudderController, GpsReceiver gpsReceiver, PathPlanningController pathPlanningController) {
		this.cacheDirectives = cacheDirectives;
		this.motorController = motorController;
		this.rudderController = rudderController;
		this.gpsReceiver = gpsReceiver;
		this.pathPlanningController =pathPlanningController;
	}

	
	public ConcurrentMap<String, Waypoint> getWaypoints() {  
		ArrayList<Waypoint> waypointList = new ArrayList<Waypoint>(waypoints.values());
		pathPlanningController.setPathList(waypointList);
		return waypoints;  
    }  
	
    /**
     * Creates a root Restlet that will receive all incoming calls.
     */
    @Override
    public synchronized Restlet createInboundRoot() {
        Router router = new Router(getContext());
         // Stop the Path following
        Restlet stopFollowing = new Restlet() {
        	@Override
            public void handle(Request request, Response response) {
        		response.setCacheDirectives(cacheDirectives);
        		try {
        			pathPlanningController.stopFollowingPath();
        			response.setEntity("The system has stopped following the path ", MediaType.TEXT_PLAIN);
        		} catch (NumberFormatException e) {
        			response.setStatus(Status.CLIENT_ERROR_BAD_REQUEST, "The value of the speed resource has an incorrect format");
        		} 
            }
        };
        
        // Start the path following
        Restlet startFollowing = new Restlet() {
        	@Override
            public void handle(Request request, Response response) {
        		response.setCacheDirectives(cacheDirectives);
        		try {
        			pathPlanningController.startFollowingPath();
        			response.setEntity("The system has started following the path ", MediaType.TEXT_PLAIN);
        		} catch (NumberFormatException e) {
        			response.setStatus(Status.CLIENT_ERROR_BAD_REQUEST, "The value of the speed resource has an incorrect format");
        		} 
            }
        };
        
              
                
        router.attach("/enterwaypoints", WayPointResource.class);
        router.attach("/startFollowing",startFollowing);
        router.attach("/stopFollowing",stopFollowing);
        
        return router;
    }
}
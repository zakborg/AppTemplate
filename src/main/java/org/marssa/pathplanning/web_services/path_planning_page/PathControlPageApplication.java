package org.marssa.pathplanning.web_services.path_planning_page;

import java.util.ArrayList;

import mise.marssa.footprint.datatypes.decimal.MDecimal;
import mise.marssa.footprint.exceptions.NoConnection;
import mise.marssa.services.navigation.GpsReceiver;

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

public class PathControlPageApplication extends Application {

	private ArrayList<CacheDirective> cacheDirectives;
	private MotorController motorController = null;
	private RudderController rudderController = null;
	private GpsReceiver gpsReceiver = null;
	private PathPlanningController pathPlanningController = null;
	
	public PathControlPageApplication(ArrayList<CacheDirective> cacheDirectives, MotorController motorController, RudderController rudderController,
			GpsReceiver gpsReceiver, PathPlanningController pathPlanningController) {
		this.cacheDirectives = cacheDirectives;
		this.motorController = motorController;
		this.rudderController = rudderController;
		this.gpsReceiver = gpsReceiver;
		this.pathPlanningController = pathPlanningController;
	}

    /**
     * Creates a root Restlet that will receive all incoming calls.
     */
    @Override
    public synchronized Restlet createInboundRoot() {
        Router router = new Router(getContext());
        
        // Create the navigation lights state handler
        Restlet rudderAndSpeedState = new Restlet() {
        	@Override
            public void handle(Request request, Response response) {
        		response.setCacheDirectives(cacheDirectives);
				try {
					MDecimal motorSpeed = motorController.getValue();
					MDecimal rudderAngle = rudderController.getAngle();
					response.setEntity("{\"motor\":" + motorSpeed.toJSON().getContents() + ",\"rudder\":" + rudderAngle.toJSON().getContents() + "}", MediaType.APPLICATION_JSON);
				} catch (NoConnection e) {
					response.setStatus(Status.SERVER_ERROR_INTERNAL, "No connection error has been returned");
					e.printStackTrace();
				}
    			
            }
        };
        
        //output all the data on one main page
        //rudder, motor, gps & pathplanning feedback.
        router.attach("/rudderAndSpeed", rudderAndSpeedState);
        
        return router;
    }
}
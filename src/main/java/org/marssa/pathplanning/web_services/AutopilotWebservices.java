package org.marssa.pathplanning.web_services;

import java.util.ArrayList;

import org.marssa.pathplanning.web_services.StaticFileServerApplication;
import org.marssa.pathplanning.constants.Constants;
import org.marssa.pathplanning.control.electrical_motor.MotorController;
import org.marssa.pathplanning.control.path_planning.PathPlanningController;
import org.marssa.pathplanning.control.rudder.RudderController;
import org.marssa.pathplanning.web_services.GPS_Receiver.GPSReceiverApplication;
import org.marssa.pathplanning.web_services.motionControlPage.MotionControlPageApplication;
import org.marssa.pathplanning.web_services.motor.MotorControllerApplication;
import org.marssa.pathplanning.web_services.path_planning.PathControllerApplication;
import org.marssa.pathplanning.web_services.path_planning_page.PathControlPageApplication;
import org.marssa.pathplanning.web_services.rudder.RudderControllerApplication;
import org.restlet.Component;
import org.restlet.Server;
import org.restlet.data.CacheDirective;
import org.restlet.data.Protocol;
import org.restlet.resource.ServerResource;
import mise.marssa.services.navigation.GpsReceiver;

/**
 * @author Clayton Tabone
 * 
 */
public class AutopilotWebservices extends ServerResource {

	private final ArrayList<CacheDirective> cacheDirectives = new ArrayList<CacheDirective>();

	// Create a new Component
	private Component component = new Component();

	/**
	 * @throws Exception
	 * 
	 */
	public AutopilotWebservices(MotorController motorController, RudderController rudderController,
			GpsReceiver gpsReceiver, PathPlanningController pathPlanningController) {
		// Set caching directives to noCache and noStore
		cacheDirectives.add(CacheDirective.noCache());
		cacheDirectives.add(CacheDirective.noStore());

		// Add a new HTTP server listening on the given port
		Server server = component.getServers().add(Protocol.HTTP,
				Constants.WEB_SERVICES.HOST.getContents(),
				Constants.WEB_SERVICES.PORT.intValue());
		server.getContext()
				.getParameters()
				.add("maxTotalConnections",
						Constants.WEB_SERVICES.MAX_TOTAL_CONNECTIONS.toString());

		// Add new client connector for the FILE protocol
		component.getClients().add(Protocol.FILE);

		// Attach the static file server application
		component.getDefaultHost()
				.attach("", new StaticFileServerApplication());

		// Attach the motor control application
		component.getDefaultHost()
				.attach("/motor",
						new MotorControllerApplication(cacheDirectives,
								motorController));

		// Attach the rudder control application
		component.getDefaultHost().attach(
				"/rudder",
				new RudderControllerApplication(cacheDirectives,
						rudderController));
		
		// Attach the Path Planner (Autopilot) application
				component.getDefaultHost().attach(
						"/pathPlanner",
						new PathControllerApplication(cacheDirectives,
								motorController, rudderController,gpsReceiver,pathPlanningController));
				
		// Attach the GPS receiver application
		component.getDefaultHost().attach("/gps",
				new GPSReceiverApplication(cacheDirectives, gpsReceiver));

		// Attach the motion control feedback application
		component.getDefaultHost().attach(
				"/motionControlPage",
				new MotionControlPageApplication(cacheDirectives,
						motorController, rudderController));
		
		// Attach the Path Planning control feedback application
				component.getDefaultHost().attach(
						"/pathPlanningControlPage",
						new PathControlPageApplication(cacheDirectives,
								motorController, rudderController,gpsReceiver,pathPlanningController));
	}

	public void start() throws Exception {
		// Start the component
		component.start();
	}

	public void stop() throws Throwable {
		finalize();
	}

	@Override
	protected void finalize() throws Throwable {
		super.finalize();
		component.stop();
	}
}
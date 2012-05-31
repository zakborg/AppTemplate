package autopilot;

import java.util.ArrayList;
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
			GpsReceiver gpsReceiver) {
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

		// Attach the GPS receiver application
		component.getDefaultHost().attach("/gps",
				new GPSReceiverApplication(cacheDirectives, gpsReceiver));

		// Attach the motion control feedback application
		component.getDefaultHost().attach(
				"/motionControlPage",
				new MotionControlPageApplication(cacheDirectives,
						motorController, rudderController));
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
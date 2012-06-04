package org.marssa.pathplanning;


import java.net.UnknownHostException;

import mise.marssa.footprint.exceptions.ConfigurationError;
import mise.marssa.footprint.exceptions.NoConnection;
import mise.marssa.footprint.exceptions.NoValue;
import mise.marssa.footprint.exceptions.OutOfRange;
import mise.marssa.services.diagnostics.daq.LabJackU3;
import mise.marssa.services.navigation.GpsReceiver;

import org.marssa.pathplanning.constants.Constants;
import org.marssa.pathplanning.control.electrical_motor.MotorController;
import org.marssa.pathplanning.control.path_planning.PathPlanningController;
import org.marssa.pathplanning.control.rudder.RudderController;
import org.marssa.pathplanning.web_services.AutopilotWebservices;
import org.slf4j.LoggerFactory;


import ch.qos.logback.classic.Logger;
public class Main {

	private static final Logger logger = (Logger) LoggerFactory
			.getLogger(Main.class);
	public static void main(String[] args) {
	
		LabJackU3 labJack = null;
		GpsReceiver gpsReceiver;
		AutopilotWebservices webServices;
		MotorController motorController;
		RudderController rudderController;
		PathPlanningController pathPlanningController;
		// Initialise LabJack
		try {
			logger.info("Initialising LabJack ...");
			labJack = LabJackU3.getInstance(Constants.LABJACK.HOST,
					Constants.LABJACK.PORT);
			logger.info("LabJack initialized successfully on {}:{}",
					Constants.LABJACK.HOST, Constants.LABJACK.PORT);
		} catch (Exception e) {
			logger.error("Failed to initialize LabJack", e);
			System.exit(1);
		}

		// Initialise Controllers and Receivers
		try {
			
			logger.info("Initialising motor controller ... ");
			motorController = new MotorController(labJack);
			logger.info("Motor controller initialised successfully");

			logger.info("Initialising rudder controller ... ");
			rudderController = new RudderController(labJack);
			logger.info("Rudder controller initialised successfully");	
            			
			logger.info("Initialising GPS receiver ... ");
			gpsReceiver = new GpsReceiver(Constants.GPS.HOST,
					Constants.GPS.PORT);
			logger.info("GPS receiver initialised successfully");
			

			logger.info("Initialising Path Planning controller ... ");
			pathPlanningController = new PathPlanningController(motorController,rudderController,gpsReceiver);
			logger.info("Path Planning controller initialised successfully");
			
			logger.info("Initialising web services ... ");
			webServices = new AutopilotWebservices( motorController,
					rudderController, gpsReceiver, pathPlanningController);
			logger.info("Web services initialised successfully");

			logger.info("Starting restlet web servicves ... ");
			webServices.start();
			logger.info("Web servicves started. Listening on {}:{}",
					Constants.GPS.HOST, Constants.GPS.PORT);
		} catch (ConfigurationError e) {
			logger.error("ConfigurationError exception has been caught", e);
			System.exit(1);
		} catch (OutOfRange e) {
			logger.error("OutOfRange exception has been caught", e);
			System.exit(1);
		} catch (UnknownHostException e) {
			logger.error("UnknownHostException exception has been caught", e);
			System.exit(1);
		} catch (NoConnection e) {
			logger.error("NoConnection exception has been caught", e);
			System.exit(1);
		} catch (InterruptedException e) {
			logger.error("InterruptedException exception has been caught", e);
			System.exit(1);
		} catch (NoValue e) {
			logger.error("NoValue exception has been caught", e);
			System.exit(1);
		} catch (Exception e) {
			logger.error("General exception has been caught", e);
			System.exit(1);
		}
		
	}

}

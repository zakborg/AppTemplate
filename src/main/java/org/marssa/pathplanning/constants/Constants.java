package org.marssa.pathplanning.constants;

import java.io.File;
import mise.marssa.footprint.datatypes.MString;
import mise.marssa.footprint.datatypes.decimal.MDecimal;
import mise.marssa.footprint.datatypes.integer.MInteger;

/**
 * @author Clayton Tabone
 *
 */
public class Constants {
	/**
	 * System Properties
	 * @author Clayton Tabone
	 */
	public final static class SYSTEM {
		public final static MString ENVIRONMENT = new MString(System.getProperty("mise.demonstrator.constants.environment", "production"));
		public final static MString MODBUS_DEBUG = new MString(System.getProperty("net.wimpi.modbus.debug", (ENVIRONMENT.getContents() == "development") ? "true" : "false"));
		public final static MString ROOT_URI = new MString((SYSTEM.ENVIRONMENT.getContents() == "production") ? "/root/demonstrator/theDemonStrator-Front-End" : new File(System.getProperty("mise.demonstrator.constants.workspace"), "theDemonStrator-Front-End").toString());
	}

	/**
	 * LabJack Constants
	 * @author Clayton Tabone
	 */
	public final static class LABJACK {
		public final static MString HOST = new MString("192.168.1.1");
		public final static MInteger PORT = new MInteger(5021);
	}

	/**
	 * Web Services Constants
	 * @author Clayton Tabone
	 */
	public final static class WEB_SERVICES {
		public final static MString HOST = new MString((SYSTEM.ENVIRONMENT.getContents() == "production") ? "192.168.1.1" : "localhost");
		public final static MInteger PORT = new MInteger(8182);
		public final static MInteger MAX_TOTAL_CONNECTIONS = new MInteger(50);
	}

	/**
	 * Ramping Constants
	 * @author Clayton Tabone
	 */
	public final static class RAMPING {
		public final static MInteger RETRY_INTERVAL = new MInteger(5);
	}

	/**
	 * Motor Constants
	 * @author Clayton Tabone
	 */
	public final static class MOTOR {
		public final static MDecimal STEP_SIZE = new MDecimal(10);
		public final static MDecimal MAX_VALUE = new MDecimal(100);
		public final static MDecimal MIN_VALUE = new MDecimal(-100);
	}

	/**
	 * Rudder Constants
	 * @author Clayton Tabone
	 */
	public final static class RUDDER {
		// TODO What exactly should RUDDER_DELAY represent?
		public final static MInteger RUDDER_DELAY = new MInteger(50);
		public final static MInteger ROTATIONS = new MInteger(5);
		public final static MInteger BIG_ROTATIONS = new MInteger(20);
	}
	
	public final static class PATH {
		public final static MDecimal  Path_Accuracy_Lower = new MDecimal(10);
		public final static MDecimal  Path_Accuracy_Upper = new MDecimal(20);
		
	}
	/**
	 * GPS Constants
	 * @author Clayton Tabone
	 */
	public final static class GPS {
		public final static MString HOST = new MString("192.168.1.1");
		public final static MInteger PORT = new MInteger(2947);
	}

}

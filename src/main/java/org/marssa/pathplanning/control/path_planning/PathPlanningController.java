package org.marssa.pathplanning.control.path_planning;

import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentMap;

import mise.marssa.footprint.datatypes.MBoolean;
import mise.marssa.footprint.datatypes.composite.Coordinate;
import mise.marssa.footprint.datatypes.decimal.DegreesDecimal;
import mise.marssa.footprint.datatypes.decimal.MDecimal;
import mise.marssa.footprint.datatypes.integer.MInteger;
import mise.marssa.footprint.exceptions.ConfigurationError;
import mise.marssa.footprint.exceptions.NoConnection;
import mise.marssa.footprint.exceptions.NoValue;
import mise.marssa.footprint.exceptions.OutOfRange;
import mise.marssa.footprint.interfaces.control.motor.IMotorController;
import mise.marssa.footprint.interfaces.control.rudder.IRudderController;
import mise.marssa.services.control.Ramping;
import mise.marssa.services.diagnostics.daq.LabJack;
import mise.marssa.services.diagnostics.daq.LabJackU3;
import mise.marssa.services.navigation.GpsReceiver;
import mise.marssa.services.scheduling.MTimer;
import mise.marssa.services.scheduling.MTimerTask;

import org.marssa.pathplanning.constants.Constants;
import org.marssa.pathplanning.control.electrical_motor.MotorController;
import org.marssa.pathplanning.control.rudder.RudderController;
import org.marssa.pathplanning.web_services.path_planning.Waypoint;

/**
 * @author Clayton Tabone
 * 
 */
public class PathPlanningController extends MTimerTask implements IMotorController,IRudderController{
	
	private static final MTimerTask MTimerTask = null;
	private MotorController motorController;
	private RudderController rudderController;
	private GpsReceiver gpsReceiver;
	private Ramping ramping;
	private final MInteger STEPPER1 = LabJack.FIO8_ADDR;
	private final MInteger STEPPER2 = LabJack.FIO9_ADDR;
	private final MInteger STEPPER3 = LabJack.FIO10_ADDR;
	private final MInteger STEPPER4 = LabJack.FIO11_ADDR;
	private final MBoolean HIGH = new MBoolean(true);
	private final MBoolean LOW = new MBoolean(false);

	private int stepRight = 0;
	private int stepLeft = 0;
	private double voltageDifference = 0;
	private double angleDifference = 0;
	private static MDecimal angle;
	
	private final MInteger MOTOR_0_DIRECTION = LabJack.FIO6_ADDR;
	private final MInteger MOTOR_1_DIRECTION = LabJack.FIO7_ADDR;
	private final MInteger STEP_DELAY = new MInteger(20);
	private final MDecimal STEP_SIZE = new MDecimal(1.0f);
 
	private Coordinate nextHeading;
	private int count = 0;
	private LabJack lj;
	ArrayList<Waypoint> wayPointList;
	MTimer timer;
	/**
	 * @throws ConfigurationError
	 * @throws OutOfRange
	 * @throws NoConnection
	 * 
	 */	
	public  PathPlanningController(MotorController motorController, 
			RudderController rudderController, GpsReceiver gpsReceiver)
	{
		this.motorController = motorController;
		this.rudderController = rudderController;
		this.gpsReceiver = gpsReceiver;
		timer = MTimer.getInstance();
		wayPointList = new ArrayList<Waypoint>();
	}
	
	public ArrayList<Waypoint> getPathList() {
		return wayPointList;
	}


	public void setPathList(ArrayList<Waypoint> wayPointList) {
		this.wayPointList = wayPointList;
	}


	public Coordinate getNextHeading() {
		return nextHeading;
	}
	
	public void setNextHeading(Coordinate nextHeading) {
		this.nextHeading = nextHeading;
	}
	
	// This method is called upon to drive the vessel in the right direction
	public void drive() throws NoConnection, NoValue, OutOfRange, InterruptedException {
		
		double currentHeading = gpsReceiver.getCOG().doubleValue();
		double targetHeading = determineHeading();
		double difference =  (currentHeading - targetHeading) * -1;
		
		//if the difference is minimal the system will enter this if statement and adjust the rudder slightly
		if (difference < Constants.PATH.Path_Accuracy_Lower.doubleValue())
		{
			rotateToCentre(); 
		}
		if ((difference >= Constants.PATH.Path_Accuracy_Lower.doubleValue()) && (difference <= Constants.PATH.Path_Accuracy_Upper.doubleValue()))
		{
			if (currentHeading < targetHeading)
			{
				rudderController.rotateMultiple(Constants.RUDDER.ROTATIONS, new MBoolean(true));
				//rotateToCentre(); //the rudders are brought back into the center after directing the vessel.
			}
			else
			{
				rudderController.rotateMultiple(Constants.RUDDER.ROTATIONS, new MBoolean(false));
				//rotateToCentre();
			}
		}
		//if the difference is large the system will enter this if statement and adjust the rudder a lot
		else if (difference > Constants.PATH.Path_Accuracy_Upper.doubleValue())
		{
			if (currentHeading < targetHeading)
			{
				rudderController.rotateMultiple(Constants.RUDDER.BIG_ROTATIONS , new MBoolean(true));
				//rotateToCentre();
			}
			else
			{
				rudderController.rotateMultiple(Constants.RUDDER.BIG_ROTATIONS , new MBoolean(false));
				//rotateToCentre();
			}
		}
		//calculate bearing
	}
	
	//This method is used to determine the bearing we should be on to reach the next way point
	public double determineHeading() throws NoConnection, NoValue, OutOfRange
	{
		Coordinate currentPosition = gpsReceiver.getCoordinate();
		
		//double dLat = Math.toRadians(nextHeading.getLatitude().getDMS().doubleValue() - currentPosition.getLatitude().getDMS().doubleValue());
		double dLon = Math.toRadians(nextHeading.getLongitude().getDMS().doubleValue() - currentPosition.getLongitude().getDMS().doubleValue());
		
		double y = Math.sin(dLon) * Math.cos(currentPosition.getLatitude().getDMS().doubleValue());
		double x = Math.cos(nextHeading.getLatitude().getDMS().doubleValue())*Math.sin(currentPosition.getLatitude().getDMS().doubleValue()) -
		        Math.sin(nextHeading.getLatitude().getDMS().doubleValue())*Math.cos(currentPosition.getLatitude().getDMS().doubleValue())*Math.cos(dLon);
		return Math.atan2(y, x);
	}
	
	//this method is used to determine if we have arrived at the next destination. This is calculated if the distance between our current position and
	//the target waypoint is less than 10 meters
	public boolean arrived() throws NoConnection, NoValue, OutOfRange
	{
		Coordinate currentPosition = gpsReceiver.getCoordinate();
		double radius = 6371; // km
		double dLat = Math.toRadians(nextHeading.getLatitude().getDMS().doubleValue() - currentPosition.getLatitude().getDMS().doubleValue());
		double dLon = Math.toRadians(nextHeading.getLongitude().getDMS().doubleValue() - currentPosition.getLongitude().getDMS().doubleValue());
		double lat1 = Math.toRadians(nextHeading.getLatitude().getDMS().doubleValue());
		double lat2 = Math.toRadians(currentPosition.getLatitude().getDMS().doubleValue());
		
		double angle = Math.sin(dLat/2) * Math.sin(dLat/2) +
		        Math.sin(dLon/2) * Math.sin(dLon/2) * Math.cos(lat1) * Math.cos(lat2); 
		double c = 2 * Math.atan2(Math.sqrt(angle), Math.sqrt(1-angle)); 
		double distance = radius * c;
		
		if (distance < 0.01)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	//This method is used in order to check if the end of the trip has been reached, i.e there are no more way points in the list.
	public boolean endOfTrip()
	{
		if (count == wayPointList.size())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	//This method is called upon every 1 second by the timer event.
	public void run()
	{
		try {
			if (arrived() && endOfTrip()) //If we have arrived and its the end of the trip (no more way points)
			{
				motorController.rampTo(new MDecimal(0)); //we then kill the engines
			}
			else if (arrived() && ! endOfTrip()) //if we have arrived at our next way point but its not the end of the trip
			{
				count++;
				setNextHeading(wayPointList.get(count).getCoordinate()); //we get the next way points from the list and drive.
				drive();
			}
			else
			{
				drive();//else if we are on our way to the next way point we continue driving the vessel
			}
		} catch (NoConnection e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (NoValue e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (OutOfRange e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ConfigurationError e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	//this method is called upon by the RESTlet web services.
	public void startFollowingPath()
	{
		count =0;
		setNextHeading(wayPointList.get(count).getCoordinate()); //we set the next way point to the first in the list
		timer.addSchedule(this , 1000); //we create the timer schedule for every 1 sec.
	}
	
	//This method is called upon by the RESTlet web services.
	public void stopFollowingPath()
	{
		timer.cancel(); //this cancels the timer.
	}
	//Path Planning Controller
	//Motor Controller

	public void outputValue(MDecimal motorSpeed) throws ConfigurationError,
			OutOfRange, NoConnection {
		System.out.println(motorSpeed);
		MInteger actualValue = new MInteger((int) ((Math.pow(2, 32) - 1)
				* motorSpeed.abs().doubleValue() / 100.0));
		/*
		 * The Motor connected to TIMER_0 is slightly faster. Hence it is being
		 * scaled down to 90% of the requested value.
		 */
		lj.setTimerValue(LabJackU3.TimerU3.TIMER_0, new MInteger(
				(int) (actualValue.doubleValue() * 0.9)));
		lj.setTimerValue(LabJackU3.TimerU3.TIMER_1, actualValue);
	}

	public MDecimal getValue() {
		return ramping.getCurrentValue();
	}

	public void rampTo(MDecimal desiredValue) throws InterruptedException,
			ConfigurationError, OutOfRange {
		ramping.rampTo(desiredValue);
	}
    
	public void setPolaritySignal(Polarity polarity) throws NoConnection {
		switch (polarity) {
		case POSITIVE:
			lj.write(MOTOR_0_DIRECTION, new MBoolean(true));
			lj.write(MOTOR_1_DIRECTION, new MBoolean(true));
			break;
		case NEGATIVE:
			lj.write(MOTOR_0_DIRECTION, new MBoolean(false));
			lj.write(MOTOR_1_DIRECTION, new MBoolean(false));
			break;
		case OFF:
			// TODO handle the case to switch off the motors
			break;
		}
	}
	
	//Motor controller
	
	//Rudder Controller
	
	public void rotate(MBoolean direction) throws NoConnection,
	InterruptedException {
		if ((stepLeft == 0 && direction.getValue())
		|| (stepRight == 3 && direction.getValue() == false)) {
			lj.write(STEPPER1, HIGH);
			lj.write(STEPPER2, HIGH);
			lj.write(STEPPER3, LOW);
			lj.write(STEPPER4, LOW);
			stepLeft = 1;
			stepRight = 0;
			Thread.sleep(Constants.RUDDER.RUDDER_DELAY.intValue());
	return;
		}
		if ((stepLeft == 1 && direction.getValue())
			|| (stepRight == 2 && direction.getValue() == false)) {
			lj.write(STEPPER1, LOW);
			lj.write(STEPPER2, HIGH);
			lj.write(STEPPER3, HIGH);
			lj.write(STEPPER4, LOW);
			stepLeft = 2;
			stepRight = 3;
			Thread.sleep(Constants.RUDDER.RUDDER_DELAY.intValue());
	return;
		}
		if ((stepLeft == 2 && direction.getValue())
		|| (stepRight == 1 && direction.getValue() == false)) {
			lj.write(STEPPER1, LOW);
			lj.write(STEPPER2, LOW);
			lj.write(STEPPER3, HIGH);
			lj.write(STEPPER4, HIGH);
			stepLeft = 3;
			stepRight = 2;
			Thread.sleep(Constants.RUDDER.RUDDER_DELAY.intValue());
	return;
		}
		if ((stepLeft == 3 && direction.getValue())
		|| (stepRight == 0 && direction.getValue() == false)) {
			lj.write(STEPPER1, HIGH);
			lj.write(STEPPER2, LOW);
			lj.write(STEPPER3, LOW);
			lj.write(STEPPER4, HIGH);
			stepLeft = 0;
			stepRight = 1;
			Thread.sleep(Constants.RUDDER.RUDDER_DELAY.intValue());
			return;
		}
}

	public void rotateToCentre() throws NoConnection, InterruptedException {
		while (angle.doubleValue() > 5) {
			rotate(new MBoolean(false));
		}
		while (angle.doubleValue() < -5) {
			rotate(new MBoolean(true));
		}
	}
	
/**
* The getAngle returns the actual angle of the rudder
*/
	public MDecimal getAngle() throws NoConnection 
	{
		try {
			double voltageValue = lj.read(new MInteger(0), new MInteger(8),
			new MInteger(1)).doubleValue(); // value that needs to be
											// read from the labjack
			if (voltageValue < 2.45) {
				voltageDifference = 2.45 - voltageValue;
				angleDifference = voltageDifference * 57.14;
				angle = new MDecimal(angleDifference);
			}
			if (voltageValue > 2.45) {
				voltageDifference = voltageValue - 2.45;
				angleDifference = voltageDifference * 57.14;
				angle = new MDecimal(-angleDifference);
			}
			if (voltageValue == 2.45) {
				angle = new MDecimal(0);
			}
		} catch (IOException e) {
			throw new NoConnection("Cannot read from LabJack\n"
			+ e.getMessage(), e.getCause());
		}
		return angle;
	}
	
	
	//Rudder Controller
	
}
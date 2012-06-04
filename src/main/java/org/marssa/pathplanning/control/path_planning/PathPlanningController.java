package org.marssa.pathplanning.control.path_planning;

import java.io.IOException;

import org.marssa.pathplanning.constants.Constants;
import org.marssa.pathplanning.control.electrical_motor.MotorController;
import org.marssa.pathplanning.control.rudder.RudderController;

import mise.marssa.footprint.datatypes.MBoolean;
import mise.marssa.footprint.datatypes.decimal.MDecimal;
import mise.marssa.footprint.datatypes.integer.MInteger;
import mise.marssa.footprint.exceptions.ConfigurationError;
import mise.marssa.footprint.exceptions.NoConnection;
import mise.marssa.footprint.exceptions.OutOfRange;
import mise.marssa.footprint.interfaces.control.IController.Polarity;
import mise.marssa.footprint.interfaces.control.motor.IMotorController;
import mise.marssa.footprint.interfaces.control.rudder.IRudderController;
import mise.marssa.services.control.Ramping;
import mise.marssa.services.control.Ramping.RampingType;
import mise.marssa.services.diagnostics.daq.LabJack;
import mise.marssa.services.diagnostics.daq.LabJackU3;
import mise.marssa.services.diagnostics.daq.LabJackU3.TimerConfigModeU3;
import mise.marssa.services.navigation.GpsReceiver;

/**
 * @author Clayton Tabone
 * 
 */
public class PathPlanningController implements IMotorController,IRudderController {
	
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

	private LabJack lj;

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
	}
	
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
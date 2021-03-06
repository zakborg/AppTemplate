package org.marssa.pathplanning.control.electrical_motor;

import org.marssa.pathplanning.constants.Constants;

import mise.marssa.footprint.datatypes.MBoolean;
import mise.marssa.footprint.datatypes.decimal.MDecimal;
import mise.marssa.footprint.datatypes.integer.MInteger;
import mise.marssa.footprint.exceptions.ConfigurationError;
import mise.marssa.footprint.exceptions.NoConnection;
import mise.marssa.footprint.exceptions.OutOfRange;
import mise.marssa.footprint.interfaces.control.motor.IMotorController;
import mise.marssa.services.control.Ramping;
import mise.marssa.services.control.Ramping.RampingType;
import mise.marssa.services.diagnostics.daq.LabJack;
import mise.marssa.services.diagnostics.daq.LabJackU3;
import mise.marssa.services.diagnostics.daq.LabJackU3.TimerConfigModeU3;

/**
 * @author Clayton Tabone
 * 
 */
public class MotorController implements IMotorController {
	private final MInteger MOTOR_0_DIRECTION = LabJack.FIO6_ADDR;
	private final MInteger MOTOR_1_DIRECTION = LabJack.FIO7_ADDR;
	private final MInteger STEP_DELAY = new MInteger(20);
	private final MDecimal STEP_SIZE = new MDecimal(1.0f);
	private LabJackU3 lj;
	private Ramping ramping;

	/**
	 * @throws ConfigurationError
	 * @throws OutOfRange
	 * @throws NoConnection
	 * 
	 */
	public MotorController(LabJackU3 lj) throws ConfigurationError, OutOfRange,
			NoConnection {
		this.lj = lj;
		lj.setTimerMode(LabJackU3.TimerU3.TIMER_0,
				TimerConfigModeU3.PWM_OUTPUT_16BIT);
		lj.setTimerMode(LabJackU3.TimerU3.TIMER_1,
				TimerConfigModeU3.PWM_OUTPUT_16BIT);
		lj.setTimerBaseClock(LabJackU3.TimerBaseClockU3.CLOCK_4_MHZ_DIVISOR);
		lj.setTimerClockDivisor(new MInteger(2));
		lj.setTimerValue(LabJackU3.TimerU3.TIMER_0,
				new MInteger((int) Math.pow(2, 32) - 1));
		lj.setTimerValue(LabJackU3.TimerU3.TIMER_1,
				new MInteger((int) Math.pow(2, 32) - 1));
		this.ramping = new Ramping(STEP_DELAY, STEP_SIZE, this,
				RampingType.ACCELERATED);
	}

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

	public MDecimal getValue() {
		return ramping.getCurrentValue();
	}

	public void rampTo(MDecimal desiredValue) throws InterruptedException,
			ConfigurationError, OutOfRange {
		ramping.rampTo(desiredValue);
	}

	public void increase(MDecimal incrementValue) throws InterruptedException,
			ConfigurationError, OutOfRange, NoConnection {
		double currentValue = this.ramping.getCurrentValue().doubleValue();
		if ((currentValue + Constants.MOTOR.STEP_SIZE.doubleValue()) > Constants.MOTOR.MAX_VALUE
				.doubleValue())
			this.rampTo(Constants.MOTOR.MAX_VALUE);
		else
			this.ramping.increase(incrementValue);
	}

	public void decrease(MDecimal decrementValue) throws InterruptedException,
			ConfigurationError, OutOfRange, NoConnection {
		double currentValue = this.ramping.getCurrentValue().doubleValue();
		if ((currentValue - Constants.MOTOR.STEP_SIZE.doubleValue()) < Constants.MOTOR.MIN_VALUE
				.doubleValue())
			this.rampTo(Constants.MOTOR.MIN_VALUE);
		else
			this.ramping.decrease(decrementValue);
	}
}
package redbacks.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import arachne.lib.io.GettableDouble;
import redbacks.robot.Constants;

public class ShooterHardware {
	// TODO Tune velocity pid
	private static final double
		VELOCITY_KP = 0.65,
		VELOCITY_KI = 0,
		VELOCITY_KD = 100,
		VELOCITY_KF = 0.05213;

	private static final double RAMP_TIME_SECONDS = 1; // Time for shooter to go from 0 to 1.

	public final WPI_TalonFX
		slaveMotor = new WPI_TalonFX(7), // Left side
		motor = new WPI_TalonFX(8); // Right side

	public final GettableDouble velocityTicksSensor = motor::getSelectedSensorVelocity;

	public void initialize() {
		motor.configFactoryDefault(Constants.CAN_TIMEOUT);
		slaveMotor.configFactoryDefault(Constants.CAN_TIMEOUT);

		slaveMotor.setInverted(true);
		slaveMotor.follow(motor);

		motor.setNeutralMode(NeutralMode.Coast);
		slaveMotor.setNeutralMode(NeutralMode.Coast);

		motor.configOpenloopRamp(RAMP_TIME_SECONDS);
		motor.configClosedloopRamp(RAMP_TIME_SECONDS);

		motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

		motor.selectProfileSlot(0, 0);
		motor.config_kP(0, VELOCITY_KP);
		motor.config_kI(0, VELOCITY_KI);
		motor.config_kD(0, VELOCITY_KD);
		motor.config_kF(0, VELOCITY_KF);
	}
}

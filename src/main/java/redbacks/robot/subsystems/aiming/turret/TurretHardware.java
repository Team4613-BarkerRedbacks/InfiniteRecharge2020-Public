package redbacks.robot.subsystems.aiming.turret;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.SettableDoubleSensor;
import redbacks.robot.Constants;

public class TurretHardware {
	private static final double
		POSITIONAL_KP = 0.5,
		POSITIONAL_KI = 0,
		POSITIONAL_KD = 0,
		MAX_PID_OUTPUT = 0.3,
		STARTING_ANGLE = 90;

	private static final double	ENCODER_TICKS_PER_DEGREE = 28418d / 180;

	static double ticksToDegrees(double ticks) {
		return ticks / ENCODER_TICKS_PER_DEGREE;
	}

	static double degreesToTicks(double degrees) {
		return degrees * ENCODER_TICKS_PER_DEGREE;
	}

	public final WPI_TalonFX motor = new WPI_TalonFX(6);

	private final TalonFXSensorCollection motorSensorCollection = motor.getSensorCollection();

	public final SettableDoubleSensor positionDegreesSensor = SettableDoubleSensor.create(
		() -> ticksToDegrees(motorSensorCollection.getIntegratedSensorPosition()),
		(value) -> motorSensorCollection.setIntegratedSensorPosition((int) degreesToTicks(value), Constants.CAN_TIMEOUT)
	);

	public final GettableDouble rotationalVelocityDegreesPerSecondSensor = () -> ticksToDegrees(motorSensorCollection.getIntegratedSensorVelocity() * 10);

	public void initialize() {
		motor.configFactoryDefault(Constants.CAN_TIMEOUT);

		motor.setNeutralMode(NeutralMode.Brake);
		motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        motor.selectProfileSlot(0, 0);
        motor.config_kP(0, POSITIONAL_KP, Constants.CAN_TIMEOUT);
        motor.config_kI(0, POSITIONAL_KI, Constants.CAN_TIMEOUT);
        motor.config_kD(0, POSITIONAL_KD, Constants.CAN_TIMEOUT);

        motor.configPeakOutputForward(MAX_PID_OUTPUT, Constants.CAN_TIMEOUT);
        motor.configPeakOutputReverse(-MAX_PID_OUTPUT, Constants.CAN_TIMEOUT);

		positionDegreesSensor.accept(STARTING_ANGLE);
	}
}

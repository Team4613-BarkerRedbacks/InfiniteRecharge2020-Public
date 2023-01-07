package redbacks.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.SettableBoolean;
import arachne.lib.io.sensors.SettableDoubleSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import redbacks.robot.Constants;

public class ClimberHardware {
	// Outputs
	public final WPI_TalonFX
		motor = new WPI_TalonFX(4), // Left side
		slaveMotor = new WPI_TalonFX(5); // Right side

	public final SettableBoolean
		unlockSolenoid = new Solenoid(5)::set,
		unlockPlatformSolenoid = new Solenoid(4)::set;

	// Inputs
	public final GettableBoolean atTopSensor = new DigitalInput(0)::get;

	private final TalonFXSensorCollection motorSensorCollection = motor.getSensorCollection();

	public final SettableDoubleSensor positionSensor = SettableDoubleSensor.create(
		motorSensorCollection::getIntegratedSensorPosition,
		(value) -> motorSensorCollection.setIntegratedSensorPosition((int) value, Constants.CAN_TIMEOUT)
	);

	public void initialize() {
		motor.configFactoryDefault(Constants.CAN_TIMEOUT);
		slaveMotor.configFactoryDefault(Constants.CAN_TIMEOUT);

		slaveMotor.follow(motor);

		motor.setNeutralMode(NeutralMode.Brake);
		slaveMotor.setNeutralMode(NeutralMode.Brake);

		positionSensor.accept(0);
	}
}

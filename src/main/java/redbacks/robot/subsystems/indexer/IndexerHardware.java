package redbacks.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import arachne.lib.io.GettableBoolean;
import arachne.lib.io.SettableBoolean;
import arachne.lib.io.sensors.SettableDoubleSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import redbacks.robot.Constants;

public class IndexerHardware {
	// Outputs
	public final WPI_TalonFX motor = new WPI_TalonFX(22);
	public final SettableBoolean ptoSolenoid = new Solenoid(0)::set;
	public final SettableBoolean controlPanelWheelSolenoid = new Solenoid(1)::set;

	// Inputs
	public final GettableBoolean
		ballEnteringSensor = new DigitalInput(1)::get,
		ballAtTopSensor = new DigitalInput(2)::get;

	private final TalonFXSensorCollection feederMotorSensorCollection = motor.getSensorCollection();

	public final SettableDoubleSensor positionSensor = SettableDoubleSensor.create(
		() -> -feederMotorSensorCollection.getIntegratedSensorPosition(),
		(value) -> feederMotorSensorCollection.setIntegratedSensorPosition((int) value, Constants.CAN_TIMEOUT)
	);

	public void initialize() {
		motor.configFactoryDefault(Constants.CAN_TIMEOUT);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.setInverted(true);
	}
}

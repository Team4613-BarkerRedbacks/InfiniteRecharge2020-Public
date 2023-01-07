package redbacks.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import arachne.lib.io.SettableBoolean;
import edu.wpi.first.wpilibj.Solenoid;
import redbacks.robot.Constants;

public class IntakeHardware {
	public final WPI_TalonSRX rollerMotor = new WPI_TalonSRX(25);
	public final SettableBoolean deploySolenoid = new Solenoid(7)::set;

	public void initialize() {
		rollerMotor.configFactoryDefault(Constants.CAN_TIMEOUT);
		rollerMotor.setInverted(true);
		rollerMotor.setNeutralMode(NeutralMode.Brake);
	}
}

package redbacks.robot.subsystems.aiming.turret;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import redbacks.robot.subsystems.aiming.Aiming;

public class TurretBindings {
	public static void bind(Aiming aiming, TurretHardware hardware) {
		bindInputs(aiming, hardware);
		bindOutputs(aiming, hardware);
	}

	private static void bindInputs(Aiming aiming, TurretHardware hardware) {
		aiming.getTurretAngleSensor().populate(hardware.positionDegreesSensor);
		aiming.getTurretVelocitySensor().populate(hardware.rotationalVelocityDegreesPerSecondSensor);
	}

	private static void bindOutputs(Aiming aiming, TurretHardware hardware) {
		// FIXME Uncomment
//		aiming.getTurretPowerOutput().attach((value) -> hardware.motorTurret.set(TalonFXControlMode.PercentOutput, value));
		aiming.getTurretTargetAngleOutput().attach((value) -> hardware.motor.set(TalonFXControlMode.Position, TurretHardware.degreesToTicks(value)));
	}
}

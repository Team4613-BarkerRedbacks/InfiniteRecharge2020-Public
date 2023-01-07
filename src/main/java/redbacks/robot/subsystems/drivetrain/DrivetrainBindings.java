package redbacks.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import arachne.lib.scheduler.ScheduledSignal;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import redbacks.input.OperatorPanel.Button;
import redbacks.robot.Controllers;
import redbacks.robot.subsystems.drivetrain.Drivetrain.ModulePosition;

public class DrivetrainBindings {
	private static final double DEADZONE_SIZE = 0.1;

	public static void bind(Drivetrain drivetrain, DrivetrainHardware hardware, Controllers controllers) {
		bindControls(drivetrain, controllers);
		bindInputs(drivetrain, hardware);
		bindOutputs(drivetrain, hardware);
	}

	private static void bindControls(Drivetrain drivetrain, Controllers controllers) {
		drivetrain.addBinding(
			Drivetrain.SystemState.DRIVER_CONTROLLED,
			() -> drivetrain.fieldRelativeDrive(
				-applyJoystickDeadzone(controllers.driverRight.getY()),
				-applyJoystickDeadzone(controllers.driverRight.getX()),
				-applyJoystickDeadzone(controllers.driverLeft.getX())
			)
		);

		drivetrain
		.addBinding(new ScheduledSignal(controllers.operator::getStartButton))
		.attach(() -> drivetrain.setPosition(0, 0, new Rotation2d()));

		drivetrain
		.addBinding(new ScheduledSignal(() -> controllers.panel.getButton(Button.ZERO_HEADING)))
		.attach(() -> drivetrain.setPosition(0, 0, new Rotation2d()));
	}

	private static void bindInputs(Drivetrain drivetrain, DrivetrainHardware hardware) {
		drivetrain.getYawSensor().populate(hardware.yaw);

		drivetrain.getModuleAngleSensor(ModulePosition.LEFT_FRONT).populate(hardware.swerveModuleAngleLF);
		drivetrain.getModuleAngleSensor(ModulePosition.RIGHT_FRONT).populate(hardware.swerveModuleAngleRF);
		drivetrain.getModuleAngleSensor(ModulePosition.LEFT_BACK).populate(hardware.swerveModuleAngleLB);
		drivetrain.getModuleAngleSensor(ModulePosition.RIGHT_BACK).populate(hardware.swerveModuleAngleRB);

		drivetrain.getModuleVelocitySensor(ModulePosition.LEFT_FRONT).populate(hardware.swerveModuleVelocityLF);
		drivetrain.getModuleVelocitySensor(ModulePosition.RIGHT_FRONT).populate(hardware.swerveModuleVelocityRF);
		drivetrain.getModuleVelocitySensor(ModulePosition.LEFT_BACK).populate(hardware.swerveModuleVelocityLB);
		drivetrain.getModuleVelocitySensor(ModulePosition.RIGHT_BACK).populate(hardware.swerveModuleVelocityRB);

		drivetrain.getModuleDistanceSensor(ModulePosition.LEFT_FRONT).populate(hardware.swerveModuleDistanceLF);
		drivetrain.getModuleDistanceSensor(ModulePosition.RIGHT_FRONT).populate(hardware.swerveModuleDistanceRF);
		drivetrain.getModuleDistanceSensor(ModulePosition.LEFT_BACK).populate(hardware.swerveModuleDistanceLB);
		drivetrain.getModuleDistanceSensor(ModulePosition.RIGHT_BACK).populate(hardware.swerveModuleDistanceRB);
	}

	private static void bindOutputs(Drivetrain drivetrain, DrivetrainHardware hardware) {
 		drivetrain.getModuleSteerOutput(ModulePosition.LEFT_FRONT).attach(hardware.motorSteerLF::set);
 		drivetrain.getModuleTargetVelocityTicksPer100msOutput(ModulePosition.LEFT_FRONT).attach((encoderVelocity) -> hardware.motorDriveLF.set(TalonFXControlMode.Velocity, encoderVelocity));

 		drivetrain.getModuleSteerOutput(ModulePosition.RIGHT_FRONT).attach(hardware.motorSteerRF::set);
 		drivetrain.getModuleTargetVelocityTicksPer100msOutput(ModulePosition.RIGHT_FRONT).attach((encoderVelocity) -> hardware.motorDriveRF.set(TalonFXControlMode.Velocity, encoderVelocity));

 		drivetrain.getModuleSteerOutput(ModulePosition.LEFT_BACK).attach(hardware.motorSteerLB::set);
 		drivetrain.getModuleTargetVelocityTicksPer100msOutput(ModulePosition.LEFT_BACK).attach((encoderVelocity) -> hardware.motorDriveLB.set(TalonFXControlMode.Velocity, encoderVelocity));

 		drivetrain.getModuleSteerOutput(ModulePosition.RIGHT_BACK).attach(hardware.motorSteerRB::set);
 		drivetrain.getModuleTargetVelocityTicksPer100msOutput(ModulePosition.RIGHT_BACK).attach((encoderVelocity) -> hardware.motorDriveRB.set(TalonFXControlMode.Velocity, encoderVelocity));
	}

	// TODO Make translational deadzone dependent on hypotenuse length
	private static double applyJoystickDeadzone(double value) {
		if(Math.abs(value) < DEADZONE_SIZE) return 0;

		if(value > 0) return (value - DEADZONE_SIZE) / (1 - DEADZONE_SIZE);
		else return (value + DEADZONE_SIZE) / (1 - DEADZONE_SIZE);
	}
}

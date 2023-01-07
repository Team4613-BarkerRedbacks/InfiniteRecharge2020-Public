package redbacks.robot.subsystems.aiming;

import arachne.lib.scheduler.ScheduledSignal;
import arachne.lib.scheduler.ScheduledSource;
import arachne.lib.types.Directions2D;
import edu.wpi.first.wpilibj.GenericHID;
import redbacks.robot.Controllers;
import redbacks.robot.subsystems.aiming.hood.HoodBindings;
import redbacks.robot.subsystems.aiming.turret.TurretBindings;

public class AimingBindings {
	public static void bind(Aiming aiming, AimingHardware hardware, Controllers controllers) {
		bindControls(aiming, controllers);
		bindInputs(aiming, hardware);
		bindOutputs(aiming, hardware);

		HoodBindings.bind(aiming, hardware.hood);
		TurretBindings.bind(aiming, hardware.turret);
	}

	private static void bindControls(Aiming aiming, Controllers controllers) {
		aiming
		.addBinding(new ScheduledSource<Directions2D>(
			new Directions2D(),
			() -> Directions2D.fromAngle(controllers.operator.getPOV())
		)).attach(aiming::move);

		aiming
		.addBinding(new ScheduledSignal(controllers.operator::getBackButton))
		.attach(aiming::calculateRobotPosition);

		aiming.addBinding(new ScheduledSignal(() -> controllers.operator.getBumper(GenericHID.Hand.kRight)))
		.attach(aiming::aimFromTrench);
		
		aiming.addBinding(new ScheduledSignal(controllers.operator::getYButton))
		.attach(aiming::aimFromClose);
	}

	private static void bindInputs(Aiming aiming, AimingHardware hardware) {
		// FIXME Uncomment
//		aiming.getLimelightHasTargetSensor().populate(hardware.limelight::hasTarget);
		aiming.getLimelightCornerPixelPositionsSensor().populate(hardware.limelight::getTargetCornerPixelPositions);

//		aiming.getShooterSpeedSensor().populate(hardware.shooterVelocity);
	}

	private static void bindOutputs(Aiming aiming, AimingHardware hardware) {
		// FIXME Uncomment
//		aiming.getLimelightLedOutput().attach(hardware.limelight::setLedMode);
//		aiming.getLimelightPipelineOutput().attach(hardware.limelight::setPipeline);
	}
}

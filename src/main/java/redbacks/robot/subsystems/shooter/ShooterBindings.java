package redbacks.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import arachne.lib.scheduler.ScheduledBooleanSource;
import arachne.lib.scheduler.ScheduledSignal;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import redbacks.robot.Controllers;

public class ShooterBindings {
	public static void bind(Shooter shooter, ShooterHardware hardware, Controllers controllers) {
		bindControls(shooter, controllers);
		bindInputs(shooter, hardware);
		bindOutputs(shooter, hardware);
	}

	private static void bindControls(Shooter shooter, Controllers controllers) {
//		shooter
//		.addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.operator, Hand.kRight)))
//		.attach(shooter::spin);

		shooter
		.addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.operator, Hand.kRight)))
		.attach(shooter::spinWithPower);

		shooter
		.addBinding(new ScheduledBooleanSource(false, () -> controllers.operator.getBumper(Hand.kRight)))
		.attach(shooter::spinWithPower);
		
		shooter
		.addBinding(new ScheduledBooleanSource(false, controllers.operator::getYButton))
		.attach(shooter::spinFromClose);
	}

	private static void bindInputs(Shooter shooter, ShooterHardware hardware) {
		shooter.getSpeedSensor().populate(hardware.velocityTicksSensor);
	}

	private static void bindOutputs(Shooter shooter, ShooterHardware hardware) {
		shooter.getTargetSpeedOutput().attach((value) -> {
			if(value <= 1) hardware.motor.set(TalonFXControlMode.PercentOutput, value);
			else hardware.motor.set(TalonFXControlMode.Velocity, value);
		});
	}
}

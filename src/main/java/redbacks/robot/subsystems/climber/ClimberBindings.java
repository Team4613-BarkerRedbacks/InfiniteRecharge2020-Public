package redbacks.robot.subsystems.climber;

import arachne.lib.scheduler.ScheduledBooleanSource;
import arachne.lib.scheduler.ScheduledDoubleSource;
import arachne.lib.scheduler.ScheduledSignal;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import redbacks.input.OperatorPanel.Button;
import redbacks.robot.Controllers;

public class ClimberBindings {
	public static void bind(Climber climber, ClimberHardware hardware, Controllers controllers) {
		bindControls(climber, controllers);
		bindInputs(climber, hardware);
		bindOutputs(climber, hardware);
	}

	private static void bindControls(Climber climber, Controllers controllers) {
		climber
		.addBinding(new ScheduledDoubleSource(0, () -> controllers.operator.getY(Hand.kLeft)))
		.attach(climber::move);

		climber
		.addBinding(new ScheduledBooleanSource(false, () -> controllers.panel.getButton(Button.RAISE_HOOKS)))
		.attach((active) -> {
			if(active) climber.raiseHooks();
			else climber.setState(Climber.SystemState.NOT_CLIMBED);
		});

		climber
		.addBinding(new ScheduledSignal(() -> controllers.panel.getButton(Button.RAISE_ROBOT)))
		.attach(climber::raiseRobot);

		// climber
		// .addBinding(new ScheduledSignal(() -> controllers.climber.getStickButton(Hand.kRight)))
		// .attach(() -> climber.setState(Climber.SystemState.OPERATOR_CONTROL));

		// climber
		// .addBinding(new ScheduledBooleanSource(false, controllers.climber::getXButton))
		// .attach(climber::unlock);

		climber
		.addBinding(new ScheduledBooleanSource(false, () -> controllers.panel.getButton(Button.RELEASE_PLATFORM)))
		.attach(climber::deployPlatform);
	}

	private static void bindInputs(Climber climber, ClimberHardware hardware) {
		climber.getRobotAtMaxHeightSensor().populate(hardware.atTopSensor);
		climber.getPositionSensor().populate(hardware.positionSensor);
	}

	private static void bindOutputs(Climber climber, ClimberHardware hardware) {
		climber.getPowerOutput().attach(hardware.motor::set);

		climber.getUnlockOutput().attach(hardware.unlockSolenoid);
		climber.getPlatformUnlockOutput().attach(hardware.unlockPlatformSolenoid);
	}
}

package redbacks.robot.subsystems.intake;

import arachne.lib.scheduler.ScheduledBooleanSource;
import arachne.lib.scheduler.ScheduledSource;
import arachne.lib.types.InOut;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import redbacks.robot.Controllers;

public class IntakeBindings {
	public static void bind(Intake intake, IntakeHardware hardware, Controllers controllers) {
		bindControls(intake, controllers);
		bindOutputs(intake, hardware);
	}

	private static void bindControls(Intake intake, Controllers controllers) {
//		intake
//		.addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.operator, Hand.kLeft)))
//		.attach(intake::deploy);

		intake
		.addBinding(new ScheduledSource<InOut>(InOut.NONE, () ->
			  controllers.operator.getBumper(Hand.kLeft) ? InOut.OUT
			: Controllers.isTriggerPressed(controllers.operator, Hand.kLeft) ? InOut.IN
			: InOut.NONE
		)).attach(intake::spin);

		intake
		.addBinding(new ScheduledBooleanSource(false, () -> controllers.operator.getBumper(Hand.kLeft)))// Also controls indexer
		.attach((pressed) -> {
			if(pressed) {
				intake.spin(InOut.OUT);
				intake.deploy(true);
			}
			else {
				intake.spin(InOut.NONE);
				intake.deploy(false);
			}
		});

		intake
		.addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.operator, Hand.kLeft)))
		.attach((pressed) -> {
			if(pressed) {
				intake.spin(InOut.IN);
				intake.deploy(true);
			}
			else {
				intake.spin(InOut.NONE);
				intake.deploy(false);
			}
		});
	}

	private static void bindOutputs(Intake intake, IntakeHardware hardware) {
		intake.getDeployOutput().attach(hardware.deploySolenoid);
		intake.getRollerOutput().attach(hardware.rollerMotor::set);
	}
}

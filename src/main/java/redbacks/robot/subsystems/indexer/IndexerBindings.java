package redbacks.robot.subsystems.indexer;

import arachne.lib.scheduler.ScheduledBooleanSource;
import edu.wpi.first.wpilibj.GenericHID;
import redbacks.input.OperatorPanel.Button;
import redbacks.robot.Controllers;

public class IndexerBindings {
	public static void bind(Indexer indexer, IndexerHardware hardware, Controllers controllers) {
		bindControls(indexer, controllers);
		bindInputs(indexer, hardware);
		bindOutputs(indexer, hardware);
	}

	private static void bindControls(Indexer indexer, Controllers controllers) {
		indexer
		.addBinding(new ScheduledBooleanSource(false, controllers.operator::getAButton)) // Also controls intake
		.attach((pressed) -> {
			if(pressed) indexer.tryToShoot();
			else indexer.index();
		});

		indexer
		.addBinding(new ScheduledBooleanSource(false, () -> controllers.operator.getStickButton(GenericHID.Hand.kRight)))
		.attach((pressed) -> {
			if(pressed) indexer.eject();
			else indexer.index();
		});

		indexer
		.addBinding(new ScheduledBooleanSource(false, () -> controllers.operator.getStickButton(GenericHID.Hand.kLeft)))
		.attach((pressed) -> {
			if(pressed) indexer.spinControlPanel();
			else indexer.index();
		});

		indexer
		.addBinding(new ScheduledBooleanSource(false, () -> controllers.panel.getButton(Button.DISABLE_INDEXER)))
		.attach(indexer::setDisabled);
	}

	private static void bindInputs(Indexer indexer, IndexerHardware hardware) {
		indexer.getBallAtBottomInput().populate(hardware.ballEnteringSensor);
		indexer.getBallAtTopInput().populate(hardware.ballAtTopSensor);
		indexer.getPositionInput().populate(hardware.positionSensor);
	}

	private static void bindOutputs(Indexer indexer, IndexerHardware hardware) {
		indexer.getPowerOutput().attach(hardware.motor::set);
		indexer.getEnableTowerOutput().attach(hardware.ptoSolenoid);
		indexer.getEnableWheelOutput().attach(hardware.controlPanelWheelSolenoid);
	}
}

package redbacks.robot.subsystems.aiming.hood;

import redbacks.robot.subsystems.aiming.Aiming;
import arachne.lib.dashboard.Dashboard;

public class HoodBindings {
	public static void bind(Aiming aiming, HoodHardware hardware) {
		bindInputs(aiming, hardware);
		bindOutputs(aiming, hardware);
	}

	private static void bindInputs(Aiming aiming, HoodHardware hardware) {
		aiming.getHoodAngleSensor().populate(hardware.angleDegreesSensor);
	}

	private static void bindOutputs(Aiming aiming, HoodHardware hardware) {
		aiming.getHoodPowerOutput().attach((speed) -> hardware.hoodServo.set(-speed));
	}
}

package redbacks.robot.subsystems.aiming.hood;

import arachne.lib.io.GettableDouble;
import edu.wpi.first.wpilibj.AnalogInput;
import redbacks.lib.hardware.SPT5325LV;

public class HoodHardware {
	private static final double
		POT_MIN = 2.352,
		POT_MAX = 1.958,
		POT_RANGE = POT_MAX - POT_MIN,
		POT_SCALING_FACTOR = Hood.ANGLE_RANGE / POT_RANGE,
		POT_OFFSET = Hood.ANGLE_MAX - POT_SCALING_FACTOR * POT_MAX;

	public final SPT5325LV hoodServo = new SPT5325LV(0);

	private final AnalogInput mxpInput = new AnalogInput(4);

	public final GettableDouble angleDegreesSensor = () -> POT_SCALING_FACTOR * mxpInput.getVoltage() + POT_OFFSET;

	public void initialize() {}
}

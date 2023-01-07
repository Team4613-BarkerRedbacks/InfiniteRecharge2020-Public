package redbacks.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import redbacks.input.OperatorPanel;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Controllers {
	private static final double TRIGGER_DISTANCE_FOR_PRESS = 0.1;

	public final Joystick
		driverLeft = new Joystick(0),
		driverRight = new Joystick(1);

	public final XboxController operator = new XboxController(2);

	public final OperatorPanel panel = new OperatorPanel(3);

	public static final boolean isTriggerPressed(XboxController controller, Hand hand) {
		return controller.getTriggerAxis(hand) >= TRIGGER_DISTANCE_FOR_PRESS;
	}
}

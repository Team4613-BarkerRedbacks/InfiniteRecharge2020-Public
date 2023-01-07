package redbacks.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import static redbacks.robot.Translations.*;

public class TestTranslations {
	@Test
	void mapDeadzones() {
		final double E = 0.001;
		
		// Test deadzone
		assertEquals(applyJoystickDeadzone(0), 0, E);
		assertEquals(applyJoystickDeadzone(0.03), 0, E);
		assertEquals(applyJoystickDeadzone(-0.09), 0, E);
		
		// Test extremities
		assertEquals(applyJoystickDeadzone(0.1), 0, E);
		assertEquals(applyJoystickDeadzone(1), 1, E);

		assertEquals(applyJoystickDeadzone(-0.1), 0, E);
		assertEquals(applyJoystickDeadzone(-1), -1, E);
		
		// Test midrange
		assertEquals(applyJoystickDeadzone(0.55), 0.5, E);
		assertEquals(applyJoystickDeadzone(-0.28), -0.2, E);
	}
}

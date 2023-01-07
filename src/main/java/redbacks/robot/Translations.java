package redbacks.robot;

public class Translations {
	// ----------------------------------------
	// Analog controller inputs
	// ----------------------------------------
	
	// Constants
	private static final double
			DEADZONE_SIZE = 0.1,
			TRIGGER_MINIMUM_PRESS = 0.1;

	protected static double applyJoystickDeadzone(double value) {
		if(Math.abs(value) < DEADZONE_SIZE) return 0;
		
		if(value > 0) return (value - DEADZONE_SIZE) / (1 - DEADZONE_SIZE);
		else return (value + DEADZONE_SIZE) / (1 - DEADZONE_SIZE);
	}

	protected static boolean isTriggerPressed(double value) {
		return value >= TRIGGER_MINIMUM_PRESS;
	}
	
	// ----------------------------------------
	// Turret units
	// ----------------------------------------
	
	// Constants
	private static final double	TURRET_ENCODER_TICKS_PER_DEGREE = 28418d / 180;
	
	protected static double convertTurretTicksToDegrees(double ticks) {
		return ticks / TURRET_ENCODER_TICKS_PER_DEGREE;
	}
	
	protected static double convertTurretDegreesToTicks(double degrees) {
		return degrees * TURRET_ENCODER_TICKS_PER_DEGREE;
	}
}

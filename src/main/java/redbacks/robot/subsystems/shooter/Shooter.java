package redbacks.robot.subsystems.shooter;

import java.util.function.DoublePredicate;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.logic.DoubleComparison;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.systems.Subsystem;

public class Shooter extends Subsystem {
	// Outputs
	private final DoublePipe targetSpeedOutput;

	// Sensors
	private final GettableDoubleInput speedSensor;

	// Constants
	// TODO Should be in hardware
	private static final double
			TARGET_SPEED = 20000,
			TARGET_SPEED_TOLERANCE = 1000, // TODO Determine acceptable tolerance
			MIN_SPEED = 13000,
			TRENCH_SPEED = 20000,
			CLOSE_SPEED = 15000,
			TARGET_POWER = 1;

	private static final double
			GEAR_RATIO = 5d / 3,
			WHEEL_DIAMETER = 0.1,
			WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

	public static final double
			MIN_EXIT_VELOCITY = MIN_SPEED * GEAR_RATIO * WHEEL_CIRCUMFERENCE * 10 / 4096 / 2,
			TARGET_EXIT_VELOCITY = TARGET_SPEED * GEAR_RATIO * WHEEL_CIRCUMFERENCE * 10 / 4096 / 2;

	// Static helpers
	private static final DoublePredicate isAtTargetSpeed = DoubleComparison.within(TARGET_SPEED_TOLERANCE).of(TARGET_SPEED);
	private static final DoublePredicate isAboveMinimumSpeed = DoubleComparison.greaterThanOrEqualTo(MIN_SPEED);

	public Shooter() {
		this.targetSpeedOutput = new DoublePipe(0);

		this.speedSensor = new GettableDoubleInput();
	}

	@Override
	public void run(){
		super.run();

		Dashboard.getInstance().putNumber("Shooter speed", getSpeedSensor().get());
	}

	public void spin(boolean active) {
		targetSpeedOutput.accept(active ? TARGET_SPEED : 0);
	}

	public void spinWithPower(boolean active) {
		targetSpeedOutput.accept(active ? TARGET_POWER : 0);
	}

	public void spinFromTrench(boolean active) {
		targetSpeedOutput.accept(active ? TRENCH_SPEED : 0);
	}

	public void spinFromClose(boolean active) {
		targetSpeedOutput.accept(active ? CLOSE_SPEED : 0);
	}

	public boolean isAtTargetSpeed() {
		return isAtTargetSpeed.test(speedSensor.get());
	}

	public boolean isAboveMinimumSpeed() {
		return isAboveMinimumSpeed.test(speedSensor.get());
	}

	public double getExitVelocity() {
		return speedSensor.get() * GEAR_RATIO * WHEEL_CIRCUMFERENCE * 10 / 4096 / 2;
	}

	public DoubleSource getTargetSpeedOutput() {
		return targetSpeedOutput;
	}

	public GettableDoubleInput getSpeedSensor() {
		return speedSensor;
	}
}

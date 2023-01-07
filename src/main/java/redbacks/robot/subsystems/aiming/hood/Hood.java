package redbacks.robot.subsystems.aiming.hood;

import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.logic.ArachneMath;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;
import arachne.lib.types.Directions2D.Vertical;

public class Hood extends StatefulSubsystem<Hood.SystemState, Hood> {
	// Pipelines
	private final DoublePipe powerOutput;
	private final GettableDoubleInput angleSensor;

	// Constants
	// TODO Should be in hardware
	public static final double
			ANGLE_MIN = 20.7, // Angle of limelight, initial ball vector may differ
			ANGLE_MAX = 50.2,
			ANGLE_RANGE = ANGLE_MAX - ANGLE_MIN, // Should be ~30 degrees
			TRACKING_ANGLE_MAX = 32;

	private static final double
			TARGET_ANGLE_TOLERANCE = 1, // TODO Determine acceptable tolerance
			SPEED = 1,
			SEARCHING_ANGLE = (ANGLE_MIN + ANGLE_MAX) / 2; // TODO Change to account for new hood angles

	// Control variables
	private double targetAngle = SEARCHING_ANGLE;
	private boolean isTargetValid = true;

	public Hood() {
		super(SystemState.POSITION, SystemState.class);

		this.powerOutput = new DoublePipe(0);
		this.angleSensor = new GettableDoubleInput();
	}

	// States
	public enum SystemState implements State<SystemState, Hood> {
		POWER {
			@Override
			protected boolean isOnTarget(Hood hood) {
				return true;
			}
		},
		POSITION {
			@Override
			public void run(Hood hood) {
				super.run(hood);

				hood.powerOutput.accept(
					  isWithinTolerance(hood) ? 0
					: hood.angleSensor.get() > hood.targetAngle ? -SPEED
					: SPEED
				);
			}

			private boolean isWithinTolerance(Hood hood) {
				return Math.abs(hood.targetAngle - hood.angleSensor.get()) <= TARGET_ANGLE_TOLERANCE;
			}

			@Override
			protected boolean isOnTarget(Hood hood) {
				return isWithinTolerance(hood) && hood.isTargetValid;
			}
		};

		protected abstract boolean isOnTarget(Hood hood);
	}

	@Override
	protected Hood getSelf() {
		return this;
	}

	// Instructions
	public void move(Vertical direction) {
		setState(SystemState.POWER);

		powerOutput.accept(
			  direction == Vertical.UP ? SPEED
			: direction == Vertical.DOWN ? -SPEED
			: 0
		);
	}

	public void moveTo(double targetAngle) {
		moveToWithLimit(targetAngle, ANGLE_MAX);
	}

	public void moveToWithLimit(double targetAngle, double limitedMax) {
		setState(SystemState.POSITION);

		double boundedTarget = ArachneMath.inBounds(targetAngle, ANGLE_MIN, limitedMax);
		if(boundedTarget != targetAngle) isTargetValid = false;

		this.targetAngle = boundedTarget;
	}

	// Accessors
	public double getAngle() {
		return angleSensor.get();
	}

	public boolean isOnTarget() {
		return state.isOnTarget(this);
	}

	// Pipeline methods
	public DoubleSource getPowerOutput() {
		return powerOutput;
	}

	public Input<GettableDouble> getAngleSensor() {
		return angleSensor;
	}
}

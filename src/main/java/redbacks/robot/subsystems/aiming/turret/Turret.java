package redbacks.robot.subsystems.aiming.turret;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.logic.ArachneMath;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;
import arachne.lib.types.Directions2D.Horizontal;

public class Turret extends StatefulSubsystem<Turret.SystemState, Turret> {
	// Pipelines
	private final DoublePipe powerOutput, targetAngleOutput;
	private final GettableDoubleInput angleSensor;

	// Constants
	// TODO Should be in hardware
	private static final double
			ANGLE_MIN = -120,
			ANGLE_MAX = 120,
			TARGET_ANGLE_TOLERANCE = 1, // TODO Determine acceptable tolerance
			MANUAL_SPEED = 0.2;

	// Control variables
	private boolean isTargetValid = true;

	public Turret() {
		super(SystemState.POSITION, SystemState.class);

		this.powerOutput = new DoublePipe(0);
		this.targetAngleOutput = new DoublePipe(0);

		this.angleSensor = new GettableDoubleInput();
	}

	// States
	public enum SystemState implements State<SystemState, Turret> {
		POWER {
			@Override
			protected boolean isOnTarget(Turret turret) {
				return true;
			}
		},
		POSITION {
			@Override
			protected boolean isOnTarget(Turret turret) {
				return Math.abs(turret.targetAngleOutput.get() - turret.angleSensor.get()) <= TARGET_ANGLE_TOLERANCE
						&& turret.isTargetValid;
			}
		};

		protected abstract boolean isOnTarget(Turret turret);
	}

	@Override
	protected Turret getSelf() {
		return this;
	}

	// Instructions
	public void move(Horizontal direction) {
		setState(SystemState.POWER);

		powerOutput.accept(
			  direction == Horizontal.LEFT ? -MANUAL_SPEED
			: direction == Horizontal.RIGHT ? MANUAL_SPEED
			: 0
		);
	}

	public void moveUnrestricted(double targetAngle) {
		setState(SystemState.POSITION);

		targetAngleOutput.accept(targetAngle);
		Dashboard.getInstance().putNumber("Target turret angle", targetAngle);
	}

	public void moveTo(double targetAngle) {
		double boundedTarget = ArachneMath.inBounds(targetAngle, ANGLE_MIN, ANGLE_MAX);
		if(boundedTarget != targetAngle) isTargetValid = false;

		moveUnrestricted(boundedTarget);
	}

	@Override
	public void run() {
		super.run();

		Dashboard.getInstance().putNumber("Current turret angle", getAngle());
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

	public DoubleSource getTargetAngleOutput() {
		return targetAngleOutput;
	}

	public Input<GettableDouble> getAngleSensor() {
		return angleSensor;
	}
}

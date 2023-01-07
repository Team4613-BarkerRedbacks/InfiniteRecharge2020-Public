package redbacks.robot.subsystems.aiming;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.immutables.Pair;
import arachne.lib.io.Gettable;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.GettableInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.listeners.Signal;
import arachne.lib.maths.ImmutableVector2d;
import arachne.lib.maths.Vector3d;
import arachne.lib.maths.Vector3d.Axis;
import arachne.lib.physics.ProjectileMotion;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.Pipe;
import arachne.lib.pipeline.Source;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;
import arachne.lib.types.Directions2D;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import redbacks.lib.hardware.Limelight.LedMode;
import redbacks.lib.motion.MotionState2d;
import redbacks.robot.subsystems.aiming.hood.Hood;
import redbacks.robot.subsystems.aiming.turret.Turret;
import redbacks.robot.subsystems.shooter.Shooter;
import redbacks.robot.types.LimelightPipeline;

public class Aiming extends StatefulSubsystem<Aiming.SystemState, Aiming> {
	// Subsystems
	private final Turret turret;
	private final Hood hood;

	// Inputs
	private final Pipe<MotionState2d> robotStateInput;
	private final Signal calculatePositionSignal;

	// Outputs
	private final Pipe<LedMode> limelightLedOutput;
	private final Pipe<LimelightPipeline> limelightPipelineOutput;
	private final Pipe<Pose2d> robotPositionOutput;

	// Shooter state sensors
	private final GettableDoubleInput turretVelocitySensor, shooterExitVelocitySensor;

	// Limelight sensors
	private final GettableInput<double[]> limelightCornerPixelPositionsSensor;
	private final GettableBooleanInput limelightHasTargetSensor;

	// Positional constants
	private static final double
			SHOOTER_HEIGHT = 0.94,
			HOOD_PIVOT_HEIGHT = 0.92;

	private static final Translation2d
			ROBOT_TO_TURRET = new Translation2d(0.2, 0), // TODO Measure
			HOOD_PIVOT_TO_LIMELIGHT = new Translation2d(0.1, -0.03);

	private double getLimelightHeight() {
		return HOOD_PIVOT_HEIGHT + HOOD_PIVOT_TO_LIMELIGHT.rotateBy(Rotation2d.fromDegrees(hood.getAngle())).getY();
	}

	// Camera constants
	private static final double
			IMAGE_WIDTH = 320,
			IMAGE_HEIGHT = 240,
			CAMERA_X_CENTER = (IMAGE_WIDTH / 2) - 0.5,
			CAMERA_Y_CENTER = (IMAGE_HEIGHT / 2) - 0.5,

			CAMERA_FOV_WIDTH = Math.toRadians(59.6),
			CAMERA_FOV_HEIGHT = Math.toRadians(45.7),
			VIEWPORT_WIDTH = 2 * Math.tan(CAMERA_FOV_WIDTH / 2),
			VIEWPORT_HEIGHT = 2 * Math.tan(CAMERA_FOV_HEIGHT / 2);

	// Goal coordinates
	private static final double
			GOAL_X = 0,
			GOAL_Y = 2.36,
			GOAL_BOTTOM_HEIGHT = 2.11,
			GOAL_TOP_HEIGHT = 2.87,
			GOAL_CENTER_HEIGHT = (GOAL_TOP_HEIGHT + GOAL_BOTTOM_HEIGHT) / 2,
			INNER_GOAL_RECESS_DISTANCE = 0.745,
			TRENCH_TURRET_ANGLE = 82.5,
			TRENCH_HOOD_ANGLE = 20.7,
			CLOSE_TURRET_ANGLE = 180,
			CLOSE_HOOD_ANGLE = 50.7;

	private static final Vector3d
			OUTER_GOAL_CENTER = new Vector3d(
				GOAL_X,
				GOAL_Y,
				GOAL_CENTER_HEIGHT
			),
			INNER_GOAL_CENTER = new Vector3d(
				OUTER_GOAL_CENTER.getX() + INNER_GOAL_RECESS_DISTANCE,
				OUTER_GOAL_CENTER.getY(),
				OUTER_GOAL_CENTER.getZ()
			);

	// Control variables
	private boolean hasValidTarget = false;
	private double hoodAngleDuringTracking = Double.NaN;

	public static enum SystemState implements State<SystemState, Aiming> {
		MANUAL {
			@Override
			public void deconstructState(Aiming aiming) {
				// TODO determine ideal functionality with return to automation
			}

			@Override
			public boolean isOnTarget(Aiming aiming) {
				return true;
			}
		},
		SHOOT_WHILE_MOVING {
			private static final int N_TARGET_CORNERS = 4;

			@Override
			public boolean isOnTarget(Aiming aiming) {
				return aiming.hasValidTarget
						&& aiming.turret.isOnTarget()
						&& aiming.hood.isOnTarget();
			}

			@Override
			public void initialize(Aiming aiming) {
				aiming.robotStateInput.attachListener((oldState, newState) -> {
					if(aiming.state == this) updateTarget(aiming, newState);
				});

				aiming.calculatePositionSignal.attach(() -> {
					Pose2d position = calculatePositionFromLimelight(aiming);
					if(position != null) {
						aiming.robotPositionOutput.accept(position);
						aiming.setState(this);
					}
				});
			}

			private void updateTarget(Aiming aiming, MotionState2d odometry) {
				aiming.hasValidTarget = true;

				Rotation2d robotYaw = odometry.getPose().getRotation();

				// Get position and heading from odometry
				Translation2d position2d =
					odometry.getPose().getTranslation();
					// .plus(ROBOT_TO_TURRET.rotateBy(robotYaw));

				// Calculate offset from goal as a 3d vector
				Vector3d offset = OUTER_GOAL_CENTER.minus(new Vector3d(position2d.getX(), position2d.getY(), SHOOTER_HEIGHT));

				// Calculate rotational error
				Rotation2d yawOffset = new Rotation2d(Math.atan2(-offset.getY(), offset.getX()));

				// Calculate offset in vertical 2d plane
				ImmutableVector2d offset2d = new ImmutableVector2d(
						Math.sqrt(Math.pow(offset.getX(), 2) + Math.pow(offset.getY(), 2)),
						offset.getZ()
				);

				double launchVelocity = aiming.shooterExitVelocitySensor.get();

				if(launchVelocity < Shooter.MIN_EXIT_VELOCITY) {
					launchVelocity = Shooter.MIN_EXIT_VELOCITY;
					aiming.hasValidTarget = false;
				}

				// FIXME Remove
				launchVelocity = Shooter.TARGET_EXIT_VELOCITY;

				// Stationary code
//				// Set turret target to rotational error plus angle of the robot
//				aiming.turretTargetAngleOutput.accept(yawOffset.plus(odometry.getPose().getRotation()).getDegrees());
//
//				// Calculate hood angle from shooter speed and acceleration due to gravity
//				Rotation2d hoodAngle = ProjectileMotion.getStationaryLaunchAngle(offset2d, launchVelocity);
//
//				aiming.hoodTargetAngleOutput.accept(hoodAngle.getDegrees());

				// Moving code
				ImmutableVector2d relativeVelocity = getRelativeVelocity(aiming, odometry);

				Pair<Rotation2d, Rotation2d> rotationAndLaunchAngles = ProjectileMotion.getMovingRotationAndLaunchAngles(
						offset2d,
						launchVelocity,
						relativeVelocity.getX(),
						relativeVelocity.getY()
				);

				if(rotationAndLaunchAngles == null) {
					aiming.hasValidTarget = false;
					return;
				}

				aiming.turret.moveTo(
						yawOffset
						// Plus rather than minus due to differing direction of positive angle on turret
						.plus(robotYaw)
						 // As above
						.minus(rotationAndLaunchAngles.getFirst())
						.getDegrees()
				);

				if(!Double.isNaN(aiming.hoodAngleDuringTracking)) aiming.hood.moveTo(aiming.hoodAngleDuringTracking);
				else aiming.hood.moveToWithLimit(rotationAndLaunchAngles.getSecond().getDegrees(), Hood.TRACKING_ANGLE_MAX);
			}

			private Pose2d calculatePositionFromLimelight(Aiming aiming) {
				// FIXME Fix incorrect calculations when close
				// Due to different order of corners received?
				double[] limelightTargetCornerPixelPositions = aiming.limelightCornerPixelPositionsSensor.get();

				if(limelightTargetCornerPixelPositions == null) return null;
				if(limelightTargetCornerPixelPositions.length != N_TARGET_CORNERS * 2) return null;

				Vector3d[] initialVectors = new Vector3d[N_TARGET_CORNERS];

				for(int i = 0; i < N_TARGET_CORNERS; i++) {
					double nx = 1d / IMAGE_WIDTH * 2 * (limelightTargetCornerPixelPositions[2 * i] - CAMERA_X_CENTER);
					double ny = -1d / IMAGE_HEIGHT * 2 * (limelightTargetCornerPixelPositions[2 * i + 1] - CAMERA_Y_CENTER);

					double x = VIEWPORT_WIDTH / 2 * nx;
					double y = VIEWPORT_HEIGHT / 2 * ny;

					initialVectors[i] = new Vector3d(x, y, 1).rotate(Axis.Y, Axis.Z, Rotation2d.fromDegrees(-aiming.hood.getAngle()));
				}

				Vector3d[] cornerVectors = scaleCornerVectors(aiming, initialVectors);
//				for(int i = 0; i < N_TARGET_CORNERS; i++) Dashboard.getInstance().putString("cv" + i, cornerVectors[i].toString());

				Vector3d topLeftToRight = cornerVectors[1].minus(cornerVectors[0]);
				Vector3d bottomLeftToRight = cornerVectors[2].minus(cornerVectors[3]);
				Vector3d avgLeftToRight = Vector3d.avg(topLeftToRight, bottomLeftToRight);

				/*
				 * ---TTT---
				 * \   |
				 *  \  |
				 *   \a|
				 *    \|
				 *     R
				 */
				Rotation2d angleToWall = new Rotation2d(avgLeftToRight.getX(), -avgLeftToRight.getZ());

				Vector3d avg = Vector3d.avg(cornerVectors);

				Vector3d vectorToGoal = avg
						.rotate(Axis.X, Axis.Z, angleToWall)
						.withY(GOAL_CENTER_HEIGHT);

				// Translate to field coords
				vectorToGoal = new Vector3d(
					vectorToGoal.getZ(),
					-vectorToGoal.getX(),
					vectorToGoal.getY()
				);

				Vector3d fromOrigin = OUTER_GOAL_CENTER.minus(vectorToGoal);

				Translation2d robotPosition = new Translation2d(fromOrigin.getX(), fromOrigin.getY()).minus(ROBOT_TO_TURRET);
				Rotation2d robotYaw = angleToWall.plus(Rotation2d.fromDegrees(aiming.turret.getAngle()));

				return new Pose2d(robotPosition.getX(), robotPosition.getY(), robotYaw);
			}

			private Vector3d[] scaleCornerVectors(Aiming aiming, Vector3d... vectors) {
				Vector3d[] results = new Vector3d[vectors.length];

				double limelightHeight = aiming.getLimelightHeight();

				// Scale based on known heights
				for(int i = 0; i < vectors.length; i++) {
					results[i] = vectors[i].scale(
						((i <= 1 ? GOAL_CENTER_HEIGHT : GOAL_BOTTOM_HEIGHT) - limelightHeight) / vectors[i].getY()
					);
				}

				// TODO Scale based on known widths

				return results;
			}

			private ImmutableVector2d getRelativeVelocity(Aiming aiming, MotionState2d odometry) {
				Twist2d robotVelocity = odometry.getVelocity();
				Rotation2d robotYaw = odometry.getPose().getRotation();

				ImmutableVector2d fieldVelocity =
						new ImmutableVector2d(robotVelocity.dx, robotVelocity.dy)
						.rotate(robotYaw);

				Translation2d turretPosition =
						odometry.getPose().getTranslation()
						.plus(ROBOT_TO_TURRET.rotateBy(robotYaw));

				ImmutableVector2d vectorToGoal = new ImmutableVector2d(
						GOAL_X - turretPosition.getX(),
						GOAL_Y - turretPosition.getY()
				);

				// TODO Can this be done with a rotation instead?
				double velocityToGoal = fieldVelocity.scalarProjectOnto(vectorToGoal);
				double velocityAcrossGoal = fieldVelocity.scalarProjectOnto(vectorToGoal.rotate(Rotation2d.fromDegrees(90)));

				return new ImmutableVector2d(velocityToGoal, velocityAcrossGoal);
			}
		};

		public abstract boolean isOnTarget(Aiming aiming);
	}

	public Aiming() {
		super(SystemState.MANUAL, SystemState.class);

		this.turret = new Turret();
		this.turret.setParent(this);

		this.hood = new Hood();
		this.hood.setParent(this);

		this.robotStateInput = new Pipe<MotionState2d>(null);

		this.calculatePositionSignal = new Signal();

		this.limelightLedOutput = new Pipe<LedMode>(LedMode.ON);
		this.limelightPipelineOutput = new Pipe<LimelightPipeline>(null);
		this.robotPositionOutput = new Pipe<Pose2d>(null);

		this.turretVelocitySensor = new GettableDoubleInput();
		this.shooterExitVelocitySensor = new GettableDoubleInput();

		this.limelightCornerPixelPositionsSensor = new GettableInput<double[]>();
		this.limelightHasTargetSensor = new GettableBooleanInput();
	}

	@Override
	public void initialize() {
		super.initialize();

		turret.initialize();
		hood.initialize();
	}

	@Override
	public void run() {
		super.run();

		Dashboard.getInstance().putNumber("Turret angle", turret.getAngle());
		Dashboard.getInstance().putNumber("Hood angle", hood.getAngle());

		turret.run();
		hood.run();
	}

	@Override
	protected Aiming getSelf() {
		return this;
	}

	public void move(Directions2D directions) {
		setState(SystemState.MANUAL);

		turret.move(directions.getHorizontal());
		hood.move(directions.getVertical());
	}

	public void aimFromTrench() {
		setState(SystemState.MANUAL);

		turret.moveTo(TRENCH_TURRET_ANGLE);
		hood.moveTo(TRENCH_HOOD_ANGLE);
	}

	public void aimFromClose() {
		setState(SystemState.MANUAL);

		turret.moveUnrestricted(CLOSE_TURRET_ANGLE);
		hood.moveTo(CLOSE_HOOD_ANGLE);
	}

	public void calculateRobotPosition() {
		hoodAngleDuringTracking = Double.NaN;
		calculatePositionSignal.fire();
	}

	public void trackWithFixedHood(double fixedAngle) {
		hoodAngleDuringTracking = fixedAngle;
		setState(SystemState.SHOOT_WHILE_MOVING);
	}

	public void updateOdometry(MotionState2d odometry) {
		robotStateInput.accept(odometry);
	}

	public Source<Pose2d> getRobotPositionOutput() {
		return robotPositionOutput;
	}

	public DoubleSource getTurretTargetAngleOutput() {
		return turret.getTargetAngleOutput();
	}

	public DoubleSource getHoodPowerOutput() {
		return hood.getPowerOutput();
	}

	public Input<Gettable<double[]>> getLimelightCornerPixelPositionsSensor() {
		return limelightCornerPixelPositionsSensor;
	}

	public Input<GettableDouble> getTurretAngleSensor() {
		return turret.getAngleSensor();
	}

	public Input<GettableDouble> getTurretVelocitySensor() {
		return turretVelocitySensor;
	}

	public Input<GettableDouble> getHoodAngleSensor() {
		return hood.getAngleSensor();
	}

	public Input<GettableDouble> getShooterExitVelocitySensor() {
		return shooterExitVelocitySensor;
	}
}

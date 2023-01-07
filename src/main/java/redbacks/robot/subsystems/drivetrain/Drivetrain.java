package redbacks.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.io.sensors.SettableDoubleSensor;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.Pipe;
import arachne.lib.pipeline.Source;
import arachne.lib.sequences.actions.Action;
import arachne.lib.sequences.Actionable;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import redbacks.lib.motion.TrapezoidProfile;
import redbacks.lib.control.ProfiledDriveDirectionController;
import redbacks.lib.kinematics.SwerveDriveKinematics;
import redbacks.lib.kinematics.SwerveDriveOdometry;
import redbacks.lib.motion.MotionState2d;
import redbacks.robot.Robot;

/**
 * The kinematics based Swerve Drivetrain.
 *
 * @author Ben Schwarz
 */
public class Drivetrain extends StatefulSubsystem<Drivetrain.SystemState, Drivetrain> {
	// Outputs
	private final Pipe<MotionState2d> odometryOutput;

	// Sensors
	private final GettableDoubleInput yawSensor;

    // Systems
    private final SwerveModuleSet<ModulePosition> modules;

	// Control variables
	private final SwerveDriveOdometry odometry;
	private final SwerveDriveKinematics kinematics;

	// Constants
	private static final double
			DISTANCE_BETWEEN_MODULE_LENGTH = 0.5853,
			DISTANCE_BETWEEN_MODULE_WIDTH = 0.5853,
			MODULE_DISTANCE_FROM_CENTRE_LENGTH = DISTANCE_BETWEEN_MODULE_LENGTH / 2,
			MODULE_DISTANCE_FROM_CENTRE_WIDTH = DISTANCE_BETWEEN_MODULE_WIDTH / 2,
			MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC = 8.978,
            MAX_VELOCITY_METRES_PER_SEC = 4, // TODO Remove duplicate from Swerve Module
            MAX_ACCELERATION_METRES_PER_SECOND_SQUARED = MAX_VELOCITY_METRES_PER_SEC / 2, // Accelerate up to speed in 2 seconds
            MAX_ROTATIONAL_ACCELERATION_RADIANS_PER_SEC_SQUARED = MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC / 4; // Accelerate up to speed in 4 seconds

	private static final double
			STRAIGHT_KP = 0.95,
			STRAIGHT_KI = 0,
			STRAIGHT_KD = 0.001,
			ROTATIONAL_KP = 6,
			ROTATIONAL_KI = 0.7,
			ROTATIONAL_KD = 1;

    private static final Pose2d
            POSITION_TOLERANCE = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10)),
            VELOCITY_TOLERANCE = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10));

	private static final TrapezoidProfile.Constraints
			LINEAR_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY_METRES_PER_SEC, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED),
			ROTATION_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC, MAX_ROTATIONAL_ACCELERATION_RADIANS_PER_SEC_SQUARED);

    public enum ModulePosition {
        // Positive x values represent moving toward the front of the robot whereas
        // positive y values represent moving toward the left of the robot.
        LEFT_FRONT(MODULE_DISTANCE_FROM_CENTRE_WIDTH, MODULE_DISTANCE_FROM_CENTRE_LENGTH),
        RIGHT_FRONT(MODULE_DISTANCE_FROM_CENTRE_WIDTH, -MODULE_DISTANCE_FROM_CENTRE_LENGTH),
        LEFT_BACK(-MODULE_DISTANCE_FROM_CENTRE_WIDTH, MODULE_DISTANCE_FROM_CENTRE_LENGTH),
        RIGHT_BACK(-MODULE_DISTANCE_FROM_CENTRE_WIDTH, -MODULE_DISTANCE_FROM_CENTRE_LENGTH);

        private final Translation2d offset;

        private ModulePosition(double xOffset, double yOffset) {
            this.offset = new Translation2d(xOffset, yOffset);
        }

        private Translation2d getOffset() {
            return offset;
        }
    }

    public enum SystemState implements State<SystemState, Drivetrain> {
        DRIVER_CONTROLLED,
        KINEMATIC_AUTO
    }

    public Drivetrain() {
        super(SystemState.DRIVER_CONTROLLED, SystemState.class);

        this.odometryOutput = new Pipe<MotionState2d>(null);

        this.yawSensor = new GettableDoubleInput();

        this.modules = new SwerveModuleSet<Drivetrain.ModulePosition>(
                SwerveModule::new,
                ModulePosition.values(),
                ModulePosition::getOffset
        );

        this.modules.forEach((module) -> module.setParent(this));

        this.kinematics = new SwerveDriveKinematics(modules.getOffsets());
        this.odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d(), Timer.getFPGATimestamp());
    }

    @Override
    public void initialize() {
        super.initialize();

        modules.forEach(SwerveModule::initialize);
    }

    @Override
    public void run() {
        super.run();

        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.size()];

        modules.forEach((module, i) -> moduleStates[i] = new SwerveModuleState(module.getVelocity(), getCartesianYaw(module.getAngle())));
        odometry.updateMotionState(Timer.getFPGATimestamp(), getCartesianYaw(yawSensor.get()), moduleStates);

        odometryOutput.accept(odometry.getRobotState());
        modules.forEach(SwerveModule::run);

        MotionState2d currentState = odometry.getRobotState();
        Dashboard.getInstance().putNumber("Forward", currentState.getPose().getTranslation().getX());
        Dashboard.getInstance().putNumber("Left", currentState.getPose().getTranslation().getY());
        Dashboard.getInstance().putNumber("Rotation", currentState.getPose().getRotation().getDegrees());
    }

    @Override
    protected Drivetrain getSelf() {
        return this;
    }

    /**
     * Converts from the yaw the navX outputs to a value useful for the kinematics classes
     *
     * @param hardwareYaw
     * @return Yaw as a Rotation2d object
     */
    private Rotation2d getCartesianYaw(double hardwareYaw) {
        return Rotation2d.fromDegrees(-hardwareYaw);
    }

    /**
     * Converts a user provided field-relative set of joystick inputs into a robot-relative
     * ChassisSpeeds object.
     *
     * @param forward     The component of speed in the x direction relative to the field.
     *                    Positive x is away from your alliance wall.
     * @param left     The component of speed in the y direction relative to the field.
     *                    Positive y is to your left when standing behind the alliance wall.
     * @param rotate The angular rate of the robot.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
     */
    public void fieldRelativeDrive(double forward, double left, double rotate) {
        double vxMetersPerSecond = forward * MAX_VELOCITY_METRES_PER_SEC;
        double vyMetersPerSecond = left * MAX_VELOCITY_METRES_PER_SEC;
        double omegaRadiansPerSecond = rotate * MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC;

        feedModulesFromTargetVelocities(ChassisSpeeds.fromFieldRelativeSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                odometry.getPoseMeters().getRotation()
        ));
    }

    /**
     * Converts from a Chassis state to individual states for the modules.
     * This is responsible for providing each SwerveModule with it's angle and velocity.
     * It is not field relative but relative to the chassis.
     */
    private void feedModulesFromTargetVelocities(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        modules.forEach((module, i) -> {
            module.getTargetAngleInput().accept(-moduleStates[i].angle.getDegrees());
            module.getTargetVelocityMetresPerSecInput().accept(moduleStates[i].speedMetersPerSecond);
        });
    }

    /**
     * @param posX  The position in metres along long end of the field (The opposition alliance station is 0, yours is some negative number)
     * @param posY  The position in metres along short end of field (The right edge of the field is 0, the left is some positive number)
     * @param angle The angle in degrees of the robot (Facing the opposing alliance station is 0)
     */
    public void setPosition(double posX, double posY, Rotation2d angle) {
        Pose2d oldPose = odometry.getPoseMeters();

        if(Double.isNaN(posX)) posX = oldPose.getTranslation().getX();
        if(Double.isNaN(posY)) posY = oldPose.getTranslation().getY();
        if(angle == null) angle = oldPose.getRotation();

        odometry.resetPosition(new Pose2d(posX, posY, angle), getCartesianYaw(yawSensor.get()));
    }

    // ----------------------------------------
    // IO bindings
    // ----------------------------------------

    public Source<MotionState2d> getOdometryOutput() {
        return odometryOutput;
    }

    public DoubleSource getModuleSteerOutput(ModulePosition position) {
        return modules.getModule(position).getSteerOutput();
    }

    public DoubleSource getModuleTargetVelocityTicksPer100msOutput(ModulePosition position) {
        return modules.getModule(position).getTargetVelocityTicksPer100msOutput();
    }

	public Input<GettableDouble> getYawSensor() {
		return yawSensor;
	}

	public Input<GettableDouble> getModuleAngleSensor(ModulePosition position) {
		return modules.getModule(position).getAngleSensor();
	}

	public Input<GettableDouble> getModuleVelocitySensor(ModulePosition position) {
		return modules.getModule(position).getVelocitySensor();
	}

	public Input<SettableDoubleSensor> getModuleDistanceSensor(ModulePosition position) {
		return modules.getModule(position).getDistanceSensor();
	}

    // ----------------------------------------
    // Actionables
    // ----------------------------------------

    public Actionable doProfiledMoveTo(Pose2d endPos) {
        return doProfiledMoveTo(endPos, LINEAR_MOTION_CONSTRAINTS.maxVelocity);
    }

    public Actionable doProfiledMoveTo(Pose2d endPos, double maxLinearVelocityMetresPerSec) {
        return doProfiledMoveTo(endPos, new TrapezoidProfile.Constraints(
                maxLinearVelocityMetresPerSec,
                LINEAR_MOTION_CONSTRAINTS.maxAcceleration
        ));
    }

    public Actionable doProfiledMoveTo(Pose2d endPos, TrapezoidProfile.Constraints constraints) {
        return (host) -> new Action(host) {
            ProfiledDriveDirectionController controller;

            @Override
            protected void initialize() {
                setState(SystemState.KINEMATIC_AUTO);

                double startTime = Timer.getFPGATimestamp();
                Pose2d currentPose = odometry.getPoseMeters();

                controller = new ProfiledDriveDirectionController(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, ROTATIONAL_KP, ROTATIONAL_KI, ROTATIONAL_KD, currentPose, null,
                        endPos, null, POSITION_TOLERANCE, VELOCITY_TOLERANCE, constraints, ROTATION_MOTION_CONSTRAINTS);
//                double straightKp, double straightKi, double straightKd, double rotationalKp, double rotationalKi, double rotationalKd,
//                Pose2d startPos, Pose2d startVelocity, Pose2d endPos, Pose2d endVelocity, Pose2d poseTolerance, Pose2d velocityTolerance,
//                        TrapezoidProfile.Constraints forwardMotionConstraints, TrapezoidProfile.Constraints rotationMotionConstraints
//	            );
                controller.setNextPosition(currentPose, null, endPos, null, startTime);
            }

            @Override
            protected void execute() {
                feedModulesFromTargetVelocities(controller.calculate(odometry.getPoseMeters(), Timer.getFPGATimestamp()));
            }

            @Override
            protected void end() {
                fieldRelativeDrive(0,0,0);

//                setState(SystemState.DRIVER_CONTROLLED); // TODO maybe change which state this enters after completion of Auto
            }

            @Override
            protected boolean isFinished() {
                return controller.onTarget();
            }
        };
    }

//    public Actionable trajectory(TrajectoryGenerator.ControlVectorList controlVectors) {
//        List<Rotation2d> rotations = new ArrayList<>();
//
//        for (int i = 0; i < controlVectors.size(); i++) {
//            rotations.add(Rotation2d.fromDegrees(0));
//        }
//
//        return trajectory(controlVectors, rotations);
//    }
//
//    public Actionable trajectory(TrajectoryGenerator.ControlVectorList controlVectors, List<Rotation2d> rotationWaypoints) {
//        return (host) -> new Action(host) {
//            HolonomicDriveController controller;
//            Trajectory trajectory;
//            double startTime;
//
//
//            TrajectoryGenerator.ControlVectorList waypoints;
//            List<Rotation2d> rotationStates;
//
//            @Override
//            public void initialize() {
//                setState(SystemState.KINEMATIC_AUTO);
//
//                controller = new HolonomicDriveController(
//                        new PIDController(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, Robot.LOOP_PERIOD),
//                        new PIDController(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, Robot.LOOP_PERIOD),
//                        new ProfiledPIDController(ROTATIONAL_KP, ROTATIONAL_KI, ROTATIONAL_KD, ROTATION_MOTION_CONSTRAINTS, Robot.LOOP_PERIOD)
//                );
//
//                waypoints = controlVectors;
//
//                trajectory = TrajectoryGenerator.generateTrajectory(
//                        waypoints,
//                        new TrajectoryConfig(MAX_VELOCITY_METRES_PER_SEC, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED)
//                );
//
//                rotationStates = createRotationStates();
//
//                startTime = Timer.getFPGATimestamp();
//            }
//
//            @Override
//            public void execute() {
//                Trajectory.State targetState = trajectory.sample(Timer.getFPGATimestamp() - startTime);
//                int index = (int) Math.round(targetState.timeSeconds / trajectory.getTotalTimeSeconds() * rotationStates.size());
//                index = Math.min(index, rotationStates.size() - 1);
//                Rotation2d targetRotation = rotationStates.get(index);
//
//                SmartDashboard.putNumber("Target Rotation", targetRotation.getDegrees());
//                SmartDashboard.putNumber("Target Rotation Index", index);
//                SmartDashboard.putNumber("Target X", targetState.poseMeters.getX());
//                SmartDashboard.putNumber("Target Y", targetState.poseMeters.getY());
//
//                feedModulesFromTargetVelocities(controller.calculate(odometry.getPoseMeters(), targetState, targetRotation));
//            }
//
//            @Override
//            protected void end() {
//                fieldRelativeDrive(0, 0, 0);
//                setState(SystemState.DRIVER_CONTROLLED);
//            }
//
//            @Override
//            public boolean isFinished() {
//                Pose2d goalPose = trajectory.sample(Double.POSITIVE_INFINITY).poseMeters;
//                Pose2d currentPose = odometry.getPoseMeters();
//
//                Pose2d error = goalPose.relativeTo(currentPose);
//                Rotation2d rotationError = currentPose.getRotation().minus(rotationStates.get(rotationStates.size() - 1));
//
//                return Math.abs(error.getX()) < POSITION_TOLERANCE.getX()
//                        && Math.abs(error.getY()) < POSITION_TOLERANCE.getY()
//                        && Math.abs(rotationError.getRadians()) < POSITION_TOLERANCE.getRotation().getRadians();
//            }
//
//            private ArrayList<Rotation2d> createRotationStates() {
//                ArrayList<Rotation2d> rotationStates = new ArrayList<>(); // Including start and end
//                ArrayList<Integer> waypointIndexes = new ArrayList<>();
//
//                SmartDashboard.putNumber("States", trajectory.getStates().size());
//
//                for (Trajectory.State state : trajectory.getStates()) {
//                    System.out.println("X" + state.poseMeters.getX());
//                    System.out.println("Y" + state.poseMeters.getY());
//                    for (ControlVector waypoint : waypoints) {
//                        if (Math.abs(waypoint.x[0] - state.poseMeters.getX()) < 0.0001 && Math.abs(waypoint.y[0] - state.poseMeters.getY()) < 0.0001) {
//                            waypointIndexes.add(trajectory.getStates().indexOf(state));
//                        }
//                    }
//                }
//
//                for (int i = 0; i < trajectory.getStates().size() - 1; i++) {
//                    int prevWaypointIndex = -1;
//                    int nextWaypointIndex = 0;
//
//                    for (int waypointIndex : waypointIndexes) {
//                        if (waypointIndex <= i) {
//                            prevWaypointIndex += 1;
//                            nextWaypointIndex += 1;
//                        }
//                    }
//
//                    Trajectory.State prevWaypoint = trajectory.getStates().get(waypointIndexes.get(prevWaypointIndex));
//                    Trajectory.State nextWaypoint = trajectory.getStates().get(waypointIndexes.get(nextWaypointIndex));
//
//                    double prevWaypointTime = prevWaypoint.timeSeconds;
//                    double nextWaypointTime = nextWaypoint.timeSeconds;
//
//                    double rotationRadians = lerp(
//                            rotationWaypoints.get(prevWaypointIndex).getRadians(),
//                            rotationWaypoints.get(nextWaypointIndex).getRadians(),
//                            (trajectory.getStates().get(i).timeSeconds - prevWaypointTime) / (nextWaypointTime - prevWaypointTime)
//                    );
//
//                    rotationStates.add(new Rotation2d(rotationRadians));
//                }
//
//                rotationStates.add(rotationWaypoints.get(rotationWaypoints.size() - 1));
//
//                return rotationStates;
//            }
//
//            private double lerp(double startValue, double endValue, double t) {
//                return startValue + (endValue - startValue) * t;
//            }
//        };
//    }
//
//    public Actionable barrelRacingPath() {
//    	final TrajectoryGenerator.ControlVectorList barrelRacingPathWaypoints = new TrajectoryGenerator.ControlVectorList(
//                Arrays.asList(
//                        new ControlVector(new double[]{3.55, 1, 0}, new double[]{2.3, 0.5, 0}), // 1
//                        new ControlVector(new double[]{5.1, 1, 0}, new double[]{1.67, -1, 0}), // 2
//                        new ControlVector(new double[]{4.72, -1, 0}, new double[]{0.34, 0, 0}), // 3
//                        new ControlVector(new double[]{2.85, -1, 0}, new double[]{0.54, 1, 0}), // 4
//                        new ControlVector(new double[]{2.4, 1, 0}, new double[]{2.11, 1, 0}), // 5
//                        new ControlVector(new double[]{6.31, 1, 0}, new double[]{1.74, 1, 0}), // 6
//                        new ControlVector(new double[]{7.48, 0, 0}, new double[]{2.5, 1, 0}), // 7
//                        new ControlVector(new double[]{6.41, -1, 0}, new double[]{3.73, 0, 0}), // 8
//                        new ControlVector(new double[]{5.07, 0.5, 0}, new double[]{3, -1, 0}), // 9
//                        new ControlVector(new double[]{5.79, 1, 0}, new double[]{1.47, -1, 0}), // 10
//                        new ControlVector(new double[]{7.63, 1, 0}, new double[]{0.27, 0, 0}), // 11
//                        new ControlVector(new double[]{9.14, 0, 0}, new double[]{1.54, 1, 0}), // 12
//                        new ControlVector(new double[]{8.84, -1, 0}, new double[]{2.1, 0.5, 0}), // 13
//                        new ControlVector(new double[]{-0.2, -1, 0}, new double[]{2.1, 0, 0}) // 14
//                )
//        );
//
//    	return trajectory(barrelRacingPathWaypoints);
//    }
//
//    public Actionable slalomPath() {
//        final TrajectoryGenerator.ControlVectorList slalomPathWaypoints = new TrajectoryGenerator.ControlVectorList(
//                Arrays.asList(
////                                new ControlVector(new double[]{0.19, 1, 0}, new double[]{0.07, 0, 0}), // 1
//                        new ControlVector(new double[]{2.4, 1, 0}, new double[]{1.22, 1, 0}), // 2A
//                        new ControlVector(new double[]{5.09, 1, 0}, new double[]{2.6, 1, 0.5}), // 3A
//                        new ControlVector(new double[]{7.16, 1, 0}, new double[]{1.11, -0.5, 0}), // 4A
//                        new ControlVector(new double[]{8.27, 1, 0}, new double[]{0.74, -0.25, 0}), // 5
//                        new ControlVector(new double[]{8.85, 1, 0}, new double[]{1.65, 1, 0}), // 6
//                        new ControlVector(new double[]{7.18, -1, 0}, new double[]{2.05, -1, 0}), // 7
//                        new ControlVector(new double[]{6.47, -1, 0}, new double[]{0.3, -1, 0}), // 8
//                        new ControlVector(new double[]{3.44, -1, 0}, new double[]{0.2, 0, 0}), // 9
//                        new ControlVector(new double[]{1.4, -1, 0}, new double[]{1.7, -1, 0}) // 10
//                )
//        );
//
//        return trajectory(slalomPathWaypoints);
//    }
//
//    public Actionable bouncePath() {
//    	final TrajectoryGenerator.ControlVectorList bouncePathWaypoints = new TrajectoryGenerator.ControlVectorList(
//                Arrays.asList(
//                        new ControlVector(new double[]{2.2, 0.5, 0}, new double[]{3.24, -1, 0}), // 1
//                        new ControlVector(new double[]{2.79, 1, 0}, new double[]{0.89, -1, 0}), // 2
//                        new ControlVector(new double[]{4.55, 1, 0}, new double[]{0.48, 0, 0}), // 3
//                        new ControlVector(new double[]{5.0, -0.5, 0}, new double[]{3.15, 1, 0}), // 4
//                        new ControlVector(new double[]{4.8, 0, 0}, new double[]{3.14, -1, 0}), // 5
//                        new ControlVector(new double[]{4.94, 1, 0}, new double[]{0.47, 0, 0}), // 5
//                        new ControlVector(new double[]{6.7, 0.5, 0}, new double[]{0.34, 1, 0}), // 5
//                        new ControlVector(new double[]{7.45, -0.5, 0}, new double[]{3.16, 1, 0}), // 5
//                        new ControlVector(new double[]{7.12, 0, 0}, new double[]{3.16, -1, 0}), // 5
//                        new ControlVector(new double[]{7.99, 1, 0}, new double[]{1.79, 1, 0}), // 5
//                        new ControlVector(new double[]{9.1, 1, 0}, new double[]{1.79, 1, 0}) // 5
//                )
//        );
//
//        return trajectory(bouncePathWaypoints);
//    }
//
//    public Actionable testRotationBouncePath() {
//        final TrajectoryGenerator.ControlVectorList testRotationBouncePathWaypoints = new TrajectoryGenerator.ControlVectorList(
//                Arrays.asList(
//                        new ControlVector(new double[]{3.42, 1, 0}, new double[]{1.91, -1, 0}), // 1
//                        new ControlVector(new double[]{3.42, -1, 0}, new double[]{1.1, -1, 0}), // 2A
//                        new ControlVector(new double[]{2.62, -1, 0}, new double[]{1.1, 1, 0}), // 3A
//                        new ControlVector(new double[]{2.62, -1, 0}, new double[]{1.91, 1, 0}), // 4A
//                        new ControlVector(new double[]{2.4, 1, 0}, new double[]{2.11, 0.5, 0}), // 5
//                        new ControlVector(new double[]{6.31, 1, 0}, new double[]{1.74, 1, 0}), // 6
//                        new ControlVector(new double[]{7.48, 0, 0}, new double[]{2.5, 1, 0}), // 7
//                        new ControlVector(new double[]{6.41, -1, 0}, new double[]{3.73, 0, 0}), // 8
//                        new ControlVector(new double[]{5.07, 0.5, 0}, new double[]{3, -1, 0}), // 9
//                        new ControlVector(new double[]{5.79, 1, 0}, new double[]{1.47, -1, 0}), // 10
//                        new ControlVector(new double[]{7.63, 1, 0}, new double[]{0.27, 0, 0}), // 11
//                        new ControlVector(new double[]{9.14, 0, 0}, new double[]{1.54, 1, 0}), // 12
//                        new ControlVector(new double[]{8.84, -1, 0}, new double[]{2.1, 0.5, 0}), // 13
//                        new ControlVector(new double[]{-0.2, -1, 0}, new double[]{2.1, 0, 0}) // 14
//                )
//        );
//
//        List<Rotation2d> rotationWaypoints = Arrays.asList(
//                Rotation2d.fromDegrees(-135), // 1
//                Rotation2d.fromDegrees(-215), // 2
//                Rotation2d.fromDegrees(45), // 3
//                Rotation2d.fromDegrees(0), // 4
//                Rotation2d.fromDegrees(0), // 5
//                Rotation2d.fromDegrees(0), // 6
//                Rotation2d.fromDegrees(0), // 7
//                Rotation2d.fromDegrees(0), // 8
//                Rotation2d.fromDegrees(0), // 9
//                Rotation2d.fromDegrees(0), // 10
//                Rotation2d.fromDegrees(0), // 11
//                Rotation2d.fromDegrees(0), // 12
//                Rotation2d.fromDegrees(0), // 13
//                Rotation2d.fromDegrees(0) // 14
//        );
//
//        return trajectory(testRotationBouncePathWaypoints, rotationWaypoints);
//    }
}

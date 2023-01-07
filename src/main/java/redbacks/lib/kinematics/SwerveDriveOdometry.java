package redbacks.lib.kinematics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import redbacks.lib.geometry.Differential2d;
import redbacks.lib.motion.MotionState2d;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * swerve drive encoders and swerve azimuth encoders.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 * @author Ben Schwarz.
 */
public class SwerveDriveOdometry {
	private final SwerveDriveKinematics kinematics;
	private Pose2d poseMeters;
	private Differential2d robotVelocity = new Differential2d();
	private Differential2d prevRobotVelocity = new Differential2d();
	private MotionState2d prevRobotMotionState;
	private MotionState2d robotMotionState;
	private double prevTimeSeconds = -1;
	private final static double VELOCITY_INPUT_SIGNAL_STRENTH = 0.8;

	private Rotation2d gyroOffset;
	private Rotation2d previousAngle;
	
	/**
	 * Constructs a SwerveDriveOdometry object.
	 *
	 * @param kinematics  The swerve drive kinematics for your drivetrain.
	 * @param gyroAngle   The angle reported by the gyroscope.
	 * @param initialPose The starting position of the robot on the field.
	 * @param timestamp Seconds.
	 */
	public SwerveDriveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, Pose2d initialPose, double timestamp) {
		this.kinematics = kinematics;
		this.poseMeters = initialPose;
		this.robotMotionState = new MotionState2d(initialPose, timestamp);
		this.prevRobotMotionState = robotMotionState;
		this.gyroOffset = poseMeters.getRotation().minus(gyroAngle);
		this.previousAngle = initialPose.getRotation();
	}

	/**
	 * Constructs a SwerveDriveOdometry object with the default pose at the origin.
	 *
	 * @param kinematics The swerve drive kinematics for your drivetrain.
	 * @param gyroAngle  The angle reported by the gyroscope.
	 */
	public SwerveDriveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, double timestamp) {
		this(kinematics, gyroAngle, new Pose2d(), timestamp);
	}

	/**
	 * Resets the robot's position on the field.
	 *
	 * <p>The gyroscope angle does not need to be reset here on the user's robot code.
	 * The library automatically takes care of offsetting the gyro angle.
	 *
	 * @param pose      The position on the field that your robot is at.
	 * @param gyroAngle The angle reported by the gyroscope.
	 */
	public void resetPosition(Pose2d pose, Rotation2d gyroAngle) {
		poseMeters = pose;
		previousAngle = pose.getRotation();
		gyroOffset = poseMeters.getRotation().minus(gyroAngle);
	}
	
	/**
	 * Updates the recorded state of motion the swerve drivetrain is in. 
	 * This includes position, velocity and acceleration values. 
	 *
	 * <p>The velocity and acceleration values have very simple noise reduction calculations added.
	 * The filter is an infinite impulse response (IIR) and also introduces phaselag while being in effect a lowpass filter. Yet it is computationally simple. </p>
	 * @param currentTimeSeconds
	 * @param gyroAngle
	 * @param moduleStates
	 * @return
	 */
	public MotionState2d updateMotionState(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState...moduleStates) {
		double period = prevTimeSeconds >= 0 ? currentTimeSeconds - prevTimeSeconds : 0.0;
		prevTimeSeconds = currentTimeSeconds;
		
		if(period != 0) {
			Rotation2d angle = gyroAngle.plus(gyroOffset);
			ChassisSpeeds chassisState = kinematics.toChassisSpeeds(moduleStates);
			Pose2d newPose = poseMeters.exp(
				new Differential2d(
						chassisState.vxMetersPerSecond * period,
						chassisState.vyMetersPerSecond * period,
						angle.minus(previousAngle).getRadians()
					)
			);
			
			/**
			 * This isn't a great way of reducing noise, but it is less computationally costing than other methods.
			 */
			robotVelocity = new Differential2d(
					chassisState.vxMetersPerSecond * VELOCITY_INPUT_SIGNAL_STRENTH + prevRobotMotionState.getVelocity().dx * (1 - VELOCITY_INPUT_SIGNAL_STRENTH),
					chassisState.vyMetersPerSecond * VELOCITY_INPUT_SIGNAL_STRENTH + prevRobotMotionState.getVelocity().dy * (1 - VELOCITY_INPUT_SIGNAL_STRENTH), 
					chassisState.omegaRadiansPerSecond * VELOCITY_INPUT_SIGNAL_STRENTH + prevRobotMotionState.getVelocity().dtheta * (1 - VELOCITY_INPUT_SIGNAL_STRENTH),
					period, currentTimeSeconds
			);
			
			Differential2d robotAcceleration = new Differential2d(
					(chassisState.vxMetersPerSecond - prevRobotMotionState.getVelocity().dx) * VELOCITY_INPUT_SIGNAL_STRENTH  + prevRobotMotionState.getAcceleration().dx * (1 - VELOCITY_INPUT_SIGNAL_STRENTH),
					(chassisState.vyMetersPerSecond - prevRobotMotionState.getVelocity().dy) * VELOCITY_INPUT_SIGNAL_STRENTH  + prevRobotMotionState.getAcceleration().dy * (1 - VELOCITY_INPUT_SIGNAL_STRENTH), 
					(chassisState.omegaRadiansPerSecond  - prevRobotMotionState.getVelocity().dtheta) * VELOCITY_INPUT_SIGNAL_STRENTH  + prevRobotMotionState.getAcceleration().dtheta * (1 - VELOCITY_INPUT_SIGNAL_STRENTH),
					period, currentTimeSeconds
			);
			
			previousAngle = angle;
			poseMeters = new Pose2d(newPose.getTranslation(), angle);
			prevRobotMotionState = robotMotionState;
			robotMotionState = new MotionState2d(poseMeters, robotVelocity, robotAcceleration, currentTimeSeconds);
			
			return robotMotionState;
		}
		
		return robotMotionState;
	}
	
	/**
	 * @deprecated Use updateMotionState instead.
	 * Updates the robot's position on the field using forward kinematics and
	 * integration of the pose over time. This method takes in the current time as
	 * a parameter to calculate period (difference between two timestamps). The
	 * period is used to calculate the change in distance from a velocity. This
	 * also takes in an angle parameter which is used instead of the
	 * angular rate that is calculated from forward kinematics.
	 *
	 * @param currentTimeSeconds The current time in seconds.
	 * @param gyroAngle          The angle reported by the gyroscope.
	 * @param moduleStates       The current state of all swerve modules. Please provide
	 *                           the states in the same order in which you instantiated your
	 *                           SwerveDriveKinematics.
	 * @return The new pose of the robot.
	 */
	@Deprecated
	public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
		double period = prevTimeSeconds >= 0 ? currentTimeSeconds - prevTimeSeconds : 0.0;
		prevTimeSeconds = currentTimeSeconds;

		var angle = gyroAngle.plus(gyroOffset);
		var chassisState = kinematics.toChassisSpeeds(moduleStates);
		
		if(period != 0) {
			prevRobotVelocity = robotVelocity;
			robotVelocity = new Differential2d(
				chassisState.vxMetersPerSecond, 
				chassisState.vyMetersPerSecond, 
				angle.minus(previousAngle).getRadians() / period
			);
		}
		
		//Updates the pose by first determining the displacement since the previous pose.
		var newPose = poseMeters.exp(
			new Differential2d(
				chassisState.vxMetersPerSecond * period,
				chassisState.vyMetersPerSecond * period,
				angle.minus(previousAngle).getRadians()
			)
		);

		previousAngle = angle;
		poseMeters = new Pose2d(newPose.getTranslation(), angle);
		update(gyroAngle, moduleStates);
		return poseMeters;
	}

	/**
	 * @deprecated Use updateMotionState instead. (This will crash the code if simulated not on a robot due to HAL errors).
	 * This method still exists in order to replicate the functionality of SwerveDriveOdometry in WPI. 
	 * Updates the robot's position on the field using forward kinematics and
	 * integration of the pose over time. This method automatically calculates the
	 * current time to calculate period (difference between two timestamps). The
	 * period is used to calculate the change in distance from a velocity. This
	 * also takes in an angle parameter which is used instead of the angular
	 * rate that is calculated from forward kinematics.
	 *
	 * @param gyroAngle    The angle reported by the gyroscope.
	 * @param moduleStates The current state of all swerve modules. Please provide
	 *                     the states in the same order in which you instantiated your
	 *                     SwerveDriveKinematics.
	 * @return The new pose of the robot.
	 */
	@Deprecated
	public Pose2d update(Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
		return updateWithTime(Timer.getFPGATimestamp(), gyroAngle, moduleStates);
	}
	
	/**
	 * Returns the robot's position, velocity and acceleration.
	 */
	public MotionState2d getRobotState() {
		return robotMotionState;
	}
	
	/**
	 * Returns the position of the robot on the field.
	 *
	 * @return The pose of the robot (x and y are in meters).
	 */
	public Pose2d getPoseMeters() {
		return poseMeters;
	}
	
	/**
	 * The velocity of the robot.
	 * 
	 * @return A Differential2d Object with the robot's 
	 */
	public Differential2d getRobotVelocity() {
		return robotVelocity;
	}
}

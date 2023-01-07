package redbacks.lib.control;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import redbacks.lib.motion.TrapezoidProfile;

/**
 * Class to drive a swervedrivetrain between two points on the field - independently of robot heading.
 * 
 * It works by using two profiledPID controllers - one for straightline velocity and one for rotational velocity. 
 * It calculates the angle from it's current point and forms a straightline between that and the desired location.
 * 
 * TODO Needs abstraction.
 * @author Ben Schwarz.
 */
public class ProfiledDriveDirectionController {
	// Controllers
	private ProfiledPIDController linearPID, rotationalPID;
	
	// Pose Values
	private double straightLineDistance;

	private Pose2d endPos, poseTolerance;
	
	private boolean onTarget;
	
	// Time Values
	private double prevTime;

	/**
	 * FIXME velocity is currently unimplemented
	 */
	public ProfiledDriveDirectionController(
			double straightKp, double straightKi, double straightKd, double rotationalKp, double rotationalKi, double rotationalKd,
			Pose2d startPos, Pose2d startVelocity, Pose2d endPos, Pose2d endVelocity, Pose2d poseTolerance, Pose2d velocityTolerance,
			TrapezoidProfile.Constraints forwardMotionConstraints, TrapezoidProfile.Constraints rotationMotionConstraints
	) {
		this.straightLineDistance = startPos.getTranslation().getDistance(endPos.getTranslation());
		this.endPos = endPos;
		this.poseTolerance = poseTolerance;
		this.linearPID = new ProfiledPIDController(straightKp, straightKi, straightKd, forwardMotionConstraints);
		this.linearPID.setGoal(new TrapezoidProfile.State(straightLineDistance, 0)); // TODO Change target velocity from 0
		this.rotationalPID = new ProfiledPIDController(rotationalKp, rotationalKi, rotationalKd, rotationMotionConstraints);
		this.rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
		this.rotationalPID.reset(startPos.getRotation().getRadians());
		this.rotationalPID.setGoal(new TrapezoidProfile.State(endPos.getRotation().getRadians(), 0)); // TODO Change target velocity from 0
	}
	
	/**
	 * Calculates the velocities that the robot should be at the current robot state in time. 
	 * @param currentPose
	 * @param currentTime - This is the current time of the system in seconds. It should use the same time method as that of startTime. Note on the robot this is typically Timer.getFPGATimestamp().
	 * @return A ChassisSpeeds object with the desired vxVelocity, vyVelocity and omegaRadiansPerSecond.
	 */
	public ChassisSpeeds calculate(Pose2d currentPose, double currentTime) {
		double secondsSinceLastCall = currentTime - prevTime;
		this.prevTime = currentTime;
		double angleFromCentre = calculateAngle(currentPose, endPos);
		
		//Generates values for traveling along the straight line.
		final double distanceTravelled = straightLineDistance - currentPose.getTranslation().getDistance(endPos.getTranslation());
		final double linearSpeed = linearPID.calculate(distanceTravelled, secondsSinceLastCall) + linearPID.getSetpoint().velocity; //The setpoint is determined by a trapezoidal profile and as such the feedforward value is gained by calling this method.
		
		//Converts straight line values to field relative position
		final double frontSpeed = linearSpeed * Math.cos(Math.toRadians(angleFromCentre));
		final double strafeSpeed = linearSpeed * Math.sin(Math.toRadians(angleFromCentre));
		final double rotationSpeed = rotationalPID.calculate(currentPose.getRotation().getRadians(), secondsSinceLastCall) + rotationalPID.getSetpoint().velocity;
		
		this.onTarget = isOnTarget(currentPose, endPos);
		return ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, strafeSpeed, rotationSpeed, currentPose.getRotation());	
	}
	
	/**
	 * Returns the angle of the path to follow in degrees. 
	 * @param currentPos The current position of the robot.
	 * @param endPos The ending position of the robot.
	 * @return The angle that the robot needs to follow to end up in the end position given.
	 */
	public double calculateAngle(Pose2d currentPos, Pose2d endPos) {
		double forwardLength = endPos.getTranslation().getX() - currentPos.getTranslation().getX();
		double leftLength = endPos.getTranslation().getY() - currentPos.getTranslation().getY();
		
		// Determines if the robot should drive parallel to the alliance wall.
		if(leftLength == 0) {
			if(forwardLength > 0) return 0;
			else return 180;
		}
		
		// Determines if robot should drive perpendicular to the alliance wall.
		if(forwardLength == 0) {
			if(leftLength > 0) return 90;
			else return -90;
		}
		
		return Math.toDegrees(Math.atan2(leftLength, forwardLength));
	}
	
	/**
	 * Sets the robot to have a new goal. This may be redundant in future.
	 * @param startPos
	 * @param startVelocity
	 * @param endPos
	 * @param endVelocity
	 * @param endRotation
	 */
	public void setNextPosition(Pose2d startPos, Pose2d startVelocity, Pose2d endPos, Pose2d endVelocity, double startTime) {
		this.straightLineDistance = startPos.getTranslation().getDistance(endPos.getTranslation());
		this.endPos = endPos;
		this.linearPID.setGoal(new TrapezoidProfile.State(straightLineDistance, 0)); // TODO Change target velocity from 0
		this.rotationalPID.reset(startPos.getRotation().getRadians());
		this.rotationalPID.setGoal(new TrapezoidProfile.State(endPos.getRotation().getRadians(), 0)); // TODO Change target velocity from 0
		this.prevTime = startTime;
	}
	
	public double getStraightLineDistance() {
		return straightLineDistance;
	}
	
	/**
	 * Returns true if the pose error is within tolerance of the reference.
	 */
	public boolean isOnTarget(Pose2d currentPose, Pose2d desiredPose) {
		Pose2d poseError = desiredPose.relativeTo(currentPose);
		final Translation2d translateError = poseError.getTranslation();
		final Rotation2d rotateError = poseError.getRotation();
		final Translation2d translateTolerance = poseTolerance.getTranslation();
		final Rotation2d rotateTolerance = poseTolerance.getRotation();
		
		return Math.abs(translateError.getX()) < translateTolerance.getX()
				&& Math.abs(translateError.getY()) < translateTolerance.getY() 
				&& Math.abs(rotateError.getRadians()) < rotateTolerance.getRadians();
	}
	
	public boolean onTarget() {
		return this.onTarget;
	}
	
	/**
	 * Sets the pose error which is considered tolerable for use with
	 * atReference().
	 *
	 * @param poseTolerance Pose error which is tolerable.
	 */
	public void setTolerance(Pose2d poseTolerance) {
		this.poseTolerance = poseTolerance;
	}
}

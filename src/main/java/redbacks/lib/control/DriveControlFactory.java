package redbacks.lib.control;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import redbacks.lib.motion.TrapezoidProfile;

public class DriveControlFactory {
	// PID values
	private final double
			straightKp, straightKi, straightKd,
			rotationalKp, rotationalKi, rotationalKd;

	// Tolerances
	private final Pose2d defaultPositionTolerance, defaultVelocityTolerance;

	// Profiling constraints
	private final TrapezoidProfile.Constraints defaultLinearMotionContraints, defaultRotationMotionConstraints;

	public DriveControlFactory(
			double straightKp, double straightKi, double straightKd,
			double rotationalKp, double rotationalKi, double rotationalKd,
			Pose2d defaultPositionTolerance, Pose2d defaultVelocityTolerance,
			TrapezoidProfile.Constraints defaultLinearMotionContraints, TrapezoidProfile.Constraints defaultRotationMotionConstraints
	) {
		this.straightKp = straightKp;
		this.straightKi = straightKi;
		this.straightKd = straightKd;

		this.rotationalKp = rotationalKp;
		this.rotationalKi = rotationalKi;
		this.rotationalKd = rotationalKd;

		this.defaultPositionTolerance = defaultPositionTolerance;
		this.defaultVelocityTolerance = defaultVelocityTolerance;

		this.defaultLinearMotionContraints = defaultLinearMotionContraints;
		this.defaultRotationMotionConstraints = defaultRotationMotionConstraints;
	}

	public Pose2d getDefaultPositionTolerance() {
		return defaultPositionTolerance;
	}

	public Pose2d getDefaultVelocityTolerance() {
		return defaultVelocityTolerance;
	}

	public TrapezoidProfile.Constraints getDefaultLinearMotionContraints() {
		return defaultLinearMotionContraints;
	}

	public TrapezoidProfile.Constraints getDefaultRotationMotionConstraints() {
		return defaultRotationMotionConstraints;
	}
	
	public ProfiledDriveDirectionController createProfiledController(
			Pose2d startPos, Pose2d startVelocity, Pose2d endPos, Pose2d endVelocity
	) {
		return createProfiledController(
				startPos, startVelocity, endPos, endVelocity,
				defaultPositionTolerance, defaultVelocityTolerance,
				defaultLinearMotionContraints, defaultRotationMotionConstraints
		);
	}

	/**
	 * FIXME velocity is currently unimplemented
	 */
	public ProfiledDriveDirectionController createProfiledController(
			Pose2d startPos, Pose2d startVelocity, Pose2d endPos, Pose2d endVelocity,
			Pose2d positionTolerance, Pose2d velocityTolerance,
			TrapezoidProfile.Constraints forwardMotionConstraints, TrapezoidProfile.Constraints rotationMotionConstraints
	) {
		return new ProfiledDriveDirectionController(
				straightKp, straightKi, straightKd, rotationalKp, rotationalKi, rotationalKd,
				startPos, startVelocity, endPos, endVelocity, positionTolerance, velocityTolerance,
				forwardMotionConstraints, rotationMotionConstraints
		);
	}
}
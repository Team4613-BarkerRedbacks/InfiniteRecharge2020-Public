package redbacks.lib.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import redbacks.lib.geometry.Differential2d;
import redbacks.lib.motion.MotionState2d;

/**
 * Test cases for the odometry.
 * Demonstrates design choices such as the fact the gyro is the only aspect considered for the robot's heading and rotational velocity.
 * @author Ben Schwarz
 */
public class TestOdometry {
	private SwerveDriveOdometry odometry;
	private SwerveDriveKinematics kinematics;
	private double time = 0;
	private static final double EPSILON = 1E-9; //Use indendent of the velocity for stationary velocities.
	private static final double MOVING_ERROR_DELTA = 0.25; // Use as a percentage of the velocity
	
	private static final double
			DISTANCE_BETWEEN_MODULE_LENGTH = 0.5853,
			DISTANCE_BETWEEN_MODULE_WIDTH = 0.5853,
			MODULE_DISTANCE_FROM_CENTRE_LENGTH = DISTANCE_BETWEEN_MODULE_LENGTH / 2,
			MODULE_DISTANCE_FROM_CENTRE_WIDTH = DISTANCE_BETWEEN_MODULE_WIDTH / 2;
	
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
	
	@BeforeEach
	public void beforeEach() {
		this.kinematics = new SwerveDriveKinematics(
					ModulePosition.LEFT_FRONT.offset, 
					ModulePosition.RIGHT_FRONT.offset, 
					ModulePosition.LEFT_BACK.offset,
					ModulePosition.RIGHT_BACK.offset
		);
		this.time = 0;
		this.odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d(), time);
	}
	
	@AfterEach
	public void afterEach() {
		this.kinematics = null;
		this.odometry = null;
		this.time = 0;
	}
	
	//Using MotionState
	@Test
	public void ForwardOneMetreWithMotion() {
		Rotation2d moduleDirection = new Rotation2d(0);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateMotionState(time, gyroAngle, moduleStates1);
		time++;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		
		odometry.updateMotionState(time, gyroAngle, moduleStates2);
		Pose2d position = new Pose2d(1, 0, new Rotation2d(0)); 
		Differential2d velocity = new Differential2d(1, 0, 0); 
		Differential2d acceleration = new Differential2d(); 
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, time);
		
		assertTrue(odometry.getRobotState().getPose().equals(motionState.getPose()));
		assertTrue(odometry.getPoseMeters().equals(motionState.getPose()));
		assertEquals(odometry.getRobotState().getVelocity().dx, motionState.getVelocity().dx, odometry.getRobotVelocity().dx * MOVING_ERROR_DELTA);
	}
	
	@Test
	public void RightTwoMetreWithMotion() {
		Rotation2d moduleDirection = new Rotation2d(-Math.PI/2);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateMotionState(time, gyroAngle, moduleStates1);
		time += 2;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		
		odometry.updateMotionState(time, gyroAngle, moduleStates2);
		Pose2d position = new Pose2d(0, -2, new Rotation2d(0)); 
		Differential2d velocity = new Differential2d(0, 1, 0); 
		Differential2d acceleration = new Differential2d(); 
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, time);
		
		assertTrue(odometry.getRobotState().getPose().equals(motionState.getPose()));
		assertTrue(odometry.getPoseMeters().equals(motionState.getPose()));
		assertEquals(odometry.getRobotState().getVelocity().dx, motionState.getVelocity().dx, EPSILON);
	}
	
	@Test
	public void RightTwiceWithMotion() {
		Rotation2d moduleDirection = new Rotation2d(-Math.PI/2);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateMotionState(time, gyroAngle, moduleStates1);
		time += 2;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		
		odometry.updateMotionState(time, gyroAngle, moduleStates2);
		Pose2d position = new Pose2d(0, -2, new Rotation2d(0)); 
		Differential2d velocity = new Differential2d(0, 1, 0); 
		Differential2d acceleration = new Differential2d(); 
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, time);
		
		assertTrue(odometry.getRobotState().getPose().equals(motionState.getPose()));
		assertTrue(odometry.getPoseMeters().equals(motionState.getPose()));
		assertEquals(odometry.getRobotState().getVelocity().dx, motionState.getVelocity().dx, EPSILON);
		
		time++;
		SwerveModuleState[] moduleStates3 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		
		odometry.updateMotionState(time, gyroAngle, moduleStates3);
		Pose2d position2 = new Pose2d(0, -3, new Rotation2d(0)); 
		Differential2d velocity2 = new Differential2d(0, 1, 0); 
		Differential2d acceleration2 = new Differential2d(); 
		MotionState2d motionState2 = new MotionState2d(position2, velocity2, acceleration2, time);
		
		assertTrue(odometry.getRobotState().getPose().equals(motionState2.getPose()));
		assertTrue(odometry.getPoseMeters().equals(motionState2.getPose()));
		assertEquals(odometry.getRobotState().getVelocity().dx, motionState2.getVelocity().dx, EPSILON);
	}
	
	@Test
	public void MoveForwardsRepeatedly() {
		Rotation2d moduleDirection = new Rotation2d(0);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateMotionState(time, gyroAngle, moduleStates1);
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		for(int i = 1; i < 100; i++) {
			time = i;
			odometry.updateMotionState(time, gyroAngle, moduleStates2);
			Pose2d position = new Pose2d(i, 0, new Rotation2d(0)); 
			Differential2d velocity = new Differential2d(1, 0, 0); 
			Differential2d acceleration = new Differential2d(); 
			MotionState2d motionState = new MotionState2d(position, velocity, acceleration, time);
			assertTrue(odometry.getRobotState().getPose().equals(motionState.getPose()));
			assertTrue(odometry.getPoseMeters().equals(motionState.getPose()));
			assertEquals(odometry.getRobotState().getVelocity().dx, motionState.getVelocity().dx, MOVING_ERROR_DELTA);
		}
	}
	
	//Using updateWithTime
	@Test
	@Disabled("HAL")
	public void ForwardOneMetre() {
		Rotation2d moduleDirection = new Rotation2d(0);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		time++;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates2);
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(1, 0, new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(1, 0, 0)));
	}
	
	@Test
	@Disabled("HAL")
	public void ForwardThenBack() {
		Rotation2d moduleDirection = new Rotation2d(0);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		time++;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates2);
		time++;
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(1, 0, new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(1, 0, 0)));
		
		SwerveModuleState[] moduleStates3 = {
				new SwerveModuleState(-1, moduleDirection), 
				new SwerveModuleState(-1, moduleDirection), 
				new SwerveModuleState(-1, moduleDirection), 
				new SwerveModuleState(-1, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates3);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(0, 0, new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(-1, 0, 0)));
	}
	
	/**
	 * This test demonstrates that the heading of the robot and its velocity is only based on the gyro.
	 */
	@Test
	@Disabled("HAL")
	public void TurningStationaryOnlyGyro() {
		Rotation2d moduleDirection = new Rotation2d(0);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(0, 0, new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(0, 0, 0)));
		time++;
		
		gyroAngle = new Rotation2d(Math.PI / 4);
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(0, 0, new Rotation2d(Math.PI / 4))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(0, 0, Math.PI / 4)));
	}
	
	@Test
	@Disabled("HAL")
	public void Diagonal2Metres() {
		Rotation2d moduleDirection = new Rotation2d(Math.PI / 3);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		time++;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(2, moduleDirection), 
				new SwerveModuleState(2, moduleDirection), 
				new SwerveModuleState(2, moduleDirection), 
				new SwerveModuleState(2, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates2);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(1, Math.sqrt(3), new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(1, Math.sqrt(3), 0)));
	}
	
	@Test
	@Disabled("HAL")
	public void DiagonalNegative2Metres() {
		Rotation2d moduleDirection = new Rotation2d(-Math.PI/3);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		time++;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(2, moduleDirection), 
				new SwerveModuleState(2, moduleDirection), 
				new SwerveModuleState(2, moduleDirection), 
				new SwerveModuleState(2, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates2);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(1, -Math.sqrt(3), new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(1, -Math.sqrt(3), 0)));
	}
	
	/**
	 * This tests points the wheels -π/3 radians (-60 Degrees), and then drives backwards at 2 m/s.
	 * The equivalent would be a module direction of 7π/6 radians (210 degrees) driving forward at 2m/s.
	 * Hence it is a DoubleNegative.
	 */
	@Test
	@Disabled("HAL")
	public void DiagonalDoubleNegative2Metres() {
		Rotation2d moduleDirection = new Rotation2d(-Math.PI / 3);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		time++;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(-2, moduleDirection), 
				new SwerveModuleState(-2, moduleDirection), 
				new SwerveModuleState(-2, moduleDirection), 
				new SwerveModuleState(-2, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates2);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(-1, Math.sqrt(3), new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(-1, Math.sqrt(3), 0)));
	}
	
	@Test
	@Disabled("HAL")
	public void ForwardThenTurning() {
		Rotation2d moduleDirection = new Rotation2d(0);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		time += 0.5;
		
		SwerveModuleState[] moduleStates2 = {
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection), 
				new SwerveModuleState(1, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates2);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(0.5, 0, new Rotation2d(0))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(1, 0, 0)));
		
		time += 0.5;
		
		gyroAngle = new Rotation2d(Math.PI / 4);
		SwerveModuleState[] moduleStates3 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates3);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(0.5, 0, new Rotation2d(Math.PI / 4))));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(0, 0, Math.PI / 2)));
	}
	
	@Test
	@Disabled("HAL")
	public void UsingKinematics() {
		Rotation2d moduleDirection = new Rotation2d(0);
		Rotation2d gyroAngle = new Rotation2d(0);
		
		SwerveModuleState[] moduleStates1 = {
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection), 
				new SwerveModuleState(0, moduleDirection)
		};
		odometry.updateWithTime(time, gyroAngle, moduleStates1);
		ChassisSpeeds swerveVelocities = ChassisSpeeds.fromFieldRelativeSpeeds(2, 2, 0, gyroAngle);
		SwerveModuleState[] moduleStates2 = kinematics.toSwerveModuleStates(swerveVelocities);
		time++;
		
		odometry.updateWithTime(time, gyroAngle, moduleStates2);
		
		assertTrue(odometry.getPoseMeters().equals(new Pose2d(2, 2, gyroAngle)));
		assertTrue(odometry.getRobotVelocity().getAsTwist().equals(new Twist2d(2, 2, 0)));
	}
}

package redbacks.lib.control;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import redbacks.lib.control.DriveControlFactory;
import redbacks.lib.control.ProfiledDriveDirectionController;
import redbacks.lib.motion.TrapezoidProfile;

import static org.junit.jupiter.api.Assertions.*;

public class TestProfiledDriveDirectionController {
	Pose2d startPositon;
	
	private static final double
			MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC = 7.3932147,
			MAX_ROTATIONAL_ACCELERATION_RADIANS_PER_SEC_SQUARED = 0, // this value isn't particularly useful
    		MAX_VELOCITY_METRES_PER_SEC = 3.8,
			MAX_ACCELERATION_METRES_PER_SECOND_SQUARED = 3.98;
	
	private static final TrapezoidProfile.Constraints 
			forwardMotionConstraints = new TrapezoidProfile.Constraints(MAX_VELOCITY_METRES_PER_SEC, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED),
			rotationMotionConstraints = new TrapezoidProfile.Constraints(MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC, MAX_ROTATIONAL_ACCELERATION_RADIANS_PER_SEC_SQUARED);
	
	private static final DriveControlFactory controlFactory = new DriveControlFactory(0, 0, 0, 0, 0, 0, new Pose2d(), new Pose2d(), forwardMotionConstraints, rotationMotionConstraints);
	
	@BeforeEach
	void BeforeEach() {
		startPositon = new Pose2d(0, 0, new Rotation2d(0));
	}
	
	@AfterEach
	void AfterEach() {
		startPositon = null;
	}
	
	@Test
	void test0DegreeAngle(){
		Pose2d endPosition = new Pose2d(1, 0, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(0, controller.calculateAngle(startPositon, endPosition));
		assertEquals(controller.getStraightLineDistance(), 1);
	}
	
	@Test
	void test180DegreeAngle(){
		Pose2d endPosition = new Pose2d(-1, 0, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(180, controller.calculateAngle(startPositon, endPosition));
		assertEquals(controller.getStraightLineDistance(), 1);
	}
	
	@Test
	void test90DegreeAngle(){
		Pose2d endPosition = new Pose2d(0, 1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(90, controller.calculateAngle(startPositon, endPosition));
	}
	
	@Test
	void testNegative90DegreeAngle(){
		Pose2d endPosition = new Pose2d(0, -1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(-90, controller.calculateAngle(startPositon, endPosition));
	}
	
	@Test
	void test45DegreeAngle(){
		Pose2d endPosition = new Pose2d(1, 1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(45, controller.calculateAngle(startPositon, endPosition));
	}
	
	@Test
	void testNegative45DegreeAngle(){
		Pose2d endPosition = new Pose2d(1, -1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(-45, controller.calculateAngle(startPositon, endPosition));
	}
	
	@Test
	void test135DegreeAngle(){
		Pose2d endPosition = new Pose2d(-1, 1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(135, controller.calculateAngle(startPositon, endPosition));
	}
	
	@Test
	void testNegative135DegreeAngle(){
		Pose2d endPosition = new Pose2d(-1, -1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(-135, controller.calculateAngle(startPositon, endPosition));
	}
	
	@Test
	void test30DegreeAngle(){
		Pose2d endPosition = new Pose2d(Math.sqrt(3), 1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(30, controller.calculateAngle(startPositon, endPosition), 0.001);
	}
	
	@Test
	void testNegative30DegreeAngle(){
		Pose2d endPosition = new Pose2d(Math.sqrt(3), -1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(-30, controller.calculateAngle(startPositon, endPosition), 0.001);
	}
	
	@Test
	void test150DegreeAngle(){
		Pose2d endPosition = new Pose2d(-Math.sqrt(3), 1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(150, controller.calculateAngle(startPositon, endPosition), 0.001);
	}
	
	@Test
	void testNegative150DegreeAngle(){
		Pose2d endPosition = new Pose2d(-Math.sqrt(3), -1, new Rotation2d(0));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		assertEquals(-150, controller.calculateAngle(startPositon, endPosition), 0.001);
	}
	
	@Test
	@Disabled
	void SimulatePID(){
		Pose2d endPosition = new Pose2d(1, 0, Rotation2d.fromDegrees(45));
		ProfiledDriveDirectionController controller = controlFactory.createProfiledController(startPositon, new Pose2d(), endPosition, new Pose2d());
		Pose2d currentPose = startPositon;
		final int numCallsPerSecond = 50;
		double time = 0;
		controller.setNextPosition(startPositon, new Pose2d(), endPosition, new Pose2d(), 0);
		while(time < 2) {
			time = time + 1.0 / numCallsPerSecond;
			ChassisSpeeds value = controller.calculate(currentPose, time);
			System.out.println("X Velocity" + value.vxMetersPerSecond);
			currentPose = new Pose2d(
					currentPose.getTranslation().getX() + value.vxMetersPerSecond/numCallsPerSecond, 
					currentPose.getTranslation().getY() + value.vyMetersPerSecond/numCallsPerSecond, 
					currentPose.getRotation().plus(new Rotation2d(value.omegaRadiansPerSecond/numCallsPerSecond))
			);
			System.out.println("Current X: " + currentPose.getTranslation().getX());
			System.out.println("Current Y: " + currentPose.getTranslation().getY());
			System.out.println("Current Rotation: " + currentPose.getRotation());
		}
	}
}

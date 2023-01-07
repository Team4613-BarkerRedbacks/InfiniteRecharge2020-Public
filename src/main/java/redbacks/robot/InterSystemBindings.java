package redbacks.robot;

public class InterSystemBindings {
	protected void createBindings(Robot robot) {
		robot.aiming.getRobotPositionOutput().attach(
			(pose) -> robot.drivetrain.setPosition(pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation())
		);
		
		robot.aiming.getShooterExitVelocitySensor().populate(robot.shooter::getExitVelocity);
		
		robot.drivetrain.getOdometryOutput().attach(robot.aiming::updateOdometry);
		
		// FIXME
//		robot.indexer.getShooterOnTargetInput().populate(robot.aiming::isOnTarget);
		robot.indexer.getShooterAtTargetSpeedInput().populate(robot.shooter::isAtTargetSpeed); // TODO Choose between target and minimum speeds
		robot.indexer.getShooterAboveMinimumSpeedInput().populate(robot.shooter::isAboveMinimumSpeed);
	}
}

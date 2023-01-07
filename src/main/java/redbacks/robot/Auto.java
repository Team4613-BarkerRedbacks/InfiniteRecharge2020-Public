package redbacks.robot;

import arachne.lib.sequences.Actionable;
import static arachne.lib.sequences.Actionable.*;

import arachne.lib.sequences.actions.Action;
import arachne.lib.types.InOut;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.robot.subsystems.aiming.Aiming;

import java.util.function.Function;

public enum Auto implements Function<Robot, Actionable> {
	START_FROM_ANY_POSITION((robot) ->
		SEQUENCE(
			DO(robot.aiming::calculateRobotPosition),
			DO(() -> robot.shooter.spin(true)),
			shoot(robot, 15000)
		)
	),

	DO_NOTHING((robot) ->
		DO_NOTHING()
	),
	RENDEZVOUS_AUTO((robot) ->
		SEQUENCE(
				// Shoot 3 at start
				DO(() -> robot.drivetrain.setPosition(-3.54, 0.46, Rotation2d.fromDegrees(90))),
				DO(() -> robot.aiming.setState(Aiming.SystemState.SHOOT_WHILE_MOVING)),
				DO(() -> robot.shooter.spinWithPower(true)),
				shoot(robot, 600),
				DO(() -> robot.intake.deploy(true)),
				DO(() -> robot.intake.spin(InOut.IN)),
				// Pick up trench
				robot.drivetrain.doProfiledMoveTo(
						new Pose2d(-7.75, 0.9, Rotation2d.fromDegrees(90)),
						1.4
				),
				// Go to 2nd shooting position
				robot.drivetrain.doProfiledMoveTo(
						new Pose2d(-5.5, 0.9, Rotation2d.fromDegrees(90)), 4
				),
				DO(()-> robot.aiming.trackWithFixedHood(23)),
				shoot(robot, 2000),
				// Rotate
				robot.drivetrain.doProfiledMoveTo(
						new Pose2d(-5.5, 0.9, Rotation2d.fromDegrees(25)), 4
				),
				// Intake balls
				robot.drivetrain.doProfiledMoveTo(
						new Pose2d(-6.2, 3.35, Rotation2d.fromDegrees(25)), 1.75
				),
				robot.drivetrain.doProfiledMoveTo(
						new Pose2d(-5.5, 0.9, Rotation2d.fromDegrees(25)), 4
				),
				DO(()-> robot.aiming.trackWithFixedHood(23)),
				shoot(robot, 1000)
//				DO(robot.aiming::calculateRobotPosition),
				// Shoot the 2nd three
//				shoot(robot, 1500),
//				// Go to rendezvous and intake balls
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-7.4, 2.3, Rotation2d.fromDegrees(-15)), 2.5
//				),
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-7.6, 3, Rotation2d.fromDegrees(-15)), 2.5
//				)
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-7.2, 2.8, Rotation2d.fromDegrees(18-90)), 3
//				)
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-6.3, 3.3, Rotation2d.fromDegrees(18-90)), 2.5
//				)
//				),
//				shoot(robot, 2000)


//
//
//
//				// Shoot leftover balls from between trench and rendezvous
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-7, 2.62, Rotation2d.fromDegrees(20))
//				),
//				shoot(robot,1750),
//				// Intake in rendezvous
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-6.75, 3.35, Rotation2d.fromDegrees(-50))
//				),
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-6.7, 3.5, Rotation2d.fromDegrees(20))
//				),
//				robot.drivetrain.doProfiledMoveTo(
//						new Pose2d(-6.67, 3.93, Rotation2d.fromDegrees(20))
//				),
//				//Shoot from rendezvous
//				shoot(robot, 3000),
//				DO(() -> robot.shooter.spinWithPower(false)),
//				DO(() -> robot.intake.deploy(false))
		)
	),
	TRENCH_AUTO((robot) ->
		SEQUENCE(
			DO(() -> robot.drivetrain.setPosition(-3.54, 8.41, Rotation2d.fromDegrees(90))),
			DO(() -> robot.intake.deploy(true)),
			DO(() -> robot.intake.spin(InOut.IN)),
			DO(() -> robot.aiming.setState(Aiming.SystemState.SHOOT_WHILE_MOVING)),
			DO(() -> robot.shooter.spinWithPower(true)),
			robot.drivetrain.doProfiledMoveTo(
				new Pose2d(-5.79, 8.28, Rotation2d.fromDegrees(90)), 1.5
			),
			robot.drivetrain.doProfiledMoveTo(
				new Pose2d(-3.54, 4.7, Rotation2d.fromDegrees(90)), 3
			),
			DO(robot.aiming::calculateRobotPosition),
			WAIT(200),
			DO(() -> robot.intake.deploy(false)),
			WAIT(300),
			DO(() -> robot.intake.deploy(true)),
			WAIT(100),
			DO(() -> robot.intake.deploy(false)),
			shoot(robot, 10000)
		)
	);

	public static Actionable goToTrenchAndShoot(Robot robot){
		return SEQUENCE(

		);
	}

	public static Actionable shootThree(Robot robot){
		return SEQUENCE(
				DO(() -> robot.drivetrain.setPosition(-3.54, 0.46, Rotation2d.fromDegrees(90))),
				DO(() -> robot.aiming.setState(Aiming.SystemState.SHOOT_WHILE_MOVING)),
				DO(() -> robot.shooter.spinWithPower(true)),
				shoot(robot, 1500),
				DO(() -> robot.intake.deploy(true)),
				DO(() -> robot.intake.spin(InOut.IN)),
				// Pick up leftover balls at trench
				robot.drivetrain.doProfiledMoveTo(
						new Pose2d(-7.75, 0.7, Rotation2d.fromDegrees(90))
				)
		);
	}

	public static Actionable shoot(Robot robot, long wait) {
		return SEQUENCE(
				WAIT().UNSAFE_UNTIL(robot.shooter::isAboveMinimumSpeed),
				DO(robot.indexer::shootInAuto),
				WAIT(wait),
				DO(robot.indexer::index)
		);
	}

//	REVEAL((robot) ->
//			SEQUENCE(
//					DO(() -> robot.drivetrain.setPosition(-3.54, 0.46, Rotation2d.fromDegrees(90))),
//					DO(() -> robot.aiming.setState(Aiming.SystemState.SHOOT_WHILE_MOVING)),
//					DO(() -> robot.shooter.spin(true)),
//					DO(() -> robot.intake.getDeployInput().accept(true)),
//					// Trench
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-5.14, 0.70, Rotation2d.fromDegrees(90)),
//							3
//					),
//					WAIT().UNSAFE_UNTIL(robot.shooter::isAboveMinimumSpeed),
//					DO(robot.indexer::shootFast),
//					WAIT(1000),
//					DO(robot.indexer::index),
//					DO(() -> robot.intake.getIntakeInput().accept(true)),
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-7.54, 0.70, Rotation2d.fromDegrees(90)),
//							1.5
//					),
//					// 2 balls from center
//					robot.drivetrain.doProfiledMoveTo(new Pose2d(-7.54, 0.70, Rotation2d.fromDegrees(22.5))),
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-5.95, 2.30, Rotation2d.fromDegrees(22.5)),
//							3
//					),
//					DO(robot.indexer::shootFast),
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-6.00, 2.80, Rotation2d.fromDegrees(25)),
//							1.5
//					),
//					// 3 balls from other side of center
//					DO(robot.indexer::index),
//					robot.drivetrain.doProfiledMoveTo(new Pose2d(-6.00, 2.80, Rotation2d.fromDegrees(90))),
//					DO(() -> robot.intake.getDeployInput().accept(false)),
//					DO(robot.indexer::shootFast),
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-4.40, 2.40, Rotation2d.fromDegrees(90)),
//							1.5
//					),
//					DO(robot.indexer::index),
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-4.40, 4.10, Rotation2d.fromDegrees(90)),
//							3
//					),
//					DO(() -> robot.intake.getDeployInput().accept(true)),
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-5.20, 4.10, Rotation2d.fromDegrees(90)),
//							3
//					),
//					DO(robot.indexer::shootFast),
//					robot.drivetrain.doProfiledMoveTo(
//							new Pose2d(-5.60, 4.75, Rotation2d.fromDegrees(90)),
//							1.5
//					)
//			)
//	);


	private final Function<Robot, Actionable> actionableGenerator;

	private Auto(Function<Robot, Actionable> actionableGenerator) {
		this.actionableGenerator = actionableGenerator;
	}

	@Override
	public Actionable apply(Robot robot) {
		return actionableGenerator.apply(robot);
	}


}

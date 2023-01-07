package redbacks.robot.subsystems.aiming;

import edu.wpi.first.networktables.NetworkTableInstance;
import redbacks.lib.hardware.Limelight;
import redbacks.lib.hardware.Limelight.LedMode;
import redbacks.robot.subsystems.aiming.hood.HoodHardware;
import redbacks.robot.subsystems.aiming.turret.TurretHardware;
import redbacks.robot.types.LimelightPipeline;

public class AimingHardware {
	public final HoodHardware hood = new HoodHardware();
	public final TurretHardware turret = new TurretHardware();

	public final Limelight<LimelightPipeline> limelight = new Limelight<LimelightPipeline>(
		NetworkTableInstance.getDefault().getTable("limelight")
	);

	public void initialize() {
		limelight.setPipeline(LimelightPipeline.ODOMETRY_3D);
		limelight.setLedMode(LedMode.ON);

		hood.initialize();
		turret.initialize();
	}
}

package redbacks.robot;

import arachne.lib.ArachneRobot;
import arachne.lib.AutoManager;
import arachne.lib.dashboard.Dashboard;
import arachne.lib.dashboard.DefaultDashboard;
import arachne.lib.game.GameState;
import edu.wpi.first.wpilibj.DriverStation;
import redbacks.robot.subsystems.aiming.Aiming;
import redbacks.robot.subsystems.aiming.AimingBindings;
import redbacks.robot.subsystems.aiming.AimingHardware;
import redbacks.robot.subsystems.climber.Climber;
import redbacks.robot.subsystems.climber.ClimberBindings;
import redbacks.robot.subsystems.climber.ClimberHardware;
import redbacks.robot.subsystems.drivetrain.Drivetrain;
import redbacks.robot.subsystems.drivetrain.DrivetrainHardware;
import redbacks.robot.subsystems.drivetrain.DrivetrainBindings;
import redbacks.robot.subsystems.indexer.Indexer;
import redbacks.robot.subsystems.indexer.IndexerBindings;
import redbacks.robot.subsystems.indexer.IndexerHardware;
import redbacks.robot.subsystems.intake.Intake;
import redbacks.robot.subsystems.intake.IntakeBindings;
import redbacks.robot.subsystems.intake.IntakeHardware;
import redbacks.robot.subsystems.shooter.Shooter;
import redbacks.robot.subsystems.shooter.ShooterBindings;
import redbacks.robot.subsystems.shooter.ShooterHardware;

/**
 * @author Roy Cai, Will Bray, Pete (Heedo) Choung, Ashton Barker, Jess Samuelson, Ben Schwarz
 */
public class Robot extends ArachneRobot {
	public static final double LOOP_PERIOD = 0.01;

	public static void main(String[] args) {
		startRobot(Robot::new);
	}

	public Robot() {
		super(LOOP_PERIOD);
	}

	public final Controllers controllers = new Controllers();

	public final Drivetrain drivetrain = new Drivetrain();
	public final DrivetrainHardware drivetrainHardware = new DrivetrainHardware();

	public final Intake intake = new Intake();
	public final IntakeHardware intakeHardware = new IntakeHardware();

	public final Aiming aiming = new Aiming();
	public final AimingHardware aimingHardware = new AimingHardware();

	public final Shooter shooter = new Shooter();
	public final ShooterHardware shooterHardware = new ShooterHardware();

	public final Climber climber = new Climber();
	public final ClimberHardware climberHardware = new ClimberHardware();

	public final Indexer indexer = new Indexer();
	public final IndexerHardware indexerHardware = new IndexerHardware();

	public final InterSystemBindings interSystemBindings = new InterSystemBindings();

	public AutoManager<Robot, Auto> autos;

	@Override
	protected void initialize() {
		Dashboard.setImplementation(new DefaultDashboard());

		drivetrain.initialize();
		drivetrainHardware.initialize();
		DrivetrainBindings.bind(drivetrain, drivetrainHardware, controllers);

		intake.initialize();
		intakeHardware.initialize();
		IntakeBindings.bind(intake, intakeHardware, controllers);

		aiming.initialize();
		aimingHardware.initialize();
		AimingBindings.bind(aiming, aimingHardware, controllers);

		shooter.initialize();
		shooterHardware.initialize();
		ShooterBindings.bind(shooter, shooterHardware, controllers);

		climber.initialize();
		climberHardware.initialize();
		ClimberBindings.bind(climber, climberHardware, controllers);

		indexer.initialize();
		indexerHardware.initialize();
		IndexerBindings.bind(indexer, indexerHardware, controllers);

		interSystemBindings.createBindings(this);

		autos = new AutoManager<Robot, Auto>(this, Auto.DO_NOTHING, Auto.values());
	}

	@Override
	protected void onStateChange(GameState oldState, GameState newState) {
		if(newState == GameState.AUTO) {
			autos.startAuto();
		}

		if(oldState == GameState.AUTO) {
			autos.stopAuto();
			intake.reset();
			shooter.spin(false);
			drivetrain.setState(Drivetrain.SystemState.DRIVER_CONTROLLED);
			indexer.index();
		}
	}

	@Override
	protected void execute(GameState state) {
		// Subsystems
		if(state != GameState.DISABLED) {
			drivetrain.run();
			intake.run();
			aiming.run();
			shooter.run();
			climber.run();
			indexer.run();
		}

		autos.run();

		String colorWheelColor =  DriverStation.getInstance().getGameSpecificMessage();
		if(colorWheelColor.length() > 0) {
			char color = colorWheelColor.charAt(0);
			Dashboard.getInstance().putBoolean("Blue", color == 'B');
			Dashboard.getInstance().putBoolean("Green", color == 'G');
			Dashboard.getInstance().putBoolean("Red", color == 'R');
			Dashboard.getInstance().putBoolean("Yellow", color == 'Y');
		}

		Dashboard.getInstance().putNumber("LF", drivetrainHardware.swerveModuleAngleLF.get());
		Dashboard.getInstance().putNumber("LB", drivetrainHardware.swerveModuleAngleLB.get());
		Dashboard.getInstance().putNumber("RF", drivetrainHardware.swerveModuleAngleRF.get());
		Dashboard.getInstance().putNumber("RB", drivetrainHardware.swerveModuleAngleRB.get());
	}
}

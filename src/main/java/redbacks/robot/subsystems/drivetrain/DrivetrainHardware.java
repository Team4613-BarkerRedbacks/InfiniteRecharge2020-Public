package redbacks.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import arachne.lib.io.Gettable;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.SettableDoubleSensor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import redbacks.robot.Constants;

import static arachne.tapestry.LogicTapestry.wrapAround;

// TODO Create ModuleHardware and ModuleBindings
public class DrivetrainHardware {
	// Constants
	private static final double
		SWERVE_MODULE_ZERO_ANGLE_LF = -117.5+36.5,
		SWERVE_MODULE_ZERO_ANGLE_LB = -111.8,
		SWERVE_MODULE_ZERO_ANGLE_RF = -8.4,
		SWERVE_MODULE_ZERO_ANGLE_RB = 28.5;

	private static final double
		SWERVE_MODULE_VELOCITY_KP = 0.0000001 * 240,
		SWERVE_MODULE_VELOCITY_KI = 0,
		SWERVE_MODULE_VELOCITY_KD = 0.000001 * 350,
		SWERVE_MODULE_VELOCITY_MAX_PER_SECOND = 21900,
		SWERVE_MODULE_VELOCITY_KF = 0.0467,
		SWERVE_MODULE_VELOCITY_MAX_PID_OUTPUT = 1;

	// Outputs
	public final CANSparkMax
		motorSteerLF = new CANSparkMax(10, MotorType.kBrushless),
		motorSteerRF = new CANSparkMax(11, MotorType.kBrushless),
		motorSteerLB = new CANSparkMax(12, MotorType.kBrushless),
		motorSteerRB = new CANSparkMax(13, MotorType.kBrushless);

	public final WPI_TalonFX
		motorDriveLF = new WPI_TalonFX(0),
		motorDriveRF = new WPI_TalonFX(1),
		motorDriveLB = new WPI_TalonFX(2),
		motorDriveRB = new WPI_TalonFX(3);

	// Inputs
	public final GettableDouble
		swerveModuleAngleLF = GettableDouble
			.create(new AnalogPotentiometer(
				new AnalogInput(0),
				SwerveModule.ANGLE_RANGE,
				SwerveModule.ANGLE_MIN + SWERVE_MODULE_ZERO_ANGLE_LF)::get
			).change(wrapAround(SwerveModule.ANGLE_MIN, SwerveModule.ANGLE_MAX)),

		swerveModuleAngleRF = GettableDouble
			.create(new AnalogPotentiometer(
				new AnalogInput(1),
				SwerveModule.ANGLE_RANGE,
				SwerveModule.ANGLE_MIN + SWERVE_MODULE_ZERO_ANGLE_RF)::get
			).change(wrapAround(SwerveModule.ANGLE_MIN, SwerveModule.ANGLE_MAX)),

		swerveModuleAngleLB = GettableDouble
			.create(new AnalogPotentiometer(
				new AnalogInput(2),
				SwerveModule.ANGLE_RANGE,
				SwerveModule.ANGLE_MIN + SWERVE_MODULE_ZERO_ANGLE_LB)::get
			).change(wrapAround(SwerveModule.ANGLE_MIN, SwerveModule.ANGLE_MAX)),

		swerveModuleAngleRB = GettableDouble
			.create(new AnalogPotentiometer(
				new AnalogInput(3),
				SwerveModule.ANGLE_RANGE,
				SwerveModule.ANGLE_MIN + SWERVE_MODULE_ZERO_ANGLE_RB)::get
			).change(wrapAround(SwerveModule.ANGLE_MIN, SwerveModule.ANGLE_MAX));

	private final TalonFXSensorCollection
		_swerveModuleDistanceLF = motorDriveLF.getSensorCollection(),
		_swerveModuleDistanceLB = motorDriveLB.getSensorCollection(),
		_swerveModuleDistanceRF = motorDriveRF.getSensorCollection(),
		_swerveModuleDistanceRB = motorDriveRB.getSensorCollection();

	// Encoder mappings (Falcon)
	public final SettableDoubleSensor
		swerveModuleDistanceLF = SettableDoubleSensor.create(
			_swerveModuleDistanceLF::getIntegratedSensorPosition,
			(value) -> _swerveModuleDistanceLF.setIntegratedSensorPosition((int) value, Constants.CAN_TIMEOUT)
		),
		swerveModuleDistanceLB = SettableDoubleSensor.create(
			_swerveModuleDistanceLB::getIntegratedSensorPosition,
			(value) -> _swerveModuleDistanceLB.setIntegratedSensorPosition((int) value, Constants.CAN_TIMEOUT)
		),
		swerveModuleDistanceRF = SettableDoubleSensor.create(
			_swerveModuleDistanceRF::getIntegratedSensorPosition,
			(value) -> _swerveModuleDistanceRF.setIntegratedSensorPosition((int) value, Constants.CAN_TIMEOUT)
		),
		swerveModuleDistanceRB = SettableDoubleSensor.create(
			_swerveModuleDistanceRB::getIntegratedSensorPosition,
			(value) -> _swerveModuleDistanceRB.setIntegratedSensorPosition((int) value, Constants.CAN_TIMEOUT)
		);

	public final GettableDouble
		swerveModuleVelocityLF = _swerveModuleDistanceLF::getIntegratedSensorVelocity,
		swerveModuleVelocityLB = _swerveModuleDistanceLB::getIntegratedSensorVelocity,
		swerveModuleVelocityRF = _swerveModuleDistanceRF::getIntegratedSensorVelocity,
		swerveModuleVelocityRB = _swerveModuleDistanceRB::getIntegratedSensorVelocity;

	public final AHRS navx = new AHRS(SPI.Port.kMXP); // TODO Make private

	// TODO Determine whether to access through the navx object
	public final GettableDouble yaw = navx::getYaw;

	public void initialize() {
		navx.reset();

		WPI_TalonFX[] driveMotors = {
			motorDriveLF,
			motorDriveRF,
			motorDriveLB,
			motorDriveRB
		};

		for(WPI_TalonFX motor : driveMotors) {
			motor.configFactoryDefault(Constants.CAN_TIMEOUT);

			motor.setNeutralMode(NeutralMode.Brake);
			motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

	        motor.selectProfileSlot(0, 0);
	        motor.config_kP(0, SWERVE_MODULE_VELOCITY_KP, Constants.CAN_TIMEOUT);
	        motor.config_kI(0, SWERVE_MODULE_VELOCITY_KI, Constants.CAN_TIMEOUT);
	        motor.config_kD(0, SWERVE_MODULE_VELOCITY_KD, Constants.CAN_TIMEOUT);
	        motor.config_kF(0, SWERVE_MODULE_VELOCITY_KF, Constants.CAN_TIMEOUT);
		}

		CANSparkMax[] steerMotors = {
			motorSteerLF,
			motorSteerLB,
			motorSteerRF,
			motorSteerRB
		};

		for(CANSparkMax motor : steerMotors) {
			motor.restoreFactoryDefaults();
			motor.setIdleMode(IdleMode.kBrake);
			motor.burnFlash();
		}
	}
}

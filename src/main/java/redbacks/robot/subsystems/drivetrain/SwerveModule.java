package redbacks.robot.subsystems.drivetrain;

import arachne.lib.io.GettableDouble;
import arachne.lib.io.SettableDouble;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.io.sensors.SettableDoubleInput;
import arachne.lib.io.sensors.SettableDoubleSensor;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSink;
import arachne.lib.systems.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * @author Darin Huang, Benjamin Schwarz
 */
public class SwerveModule extends Subsystem {
    // Inputs
    private final DoubleSink targetAngleInput, targetVelocityInput;

	// Outputs
    private final DoublePipe steerOutput, targetVelocityOutput;
    
    // Pipes
    private final DoublePipe voltagePipe;
	
	// Sensors
	private final GettableDoubleInput angleSensor, velocitySensor;
	private final SettableDoubleInput distanceSensor;
	
	// Controllers
    private final PIDController steerPID;
	
	// Control variables
	private boolean isRotationFlipped;
    
	/*
	 * Internal constants
	 * 
	 * Constants related to Wheel Velocity (These are here for easy access):
	 * - Encoder Counts Per Shaft Revolution: 2048
	 * - Wheel to Shaft Ratio 1:8.31
	 * - Wheel Radius: 0.05 metres
	 */
	private static final double
			WHEEL_RADIUS = 0.05,
			SHAFT_TO_WHEEL_RATIO = 8.31, // 1 rotation of wheel from 8.31 shaft rotations
			ENCODER_COUNTS_PER_SHAFT_REVOLUTION = 2048,
			SECONDS_FROM_100MS_MULTIPLIER = 10,
			WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS,
			ENCODER_COUNTS_PER_WHEEL_ROTATION = ENCODER_COUNTS_PER_SHAFT_REVOLUTION * SHAFT_TO_WHEEL_RATIO,
			ENCODER_VELOCITY_TO_METRES_PER_SECOND_MULTIPLIER = (WHEEL_CIRCUMFERENCE * SECONDS_FROM_100MS_MULTIPLIER) / ENCODER_COUNTS_PER_WHEEL_ROTATION,
    		ANGLE_KP = 0.0055,
    		ANGLE_KI = 0,
    		ANGLE_KD = 0,
    		MAX_WHEEL_ENCODER_VELOCITY_PER_100ms = 21900, // TODO Get a non-freespin value
    		MAX_VELOCITY_METRES_PER_SEC = 4.04;

	// External constants
	public static final double
			ANGLE_MIN = -180,
			ANGLE_MAX = 180,
			ANGLE_RANGE = ANGLE_MAX - ANGLE_MIN;
	
	public SwerveModule() {
        this.targetAngleInput = new DoubleSink(0);
        this.targetVelocityInput = new DoubleSink(0);
        
        this.steerOutput = new DoublePipe(0);
        this.targetVelocityOutput = new DoublePipe(0);

        this.voltagePipe = new DoublePipe(0);

        this.angleSensor = new GettableDoubleInput();
        this.velocitySensor = new GettableDoubleInput();
        this.distanceSensor = new SettableDoubleInput();

        this.steerPID = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);
        this.steerPID.enableContinuousInput(ANGLE_MIN, ANGLE_MAX);
        this.steerPID.setTolerance(0);
        
        this.isRotationFlipped = false;
    }

	@Override
    public void initialize() {
		super.initialize();
		
		// Flip target angle and velocity
		targetAngleInput.setModifier((targetAngle) -> {
			double absAngleDiff = Math.abs(targetAngle - angleSensor.get());
			
			isRotationFlipped = absAngleDiff > ANGLE_RANGE / 4 && absAngleDiff < ANGLE_RANGE * 3 / 4;
			
			return isRotationFlipped ? flipAngle(targetAngle) : targetAngle;
		});

		// Translate target velocity from m/s to encoder ticks
		targetVelocityInput.setModifier((metresPerSec) -> metresPerSec / MAX_VELOCITY_METRES_PER_SEC * MAX_WHEEL_ENCODER_VELOCITY_PER_100ms);
    }
	
	@Override
	public void run() {
		super.run();
		
		double targetVelocity = targetVelocityInput.get();
		
		steerOutput.accept(targetVelocity == 0 ? 0 : -steerPID.calculate(angleSensor.get(), targetAngleInput.get()));
		targetVelocityOutput.accept(targetVelocity * (isRotationFlipped ? -1 : 1));
		
	}
	
    private double flipAngle(double angle) {
    	angle += ANGLE_RANGE / 2;
    	
		if(angle > ANGLE_MAX) angle -= ANGLE_RANGE;
		
		return angle;
    }
    
    public SettableDouble getVoltageInput() {
        return voltagePipe;
    }

    public DoubleSource getDriveOutput() {
        return voltagePipe;
    }

    public SettableDouble getTargetAngleInput() {
        return targetAngleInput;
    }

    public DoubleSource getSteerOutput() {
        return steerOutput;
    }
	
	public SettableDouble getTargetVelocityMetresPerSecInput() {
		return targetVelocityInput;
	}
	
	public DoubleSource getTargetVelocityTicksPer100msOutput() {
		return targetVelocityOutput;
	}
	
    public Input<GettableDouble> getAngleSensor() {
        return angleSensor;
    }
   
	public double getAngle() {
		return angleSensor.get();
	}
    
    public Input<GettableDouble> getVelocitySensor() {
        return velocitySensor;
    }
   
	public double getVelocity() {
		return velocitySensor.get() * ENCODER_VELOCITY_TO_METRES_PER_SECOND_MULTIPLIER;
	}
    
    public Input<SettableDoubleSensor> getDistanceSensor() {
        return distanceSensor;
    }
    
 	public double getDistance() {
 		return distanceSensor.get();
 	}
    
 	public void resetDistance() {
 		distanceSensor.reset();
 	}
}

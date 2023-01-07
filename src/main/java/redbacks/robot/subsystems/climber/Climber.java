package redbacks.robot.subsystems.climber;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.io.sensors.SettableDoubleInput;
import arachne.lib.io.sensors.SettableDoubleSensor;
import arachne.lib.listeners.Signal;
import arachne.lib.logic.ArachneMath;
import arachne.lib.pipeline.BooleanPipe;
import arachne.lib.pipeline.BooleanSource;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.sequences.actions.Action;
import arachne.lib.states.State;
import arachne.lib.states.StateTransition;
import arachne.lib.states.StatefulSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

import static arachne.lib.sequences.Actionable.*;
import static arachne.lib.logic.DoubleComparison.greaterThan;

import java.util.Arrays;
import java.util.List;

public final class Climber extends StatefulSubsystem<Climber.SystemState, Climber> {
    // Outputs
    private final DoublePipe powerOutput;
    private final BooleanPipe unlockOutput, platformUnlockOutput;
    
    // Signals
    private final Signal raiseHooksSignal, raiseRobotSignal;
    
    // Sensors
    private final GettableBooleanInput robotAtMaxHeightSensor;
    private final SettableDoubleInput positionSensor;
    
    // Controllers
    private final PIDController raiseHookPID;
    
    // Control variables
    private boolean isPlatformDeployed = false;

    // Constants
    private static final double
    		MAX_SPEED = 0.6, // Maximum speed of motors
    		LOCK_RETRACT_DURATION_SECONDS = 0.5,
			
    		// TODO Convert to ~meters
			RAISE_HOOK_POSITION = -2.25e5, // Position of top
			HOOK_KP = Math.abs(1 / (RAISE_HOOK_POSITION * 0.2)),
    		HOOK_KI = 0,
    		HOOK_KD = 0,
    		HOOK_MAX_SPEED_PERCENTAGE = 1,
    		HOOK_POSITION_TOLERANCE = 5e3,

    		RAISE_ROBOT_INITIAL_SPEED = 1,
    		RAISE_ROBOT_APPROACH_SPEED = 0.5,
    		RAISE_ROBOT_SLOWDOWN_POINT = 1.5e4,
    		
			RAISE_PLATFORM_POSITION = 1e5; // Height to raise platform TODO measure, determine usefulness

    public enum SystemState implements State<SystemState, Climber> {
        OPERATOR_CONTROL,
        NOT_CLIMBED {
        	@Override
        	public List<StateTransition<SystemState>> getTransitions(Climber climber) {
        		return Arrays.asList(
        			new StateTransition<SystemState>(climber.raiseHooksSignal, RAISE_HOOKS)
        		);
        	}
        },
        RAISE_HOOKS {
        	@Override
        	public Action createStateAction(Climber climber) {
        		return SEQUENCE(
            		DO(() -> climber.raiseHookPID.reset()),
            		DO(() -> climber.unlockOutput.accept(true)),
            		WAIT((long) (LOCK_RETRACT_DURATION_SECONDS * 1000)),
            		REPEAT(
            			() -> climber.powerOutput.accept(
            				ArachneMath.inBounds(
        						climber.raiseHookPID.calculate(climber.positionSensor.get(), RAISE_HOOK_POSITION),
        						-HOOK_MAX_SPEED_PERCENTAGE,
        						HOOK_MAX_SPEED_PERCENTAGE
            				)
            			)
            		).UNSAFE_UNTIL(climber.raiseHookPID::atSetpoint),
            		DO(() -> climber.powerOutput.accept(0)),
            		DO(() -> climber.setState(HOOK_AT_TOP))
            	).asAction(null);
        	}
        	
        	@Override
            public void deconstructState(Climber climber) {
                climber.powerOutput.accept(0);
        	}
        },
        HOOK_AT_TOP {
        	@Override
        	public List<StateTransition<SystemState>> getTransitions(Climber climber) {
        		return Arrays.asList(
        			new StateTransition<SystemState>(climber.raiseRobotSignal, RAISE_ROBOT)
        		);
        	}
        	
        	@Override
        	public void constructState(Climber climber) {
        		climber.unlockOutput.accept(false);
        		climber.powerOutput.accept(0);
        		climber.isPlatformDeployed = false;
        	}
        },
        RAISE_ROBOT {
        	@Override
        	public Action createStateAction(Climber climber) {
        		return SEQUENCE(
            		DO(() -> climber.platformUnlockOutput.accept(true)),
            		DO(() -> climber.powerOutput.accept(RAISE_ROBOT_INITIAL_SPEED)),
            		WAIT().UNSAFE_UNTIL(climber.positionSensor.is(greaterThan(RAISE_ROBOT_SLOWDOWN_POINT))),
            		DO(() -> climber.powerOutput.accept(RAISE_ROBOT_APPROACH_SPEED)),
            		WAIT().UNSAFE_UNTIL(climber.robotAtMaxHeightSensor),
            		DO(() -> climber.powerOutput.accept(0)),
            		DO(() -> climber.setState(ROBOT_AT_TOP))
            	).asAction(null);
        	}
        	
        	@Override
            public void deconstructState(Climber climber) {
                climber.powerOutput.accept(0);
        	}
        },
        ROBOT_AT_TOP;
    }

    public Climber() {
		super(SystemState.NOT_CLIMBED, SystemState.class);
    	
		this.raiseHooksSignal = new Signal();
		this.raiseRobotSignal = new Signal();

        this.powerOutput = new DoublePipe(0);
        this.unlockOutput = new BooleanPipe(false);
        this.platformUnlockOutput = new BooleanPipe(false);
        
        this.robotAtMaxHeightSensor = new GettableBooleanInput();
		this.positionSensor = new SettableDoubleInput();
        
        this.raiseHookPID = new PIDController(HOOK_KP, HOOK_KI, HOOK_KD);
        this.raiseHookPID.setTolerance(HOOK_POSITION_TOLERANCE);
    }

    @Override
    public void run() {
        super.run();

        Dashboard.getInstance().putBoolean("Climber limit switch", robotAtMaxHeightSensor.get());
    }

    @Override
	protected Climber getSelf() {
		return this;
    }

    public void move(double power) {
		if(state == SystemState.OPERATOR_CONTROL || state == SystemState.HOOK_AT_TOP || state == SystemState.ROBOT_AT_TOP) {
			powerOutput.accept(power * MAX_SPEED);
		}
    }

	public void unlock(boolean unlock) {
		if(state == SystemState.OPERATOR_CONTROL || state == SystemState.HOOK_AT_TOP || state == SystemState.ROBOT_AT_TOP) {
			unlockOutput.accept(unlock);
		}
	}
    
	public void deployPlatform(boolean deploy) {
		if(state == SystemState.OPERATOR_CONTROL || (deploy && state == SystemState.HOOK_AT_TOP)) {
			isPlatformDeployed = deploy;
			platformUnlockOutput.accept(deploy);
		}
	}

    public void raiseHooks() {
        raiseHooksSignal.fire();
    }

    public void raiseRobot() {
        if(isPlatformDeployed) raiseRobotSignal.fire();
    }
    
    public DoubleSource getPowerOutput() {
        return powerOutput;
    }

    public BooleanSource getUnlockOutput() {
        return unlockOutput;
    }
    
	public BooleanSource getPlatformUnlockOutput() {
		return platformUnlockOutput;
	}

    public Input<GettableBoolean> getRobotAtMaxHeightSensor() {
    	return robotAtMaxHeightSensor;
    }

    public Input<SettableDoubleSensor> getPositionSensor() {
    	return positionSensor;
    }
}
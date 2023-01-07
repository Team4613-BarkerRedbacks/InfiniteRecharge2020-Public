package redbacks.robot.subsystems.indexer;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.pipeline.BooleanPipe;
import arachne.lib.pipeline.BooleanSource;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;

public class Indexer extends StatefulSubsystem<Indexer.SystemState, Indexer> {
	// Outputs
	private final DoublePipe
			powerOutput = new DoublePipe(0),
			targetPositionOutput = new DoublePipe(0);

	private final BooleanPipe
			enableTowerOutput = new BooleanPipe(false),
			enableWheelOutput = new BooleanPipe(false);

	// Inputs
	private final GettableBooleanInput
			ballAtTop = new GettableBooleanInput(),
			ballAtBottom = new GettableBooleanInput(),
			shooterAtTargetSpeed = new GettableBooleanInput(),
			shooterAboveMinimumSpeed = new GettableBooleanInput();

	private final GettableDoubleInput position = new GettableDoubleInput();

	// Control Variables
	private double endOfPidTimeMillis = Double.MIN_VALUE;
	private boolean
		hasPendingTarget = false,
		isDisabled = false;

	// Constants
	private static final double
			INDEX_SPEED = 0.6,//(4.5 / 5.5),
			SHOOT_AUTO_SPEED = 0.75,
			SHOOT_TELEOP_SPEED = 0.6,
			SEPARATION_SPEED = 0.3,
			SPINNING_SPEED = -0.2,
			EJECT_SPEED = -0.6;

	private static final int
			BALL_SEPARATION = 17000,
			TOWER_TIME_DELAY_MILLIS = 300;

	public static enum SystemState implements State<Indexer.SystemState, Indexer> {
		INDEXING {
			@Override
			public void run(Indexer indexer) {
				Dashboard.getInstance().putNumber("Indexer position", indexer.position.get());
				Dashboard.getInstance().putNumber("Indexer target", indexer.targetPositionOutput.get());

				if(indexer.ballAtTop.get() || indexer.isDisabled) {
					indexer.powerOutput.accept(0);
					indexer.enableTowerOutput.accept(false);

					indexer.hasPendingTarget = false;
				}
				else if(indexer.ballAtBottom.get()) {
					indexer.powerOutput.accept(SEPARATION_SPEED);

					indexer.targetPositionOutput.accept(indexer.position.get() + BALL_SEPARATION);
					indexer.endOfPidTimeMillis = System.currentTimeMillis() + TOWER_TIME_DELAY_MILLIS;

					indexer.enableTowerOutput.accept(true);
					indexer.hasPendingTarget = true;
				}
				else {
					if(indexer.hasPendingTarget && indexer.position.get() < indexer.targetPositionOutput.get()) {
						indexer.powerOutput.accept(SEPARATION_SPEED);
						indexer.enableTowerOutput.accept(true);

						indexer.endOfPidTimeMillis = System.currentTimeMillis() + TOWER_TIME_DELAY_MILLIS;
					}
					else if(System.currentTimeMillis() < indexer.endOfPidTimeMillis) {
						indexer.powerOutput.accept(0);
						indexer.enableTowerOutput.accept(false);
					}
					else {
						indexer.powerOutput.accept(INDEX_SPEED);
						indexer.enableTowerOutput.accept(false);

						indexer.hasPendingTarget = false;
					}
				}
			}
		},
		SHOOTING_AUTO {
			@Override
			public void run(Indexer indexer) {
				indexer.enableTowerOutput.accept(true);
				indexer.powerOutput.accept(SHOOT_AUTO_SPEED);
			}
		},
		SHOOTING_TELEOP {
			@Override
			public void run(Indexer indexer) {
				indexer.enableTowerOutput.accept(true);
				indexer.powerOutput.accept(SHOOT_TELEOP_SPEED);
			}
		},
		SPIN_CONTROL_PANEL {
			@Override
			public void run(Indexer indexer) {
				indexer.enableWheelOutput.accept(true);
				indexer.enableTowerOutput.accept(true);
				indexer.powerOutput.accept(SPINNING_SPEED);
			}

			@Override
			public void deconstructState(Indexer indexer) {
				indexer.enableWheelOutput.accept(false);
			}
		},
		EJECTING {
			@Override
			public void run(Indexer indexer) {
				indexer.enableTowerOutput.accept(true);
				indexer.powerOutput.accept(EJECT_SPEED);
			}
		}
	}

	public Indexer() {
		super(SystemState.INDEXING, SystemState.class);
	}

	@Override
	protected Indexer getSelf() {
		return this;
	}

	@Override
	public void run() {
		super.run();

		Dashboard.getInstance().putBoolean("wheel output", enableWheelOutput.get());
		Dashboard.getInstance().putBoolean("Ball at Bottom", ballAtBottom.get());
		Dashboard.getInstance().putBoolean("Ball at Top", ballAtTop.get());
	}

	public void index() {
		setState(SystemState.INDEXING);
	}

	public void shootInAuto() {
		setState(SystemState.SHOOTING_AUTO);
	}

	public void shootInTeleOp() {
		setState(SystemState.SHOOTING_TELEOP);
	}

	public void tryToShoot() {
		if(shooterAboveMinimumSpeed.get()) {
			shootInTeleOp();
		}
	}

	public void spinControlPanel() {
		setState(SystemState.SPIN_CONTROL_PANEL);
	}

	public void eject() {
		setState(SystemState.EJECTING);
	}

	public void setDisabled(boolean disable) {
		isDisabled = disable;
	}

	public DoubleSource getPowerOutput(){
		return powerOutput;
	}

	public DoubleSource getTargetPositionOutput(){
		return targetPositionOutput;
	}

	public BooleanSource getEnableTowerOutput() {
		return enableTowerOutput;
	}

	public BooleanSource getEnableWheelOutput() {
		return enableWheelOutput;
	}

	public Input<GettableBoolean> getBallAtTopInput() {
		return ballAtTop;
	}

	public Input<GettableBoolean> getBallAtBottomInput() {
		return ballAtBottom;
	}

	public Input<GettableBoolean> getShooterAtTargetSpeedInput() {
		return shooterAtTargetSpeed;
	}

	public Input<GettableBoolean> getShooterAboveMinimumSpeedInput() {
		return shooterAboveMinimumSpeed;
	}

	public Input<GettableDouble> getPositionInput() {
		return position;
	}
}

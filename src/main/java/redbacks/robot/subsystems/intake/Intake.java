package redbacks.robot.subsystems.intake;

import arachne.lib.pipeline.BooleanPipe;
import arachne.lib.pipeline.BooleanSource;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.sequences.Actionable;
import arachne.lib.sequences.actions.Action;
import arachne.lib.systems.Subsystem;
import arachne.lib.types.InOut;

public class Intake extends Subsystem {
    private final BooleanPipe deployOutput = new BooleanPipe(false);
    private final DoublePipe rollerOutput = new DoublePipe(0);

    // TODO Change to proper inputs
    private DoublePipe currentDraw;

    private double currentMeasureTime;
    private boolean hasBall = false;

    // Constants
    private static final double
    		INTAKE_SPEED = 1,
            OUTTAKE_SPEED = -1,
            CURRENT_MEASURE_DELAY = 500,
            CURRENT_MEASURE_HIGH = 20;

    public Intake() {
        this.currentDraw = new DoublePipe(0);
    }

    public void deploy(boolean activate) {
        deployOutput.accept(activate);
    }

    public void spin(InOut direction) {
        rollerOutput.accept(
              direction == InOut.IN ? INTAKE_SPEED
            : direction == InOut.OUT ? OUTTAKE_SPEED
            : 0
        );
    }

    @Override
    public void run() {
        super.run();

        if (currentDraw.get() > CURRENT_MEASURE_HIGH && System.currentTimeMillis() > currentMeasureTime) {
            hasBall = true;
        }
    }

    public void reset() {
        deploy(false);
        spin(InOut.NONE);
    }

    public void resetIsBallInIntake() {
        hasBall = false;
    }

    public void enableIntakeWithCurrentMeasure() {
        currentMeasureTime = System.currentTimeMillis() + CURRENT_MEASURE_DELAY;
        spin(InOut.IN);
    }

    public DoubleSource getRollerOutput() {
        return rollerOutput;
    }

	public BooleanSource getDeployOutput() {
		return deployOutput;
    }

    // TODO Refactor
    public DoublePipe getCurrentDraw() {
        return currentDraw;
    }

    public Actionable intakeWhileChecking(double duration) {
        return (host) -> new Action(host) {
            private double startTime, passedTime;

            @Override
            public void initialize() {
                hasBall = false;
                startTime = System.currentTimeMillis();
            }

            @Override
            public void execute() {
                spin(InOut.IN);

                passedTime = System.currentTimeMillis() - startTime;
                if (passedTime > CURRENT_MEASURE_DELAY) {
                    if (currentDraw.get() > 20) {
                        hasBall = true;
                    }
                }
            }

            @Override
            protected boolean isFinished() {
                return passedTime > duration;
            }

            @Override
            protected void end() {
                spin(InOut.NONE);
            }
        };
    }
}

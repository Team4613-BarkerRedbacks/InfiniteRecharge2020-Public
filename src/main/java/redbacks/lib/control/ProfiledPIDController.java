package redbacks.lib.control;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import redbacks.lib.motion.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile. Users should call reset() when they first start running the controller
 * to avoid unwanted behavior.
 */
@SuppressWarnings("PMD.TooManyMethods")
public class ProfiledPIDController implements Sendable {
	private static int instances;

	private PIDController m_controller;
	private TrapezoidProfile profile;
	private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
	private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
	private TrapezoidProfile.Constraints m_constraints;

	/**
	 * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
	 * Kd.
	 *
	 * @param Kp          The proportional coefficient.
	 * @param Ki          The integral coefficient.
	 * @param Kd          The derivative coefficient.
	 * @param constraints Velocity and acceleration constraints for goal.
	 */
	@SuppressWarnings("ParameterName")
	public ProfiledPIDController(double Kp, double Ki, double Kd,
												TrapezoidProfile.Constraints constraints) {
		m_controller = new PIDController(Kp, Ki, Kd);
		m_constraints = constraints;
		instances++;
	}

	/**
	 * Sets the PID Controller gain parameters.
	 *
	 * <p>Sets the proportional, integral, and differential coefficients.
	 *
	 * @param Kp Proportional coefficient
	 * @param Ki Integral coefficient
	 * @param Kd Differential coefficient
	 */
	@SuppressWarnings("ParameterName")
	public void setPID(double Kp, double Ki, double Kd) {
		m_controller.setPID(Kp, Ki, Kd);
	}

	/**
	 * Sets the proportional coefficient of the PID controller gain.
	 *
	 * @param Kp proportional coefficient
	 */
	@SuppressWarnings("ParameterName")
	public void setP(double Kp) {
		m_controller.setP(Kp);
	}

	/**
	 * Sets the integral coefficient of the PID controller gain.
	 *
	 * @param Ki integral coefficient
	 */
	@SuppressWarnings("ParameterName")
	public void setI(double Ki) {
		m_controller.setI(Ki);
	}

	/**
	 * Sets the differential coefficient of the PID controller gain.
	 *
	 * @param Kd differential coefficient
	 */
	@SuppressWarnings("ParameterName")
	public void setD(double Kd) {
		m_controller.setD(Kd);
	}

	/**
	 * Gets the proportional coefficient.
	 *
	 * @return proportional coefficient
	 */
	public double getP() {
		return m_controller.getP();
	}

	/**
	 * Gets the integral coefficient.
	 *
	 * @return integral coefficient
	 */
	public double getI() {
		return m_controller.getI();
	}

	/**
	 * Gets the differential coefficient.
	 *
	 * @return differential coefficient
	 */
	public double getD() {
		return m_controller.getD();
	}

	/**
	 * Sets the goal for the ProfiledPIDController.
	 *
	 * @param goal The desired goal state.
	 */
	public void setGoal(TrapezoidProfile.State goal) {
		m_goal = goal;
	}

	/**
	 * Sets the goal for the ProfiledPIDController.
	 *
	 * @param goal The desired goal position.
	 */
	public void setGoal(double goal) {
		m_goal = new TrapezoidProfile.State(goal, 0);
	}

	/**
	 * Gets the goal for the ProfiledPIDController.
	 */
	public TrapezoidProfile.State getGoal() {
		return m_goal;
	}

	/**
	 * Returns true if the error is within the tolerance of the error.
	 *
	 * <p>This will return false until at least one input value has been computed.
	 */
	public boolean atGoal() {
		return atSetpoint() && m_goal.equals(m_setpoint);
	}

	/**
	 * Set velocity and acceleration constraints for goal.
	 *
	 * @param constraints Velocity and acceleration constraints for goal.
	 */
	public void setConstraints(TrapezoidProfile.Constraints constraints) {
		m_constraints = constraints;
	}

	/**
	 * Returns the current setpoint of the ProfiledPIDController.
	 *
	 * @return The current setpoint.
	 */
	public TrapezoidProfile.State getSetpoint() {
		return m_setpoint;
	}

	/**
	 * Returns true if the error is within the tolerance of the error.
	 *
	 * <p>This will return false until at least one input value has been computed.
	 */
	public boolean atSetpoint() {
		return m_controller.atSetpoint();
	}

	/**
	 * Enables continuous input.
	 *
	 * <p>Rather then using the max and min input range as constraints, it considers
	 * them to be the same point and automatically calculates the shortest route
	 * to the setpoint.
	 *
	 * @param minimumInput The minimum value expected from the input.
	 * @param maximumInput The maximum value expected from the input.
	 */
	public void enableContinuousInput(double minimumInput, double maximumInput) {
		m_controller.enableContinuousInput(minimumInput, maximumInput);
	}

	/**
	 * Disables continuous input.
	 */
	public void disableContinuousInput() {
		m_controller.disableContinuousInput();
	}

	/**
	 * Sets the minimum and maximum values for the integrator.
	 *
	 * <p>When the cap is reached, the integrator value is added to the controller
	 * output rather than the integrator value times the integral gain.
	 *
	 * @param minimumIntegral The minimum value of the integrator.
	 * @param maximumIntegral The maximum value of the integrator.
	 */
	public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
		m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
	}

	/**
	 * Sets the error which is considered tolerable for use with atSetpoint().
	 *
	 * @param positionTolerance Position error which is tolerable.
	 */
	public void setTolerance(double positionTolerance) {
		setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
	}

	/**
	 * Sets the error which is considered tolerable for use with atSetpoint().
	 *
	 * @param positionTolerance Position error which is tolerable.
	 * @param velocityTolerance Velocity error which is tolerable.
	 */
	public void setTolerance(double positionTolerance, double velocityTolerance) {
		m_controller.setTolerance(positionTolerance, velocityTolerance);
	}

	/**
	 * Returns the difference between the setpoint and the measurement.
	 *
	 * @return The error.
	 */
	public double getPositionError() {
		return m_controller.getPositionError();
	}

	/**
	 * Returns the change in error per second.
	 */
	public double getVelocityError() {
		return m_controller.getVelocityError();
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 */
	public double calculate(double measurement, double secondsSinceLastCalculate) {
		this.profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);	
		m_setpoint = profile.calculate(secondsSinceLastCalculate);
		System.out.println(m_goal.position);
		System.out.println("Profiled Setpoint Velocity" + m_setpoint.velocity);
		System.out.println("Profiled Setpoint Position" + m_setpoint.position);
		return m_controller.calculate(measurement, m_setpoint.position, secondsSinceLastCalculate);
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @param goal The new goal of the controller.
	 */
	public double calculate(double measurement, TrapezoidProfile.State goal, double secondsSinceLastCalculate) {
		setGoal(goal);
		return calculate(measurement, secondsSinceLastCalculate);
	}

	/**
	 * Returns the next output of the PIDController.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @param goal The new goal of the controller.
	 */
	public double calculate(double measurement, double goal, double secondsSinceLastCalculate) {
		setGoal(goal);
		return calculate(measurement, secondsSinceLastCalculate);
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @param goal        The new goal of the controller.
	 * @param constraints Velocity and acceleration constraints for goal.
	 */
	public double calculate(double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints, double timeSinceLastCalculate) {
		setConstraints(constraints);
		return calculate(measurement, goal, timeSinceLastCalculate);
	}

	/**
	 * Reset the previous error and the integral term.
	 *
	 * @param measurement The current measured State of the system.
	 */
	public void reset(TrapezoidProfile.State measurement) {
		m_controller.reset();
		m_setpoint = measurement;
	}

	/**
	 * Reset the previous error and the integral term.
	 *
	 * @param measuredPosition The current measured position of the system.
	 * @param measuredVelocity The current measured velocity of the system.
	 */
	public void reset(double measuredPosition, double measuredVelocity) {
		reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
	}

	/**
	 * Reset the previous error and the integral term.
	 *
	 * @param measuredPosition The current measured position of the system. The velocity is
	 *     assumed to be zero.
	 */
	public void reset(double measuredPosition) {
		reset(measuredPosition, 0.0);
	}
	
	public TrapezoidProfile.State getDesiredState(double timeSinceStart) {
		return profile.calculate(timeSinceStart);
	}
	
	public double getTimeUntilAtPosition(double target) {
		return profile.timeLeftUntil(target);
	}

	
	public double getTimeUntilFinish () {
		return profile.totalTime();
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("ProfiledPIDController");
		builder.addDoubleProperty("p", this::getP, this::setP);
		builder.addDoubleProperty("i", this::getI, this::setI);
		builder.addDoubleProperty("d", this::getD, this::setD);
		builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
	}
}

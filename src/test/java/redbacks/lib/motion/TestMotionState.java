package redbacks.lib.motion;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import redbacks.lib.geometry.Differential2d;

public class TestMotionState {
	//Constructor Tests
	@Test
	public void testEmptyConstructor() {
		Pose2d position = new Pose2d(); 
		Differential2d velocity = new Differential2d(); 
		Differential2d acceleration = new Differential2d(); 
		double timestamp = 0;
		
		MotionState2d motionState = new MotionState2d();
		testConstructor(motionState, position, velocity, acceleration, timestamp);
	}
	
	@Test
	public void testPositionConstructor() {
		Pose2d position = new Pose2d(4, 3, new Rotation2d(1.5)); 
		Differential2d velocity = new Differential2d(); 
		Differential2d acceleration = new Differential2d(); 
		double timestamp = 4.3;
		
		MotionState2d motionState = new MotionState2d(position, timestamp);
		testConstructor(motionState, position, velocity, acceleration, timestamp);
	}
	
	@Test
	public void testVelocityConstructor() {
		double timestamp = -1;
		Pose2d position = new Pose2d(-4, 3, new Rotation2d(-0.4)); 
		Differential2d velocity = new Differential2d(2, 5, -0.2, 4, timestamp); 
		Differential2d acceleration = new Differential2d(); 
		
		
		MotionState2d motionState = new MotionState2d(position, velocity, timestamp);
		testConstructor(motionState, position, velocity, acceleration, timestamp);
	}
	
	@Test
	public void testAccelerationConstructor() {
		double timestamp = -1;
		Pose2d position = new Pose2d(-4, 3, new Rotation2d(-0.4)); 
		Differential2d velocity = new Differential2d(2, 5, -0.2, 4, timestamp); 
		Differential2d acceleration = new Differential2d(1, 0, -0.4, 0, timestamp); 
		
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, timestamp);
		testConstructor(motionState, position, velocity, acceleration, timestamp);
	}
	
	//State Prediction Tests
	@Test
	public void testStationaryStateStaysStationary() {
		Pose2d position = new Pose2d(4, 3, new Rotation2d(1.5)); 
		Differential2d velocity = new Differential2d(); 
		Differential2d acceleration = new Differential2d(); 
		double timestamp = 4.3;
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, timestamp);
		
		assertTrue(motionState.equalsDiscountingTime(motionState.PredictState(5)));
		assertTrue(motionState.equalsDiscountingTime(motionState.PredictState(0)));
		assertTrue(motionState.equalsDiscountingTime(motionState.PredictState(-3)));
	}
	
	@Test
	public void testPredictsConstantVelocity() {
		Pose2d position = new Pose2d(0, 0, new Rotation2d(0)); 
		Differential2d velocity = new Differential2d(3, 2, 0); 
		Differential2d acceleration = new Differential2d(); 
		double timestamp = 0;
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, timestamp);
		MotionState2d predictedMotionState = motionState.PredictState(1);
		
		assertTrue(new MotionState2d(new Pose2d(3, 2, new Rotation2d(0)), velocity, acceleration, 1).equalsDiscountingDifferentialTimeData(predictedMotionState));
	}
	
	@Test
	public void testPredictsConstantAcceleration() {
		Pose2d position = new Pose2d(0, 0, new Rotation2d(0)); 
		Differential2d velocity = new Differential2d(1, 2, 0); 
		Differential2d acceleration = new Differential2d(2, 1, 0); 
		double timestamp = 0;
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, timestamp);
		MotionState2d predictedMotionState = motionState.PredictState(2);
		
		assertTrue(new MotionState2d(new Pose2d(6, 6, new Rotation2d(0)), new Differential2d(5, 4, 0), acceleration, 2).equalsDiscountingDifferentialTimeData(predictedMotionState));
	}
	
	@Test
	public void testPredictsRotation() {
		Pose2d position = new Pose2d(0, 0, new Rotation2d(0)); 
		Differential2d velocity = new Differential2d(0, 0, Math.PI / 2); 
		Differential2d acceleration = new Differential2d(0, 0, 0); 
		double timestamp = 0;
		MotionState2d motionState = new MotionState2d(position, velocity, acceleration, timestamp);
		MotionState2d predictedMotionState = motionState.PredictState(2);
		
		assertTrue(new MotionState2d(new Pose2d(0, 0, new Rotation2d(Math.PI)), velocity, acceleration, 2).equalsDiscountingDifferentialTimeData(predictedMotionState));
	}
	
	public void testConstructor(MotionState2d motionState, Pose2d position, Differential2d velocity, Differential2d acceleration, double timestamp) {
		MotionState2d expectedMotionState = new MotionState2d(position, velocity, acceleration, timestamp);
		assertTrue(expectedMotionState.equals(motionState));
	}
}

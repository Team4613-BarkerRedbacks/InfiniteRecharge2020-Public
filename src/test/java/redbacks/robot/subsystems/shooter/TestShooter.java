package redbacks.robot.subsystems.shooter;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import arachne.lib.pipeline.DoublePipe;

import static org.junit.jupiter.api.Assertions.*;

public class TestShooter {
	Shooter shooter;
	DoublePipe speed;

	private static final double
		MIN_SPEED = 13000,
		TARGET_SPEED_TOLERANCE = 1000, // TODO Determine acceptable tolerance
		TARGET_SPEED = 17620;
	
	@BeforeEach
	public void beforeEach() {
		shooter = new Shooter();
		shooter.initialize();

		speed = new DoublePipe(0);
		shooter.getSpeedSensor().populate(speed);
	}
	
	@AfterEach
	public void afterEach() {
		shooter = null;
	}
	
	@Test
	public void aboveMinimumSpeedTest() {
		speed.accept(0);
		assertFalse(shooter.isAboveMinimumSpeed());
		
		speed.accept(MIN_SPEED);
		assertTrue(shooter.isAboveMinimumSpeed());
		
		speed.accept(-MIN_SPEED);
		assertFalse(shooter.isAboveMinimumSpeed());
		
		speed.accept(MIN_SPEED + 1000);
		assertTrue(shooter.isAboveMinimumSpeed());
	}
	
	@Test
	public void atTargetSpeedTest() {
		speed.accept(0);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(MIN_SPEED);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(-MIN_SPEED);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(MIN_SPEED+1000);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(TARGET_SPEED);
		assertTrue(shooter.isAtTargetSpeed());
		
		speed.accept(0);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(TARGET_SPEED + TARGET_SPEED_TOLERANCE);
		assertTrue(shooter.isAtTargetSpeed());
		
		speed.accept(0);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(TARGET_SPEED - TARGET_SPEED_TOLERANCE);
		assertTrue(shooter.isAtTargetSpeed());
		
		speed.accept(0);
		assertFalse(shooter.isAtTargetSpeed());
		

		speed.accept(TARGET_SPEED + TARGET_SPEED_TOLERANCE + 100);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(0);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(TARGET_SPEED - TARGET_SPEED_TOLERANCE - 100);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(0);
		assertFalse(shooter.isAtTargetSpeed());
		
		speed.accept(TARGET_SPEED + TARGET_SPEED_TOLERANCE - TARGET_SPEED_TOLERANCE / 2);
		assertTrue(shooter.isAtTargetSpeed());
		
		speed.accept(0);
		assertFalse(shooter.isAtTargetSpeed());
	}
	
}

package redbacks.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Twist2d;

public class TestDifferential2d {
	Differential2d differential2d;
	
	@AfterEach
	public void AfterEach() {
		differential2d = null;
	}
	
	@Test
	public void testEmptyConstructor() {
		differential2d = new Differential2d();
		
		assertEquals(differential2d.dx, 0);
		assertEquals(differential2d.dy, 0);
		assertEquals(differential2d.dtheta, 0);
		assertEquals(differential2d.period, 0);
		assertEquals(differential2d.timestamp, 0);
		assertTrue(differential2d.equals(new Differential2d()));
	}
	
	@Test
	public void testTwist2dConstructor() {
		double dx = 1, dy = 1, dtheta = 1;
		differential2d = new Differential2d(dx, dy, dtheta);
		
		assertEquals(differential2d.dx, dx);
		assertEquals(differential2d.dy, dy);
		assertEquals(differential2d.dtheta, dtheta);
		assertEquals(differential2d.period, 0);
		assertEquals(differential2d.timestamp, 0);
		assertTrue(differential2d.getAsTwist().equals(new Twist2d(dx, dy, dtheta)));
		assertTrue(differential2d.equals(new Differential2d(dx, dy, dtheta)));
	}
	
	@Test
	public void testNonTimestampConstructor() {
		double dx = 1, dy = 1, dtheta = 1, period = 1;
		differential2d = new Differential2d(dx, dy, dtheta, period);
		
		assertEquals(differential2d.dx, dx);
		assertEquals(differential2d.dy, dy);
		assertEquals(differential2d.dtheta, dtheta);
		assertEquals(differential2d.period, period);
		assertEquals(differential2d.timestamp, 0);
		assertTrue((differential2d.getAsTwist().equals(new Twist2d(dx, dy, dtheta))));
		assertTrue(differential2d.equals(new Differential2d(dx, dy, dtheta, period)));
		assertFalse(differential2d.equals(new Differential2d(dx, dy, dtheta, period+0.1)));
	}
	
	@Test
	public void testTimeEquality() {
		double dx = 1, dy = 1, dtheta = 1, period = 1, timestamp = 2;
		differential2d = new Differential2d(dx, dy, dtheta, period, timestamp);
		Differential2d otherDifferential2d = new Differential2d(dx, dy, dtheta, period+5, timestamp-23);
		assertTrue(differential2d.equalsDiscountingTime(otherDifferential2d));
		assertFalse(differential2d.equals(otherDifferential2d));
	}
	
	@Test
	public void testTimestampConstructor() {
		double dx = 1, dy = 1, dtheta = 1, period = 1, timestamp = 2;
		differential2d = new Differential2d(dx, dy, dtheta, period, timestamp);
		
		assertEquals(differential2d.dx, dx);
		assertEquals(differential2d.dy, dy);
		assertEquals(differential2d.dtheta, dtheta);
		assertEquals(differential2d.period, period);
		assertEquals(differential2d.timestamp, timestamp);
		assertTrue((differential2d.getAsTwist().equals(new Twist2d(dx, dy, dtheta))));
		assertTrue(differential2d.equals(new Differential2d(dx, dy, dtheta, period, timestamp)));
		assertFalse(differential2d.equals(new Differential2d(dx, dy, dtheta, timestamp-0.1)));
	}
	
	@Test
	public void testScaleBy() {
		double dx = 0.5, dy = 1, dtheta = 1, period = 0.5, timestamp = 2, scalar = 2;
		differential2d = new Differential2d(dx, dy, dtheta, period, timestamp);
		
		assertTrue(differential2d.equals(new Differential2d(dx, dy, dtheta, period, timestamp)));
		differential2d = differential2d.scaleBy(scalar);
		
		assertEquals(differential2d.dx, dx * scalar);
		assertEquals(differential2d.dy, dy * scalar);
		assertEquals(differential2d.dtheta, dtheta * scalar);
		assertEquals(differential2d.period, period * scalar);
		assertEquals(differential2d.timestamp, timestamp);
	}
	
	@Test
	public void testDivideBy() {
		double dx = 0.5, dy = 1, dtheta = 1, period = 0.5, timestamp = 2, divisor = 2;
		differential2d = new Differential2d(dx, dy, dtheta, period, timestamp);
		
		assertTrue(differential2d.equals(new Differential2d(dx, dy, dtheta, period, timestamp)));
		
		differential2d = differential2d.divideBy(divisor);
		
		assertEquals(differential2d.dx, dx / divisor);
		assertEquals(differential2d.dy, dy / divisor);
		assertEquals(differential2d.dtheta, dtheta / divisor);
		assertEquals(differential2d.period, period / divisor);
		assertEquals(differential2d.timestamp, timestamp);
	}
}

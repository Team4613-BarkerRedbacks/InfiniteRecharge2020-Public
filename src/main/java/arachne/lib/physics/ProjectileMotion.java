package arachne.lib.physics;

import arachne.lib.immutables.Pair;
import arachne.lib.maths.ImmutableVector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class ProjectileMotion
{
	public static void main(String[] args) {
//		long start = System.currentTimeMillis();
		
//		for(int i = 0; i < 1000; i++) {
			getMovingRotationAndLaunchAngles(
					new ImmutableVector2d(3, 1.55),
					11.25,
//					Math.random() * 4 - 2,
//					-Math.random() * 2
					-15,
					-1.07
			);
//		}
		
//		System.out.println(System.currentTimeMillis() - start);
	}
	
	private static final double g = -9.8;
	
	public static Rotation2d getStationaryLaunchAngle(ImmutableVector2d vectorToGoal, double launchVelocity) {
		// TODO Assertions that values are within domain
		final double
			b = vectorToGoal.getX(),
			a = g * b * b / (2 * launchVelocity * launchVelocity),
			c = a - vectorToGoal.getY();
		
		// Apply inverse tan to quadratic formula
		// Numerator and denominator are multiplied by -1 to get result in correct quadrant
		return new Rotation2d(Math.atan2(
			b - Math.sqrt(b * b - 4 * a * c),
			-2 * a
		));
	}
	
	public static Pair<Rotation2d, Rotation2d> getMovingRotationAndLaunchAngles(
			ImmutableVector2d vectorToGoal, double launchVelocity,
			double inlineVelocity, double perpendicularVelocity
	) {
		// TODO Cases for either = 0
		if(Math.abs(inlineVelocity) < 1e-5) inlineVelocity = 1e-5;
		if(Math.abs(perpendicularVelocity) < 1e-5) perpendicularVelocity = 1e-5;
		
		// TODO So many domain checks
		final double
			h = vectorToGoal.getX(),
			v = vectorToGoal.getY(),
			p = perpendicularVelocity,
			t = inlineVelocity,
			u = launchVelocity;
		
		final double
			h2 = h * h,
			v2 = v * v,
			p2 = p * p,
			t2 = t * t;
			
		final double
			u2h2 = u * u * h2,
			gh2 = g * h2,
			gh2v = gh2 * v,
			h2p2 = h2 * p2,
			t2v2 = t2 * v2;
		
		final double
			a = p2*p2 * (h2 + v2),
			b = -2*p*p2*t * (h2 + 2*v2),
			c = p2 * (-u2h2 - gh2v + h2p2 + h2*t2 + 6*t2v2),
			d = 2*p*t * (u2h2 + gh2v - h2p2 - 2*t2v2),
			e = -u2h2*t2 + gh2*gh2/4 - gh2*t2*v + h2p2*t2 + t2*t2v2;
		
		final double
			eight_a2 = 8 * a * a,
			b2 = b * b,
			c2 = c * c,
			ac = a * c,
			bd = b * d;
		
		final double
			P = (8*ac - 3*b2) / eight_a2,
			Q = (b*b2 - 4*ac*b + eight_a2*d) / (eight_a2*a), // Note: This represents q, not Q
			disc0 = c2 - 3*bd + 12*a*e,
			disc1 = 2*c*c2 - 9*bd*c + 27*b2*e + 27*a*d*d - 72*ac*e,
			S = Math.sqrt(
				(-P + Math.sqrt(disc0) * Math.cos(Math.acos(
						disc1 / (2 * Math.sqrt(disc0 * disc0 * disc0))
				) / 3) / a) / 6
			);

		Rotation2d rotationAngle;
		
		if(perpendicularVelocity > 0) {
			rotationAngle = new Rotation2d(Math.atan2(
					1,
					-b/(4*a) - S - Math.sqrt(-4*S*S - 2*P + Q/S) / 2
			) - Math.PI);
		}
		else {
			rotationAngle = new Rotation2d(Math.atan2(
					1,
					-b/(4*a) + S + Math.sqrt(-4*S*S - 2*P - Q/S) / 2
			));
		}
		
		if(Double.isNaN(rotationAngle.getRadians())) return null;
		
		Rotation2d launchAngle = new Rotation2d(Math.acos(
				-perpendicularVelocity
				/ (launchVelocity * rotationAngle.getSin())
		));
		
		// TODO Remove below
//		System.out.println("Target: " + rotationAngle + ", " + launchAngle + "\n");
//		
//		System.out.println(
//				"a: " + a
//				+ ", b: " + b
//				+ ", P: " + P
//				+ ", Q: " + Q
//				+ ", S: " + S
//		);
//		
//		for(int i = 0; i < 4; i++) {
//			double ra = Math.atan2(
//					1,
//					(-b/(4*a)
//					+ (i < 2 ? -1 : 1) * S
//					+ (i % 2 == 0 ? 1 : -1) * Math.sqrt(-4*S*S - 2*P + (i < 2 ? 1 : -1) * Q/S) / 2)
//			) - (i < 2 ? Math.PI : 0),
//			la = Math.acos(
//					-perpendicularVelocity
//					/ (launchVelocity * Math.sin(ra))
//			);
//			
//			System.out.println(
//					"r_t: " + inlineVelocity
//					+ ", r_p: " + perpendicularVelocity
//					+ ", distance: " + vectorToGoal
//					+ ", ra: " + Math.toDegrees(ra)
//					+ ", la: " + Math.toDegrees(la)
//			);
//			
//			System.out.println("Height at impact: " + 
//					(u * Math.sin(la) * h / (u * Math.cos(ra) * Math.cos(la) + t)
//					+ g/2 * h2/Math.pow((u * Math.cos(ra) * Math.cos(la) + t), 2))
//			);
//		}
		
		return new Pair<Rotation2d, Rotation2d>(rotationAngle, launchAngle);
		
//		if(perpendicularVelocity == 0) {
//			if(inlineVelocity == 0) {
//				return new Pair<Rotation2d, Rotation2d>(new Rotation2d(0), getStationaryLaunchAngle(vectorToGoal, launchVelocity));
//			}
//			else {
//				// FIXME Implement
//				return null;
//			}
//		}
//		else if(inlineVelocity == 0) {
//			// TODO Assertions that values are within domain
//			final double
//				h2 = vectorToGoal.getX() * vectorToGoal.getX(),
//				v2h2 = vectorToGoal.getY() * vectorToGoal.getY() + h2,
//				pvgu = -perpendicularVelocity * perpendicularVelocity + vectorToGoal.getY() * g + launchVelocity * launchVelocity;
//			
//			// FIXME Correct signs
//			Rotation2d rotationAngle = new Rotation2d(
//				Math.sqrt(2 * v2h2 * perpendicularVelocity * perpendicularVelocity),
//				Math.sqrt(h2 * (pvgu +
//					Math.sqrt(pvgu * pvgu) - v2h2 * g2
//				))
//			);
//			
//			Rotation2d launchAngle = new Rotation2d(Math.acos(
//					-perpendicularVelocity
//					/ (launchVelocity * rotationAngle.getSin())
//			));
//			
//			return new Pair<Rotation2d, Rotation2d>(rotationAngle, launchAngle);
//		}
	}
}

package arachne.lib.maths;

// TODO Merge into Arachne
public class ArachneMath2
{
	public static double[] solveQuartic(double a, double b, double c, double d, double e) {
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
		
		return new double[] {
			// TODO Implement
		};
	}
}

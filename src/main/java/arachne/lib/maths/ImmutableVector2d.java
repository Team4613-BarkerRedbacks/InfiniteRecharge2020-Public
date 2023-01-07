package arachne.lib.maths;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class ImmutableVector2d
{
	private final double x, y;
	
	public ImmutableVector2d(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public double getLength() {
		return Math.sqrt(x * x + y * y);
	}
	
	public double atan() {
		return Math.atan2(y, x);
	}
	
	public ImmutableVector2d unit() {
		return scale(1 / getLength());
	}
	
	public ImmutableVector2d withX(double x) {
		return new ImmutableVector2d(x, this.y);
	}
	
	public ImmutableVector2d withY(double y) {
		return new ImmutableVector2d(this.x, y);
	}
	
	public ImmutableVector2d plus(ImmutableVector2d vector) {
		return new ImmutableVector2d(x + vector.x, y + vector.y);
	}
	
	public ImmutableVector2d minus(ImmutableVector2d vector) {
		return new ImmutableVector2d(x - vector.x, y - vector.y);
	}
	
	public ImmutableVector2d scale(double scalar) {
		return new ImmutableVector2d(x * scalar, y * scalar);
	}
	
	public double dot(ImmutableVector2d vector) {
		return x * vector.x + y * vector.y;
	}
	
	public ImmutableVector2d rotate(Rotation2d angle) {
		double cos = Math.cos(angle.getRadians());
	    double sin = Math.sin(angle.getRadians());
	    
	    return new ImmutableVector2d(x * cos - y * sin, x * sin + y * cos);
	}
	
	public double scalarProjectOnto(ImmutableVector2d direction) {
		return this.dot(direction) / direction.getLength();
	}
	
	public ImmutableVector2d vectorProjectOnto(ImmutableVector2d direction) {
		ImmutableVector2d unit = direction.unit();
		
		return unit.scale(this.dot(unit));
	}
	
	@Override
	public String toString() {
		return "(" + x + ", " + y + ")";
	}
}

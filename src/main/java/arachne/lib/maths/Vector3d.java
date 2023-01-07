package arachne.lib.maths;

import arachne.lib.logging.ArachneLogger;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Vector3d
{
	// TODO
	// projection onto plane
	// rotate around axis
	// angle to plane
	
	private final double x, y, z;
	
	public static enum Axis {
		X {
			@Override
			public double getComponent(Vector3d vector) {
				return vector.x;
			}
		},
		Y {
			@Override
			public double getComponent(Vector3d vector) {
				return vector.y;
			}
		},
		Z {
			@Override
			public double getComponent(Vector3d vector) {
				return vector.z;
			}
		};
		
		public abstract double getComponent(Vector3d vector);
	}
	
	public Vector3d(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public double getZ() {
		return z;
	}
	
	public double getLength() {
		return Math.sqrt(x * x + y * y + z * z);
	}
	
	public Vector3d withX(double x) {
		return new Vector3d(x, this.y, this.z);
	}
	
	public Vector3d withY(double y) {
		return new Vector3d(this.x, y, this.z);
	}
	
	public Vector3d withZ(double z) {
		return new Vector3d(this.x, this.y, z);
	}
	
	public Vector3d plus(Vector3d vector) {
		return new Vector3d(x + vector.x, y + vector.y, z + vector.z);
	}
	
	public Vector3d minus(Vector3d vector) {
		return new Vector3d(x - vector.x, y - vector.y, z - vector.z);
	}
	
	public Vector3d scale(double scalar) {
		return new Vector3d(x * scalar, y * scalar, z * scalar);
	}
	
	public double dot(Vector3d vector) {
		return x * vector.x + y * vector.y + z * vector.z;
	}
	
	public Vector3d cross(Vector3d vector) {
		return new Vector3d(
			y * vector.z - z * vector.y,
			z * vector.x - x * vector.z,
			x * vector.y - y * vector.x
		);
	}
	
	public Vector3d rotate(Axis fromAxis, Axis toAxis, Rotation2d rotation) {
		if(fromAxis == toAxis) {
			ArachneLogger.getInstance().error("Vector3d::rotate axis references should not be equal, returning vector");
			return this;
		}
		
		Translation2d translation = new Translation2d(fromAxis.getComponent(this), toAxis.getComponent(this)).rotateBy(rotation);
		
		double x = this.x, y = this.y, z = this.z;
		
		switch(fromAxis) {
			case X:
				x = translation.getX();
			case Y:
				y = translation.getX();
			case Z:
				z = translation.getX();
		}
		
		switch(toAxis) {
			case X:
				x = translation.getY();
			case Y:
				y = translation.getY();
			case Z:
				z = translation.getY();
		}
		
		return new Vector3d(x, y, z);
	}
	
	@Override
	public String toString() {
		return "(" + x + ", " + y + ", " + z + ")";
	}
	
	public static Vector3d sum(Vector3d... vectors) {
		double x = 0, y = 0, z = 0;
		
		for(Vector3d vector : vectors) {
			x += vector.x;
			y += vector.y;
			z += vector.z;
		}
		
		return new Vector3d(x, y, z);
	}
	
	public static Vector3d avg(Vector3d... vectors) {
		double x = 0, y = 0, z = 0;
		double n = vectors.length;
		
		for(Vector3d vector : vectors) {
			x += vector.x;
			y += vector.y;
			z += vector.z;
		}
		
		return new Vector3d(x / n, y / n, z / n);
	}
}

package arachne.lib.types;

import static arachne.lib.types.Directions2D.Horizontal.*;
import static arachne.lib.types.Directions2D.Vertical.*;

public class Directions2D
{
	private final Vertical vertical;
	private final Horizontal horizontal;
	
	public Directions2D() {
		this(Vertical.NONE, Horizontal.NONE);
	}
	
	public Directions2D(Vertical vertical) {
		this(vertical, Horizontal.NONE);
	}
	
	public Directions2D(Horizontal horizontal) {
		this(Vertical.NONE, horizontal);
	}
	
	public Directions2D(Vertical vertical, Horizontal horizontal) {
		this.vertical = vertical;
		this.horizontal = horizontal;
	}
	
	public Vertical getVertical() {
		return vertical;
	}
	
	public Horizontal getHorizontal() {
		return horizontal;
	}
	
	public boolean isStationary() {
		return vertical == Vertical.NONE && horizontal == Horizontal.NONE;
	}
	
	@Override
	public boolean equals(Object obj) {
		if(obj == null || obj.getClass() != this.getClass()) return false;
		
		Directions2D other = (Directions2D) obj;
		
		return this.vertical == other.vertical && this.horizontal == other.horizontal;
	}
	
	public enum Vertical {
		UP,
		DOWN,
		NONE;
	}
	
	public enum Horizontal {
		LEFT,
		RIGHT,
		NONE;
	}
	
	public static Directions2D fromAngle(int angle) {
		switch(angle) {
			case 0:		return new Directions2D(UP);
			case 45:	return new Directions2D(UP, RIGHT);
			case 90:	return new Directions2D(RIGHT);
			case 135:	return new Directions2D(DOWN, RIGHT);
			case 180:	return new Directions2D(DOWN);
			case 225:	return new Directions2D(DOWN, LEFT);
			case 270:	return new Directions2D(LEFT);
			case 315:	return new Directions2D(UP, LEFT);
		}
		
		return new Directions2D();
	}
}

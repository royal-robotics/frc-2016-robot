package org.usfirst.frc.team2522.robot;

public class LEDColor {
	private int red;
	private int green;
	private int blue;
	
	public LEDColor() {
		this(0, 0, 0);
	}
	
	public LEDColor(int r, int g, int b) {
		this.red = r;
		this.green = g;
		this.blue = b;
	}
	
	public int getRed() {
		return red;
	}
	
	public int getGreen() {
		return green;
	}
	
	public int getBlue() {
		return blue;
	}
}

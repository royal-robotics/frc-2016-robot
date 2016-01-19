package org.usfirst.frc.team2522.robot;

import java.lang.*;

import edu.wpi.first.wpilibj.*;

import org.usfirst.frc.team2522.robot.*;


//This is for the LEDController 2.0 (16 million colors)
public class LEDController {
	
	private LEDRegister register;
	private DigitalOutput clock;
	private DigitalOutput dataOut;
	
	public LEDController() {
		register = new LEDRegister(Byte.MIN_VALUE, Byte.MIN_VALUE, Byte.MIN_VALUE);
	}
	
	public void writeRegister() {
		clock.set(false);
		Byte red = Byte.MAX_VALUE;
		writeByte(red);
	}
	
	public void writeByte(Byte data) {
		(new LEDRegOutThread()).start();
	}
}
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
		writeByte();
	}
	
	public void writeByte() {
		LEDRegOutThread regThread = new LEDRegOutThread(clock, dataOut, register.getRed(), register.getGreen(), register.getBlue());
		regThread.start();
	}
}
package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.*;

import org.usfirst.frc.team2522.robot.*;


//This is for the LEDController 2.0 (16 million colors)
public class LEDControllerV2 {
	public LEDColor ledColor;
	private DigitalOutput clock;
	private DigitalOutput dataOut;
	
	public LEDControllerV2(DigitalOutput clockPort, DigitalOutput dataPort) {
		this.clock = clockPort;
		this.dataOut = dataPort;
		ledColor = new LEDColor(0, 0, 0);
	}
	
	public void writeColor() {
		LEDRegOutThread regThread = new LEDRegOutThread(clock, dataOut, ledColor);
		regThread.start();
	}
}
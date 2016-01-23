package org.usfirst.frc.team2522.robot;

import java.lang.*;
import java.awt.*;

import edu.wpi.first.wpilibj.*;

import org.usfirst.frc.team2522.robot.*;


//This is for the LEDController 2.0 (16 million colors)
public class LEDController {
	public Color ledColor;
	private DigitalOutput clock;
	private DigitalOutput dataOut;
	
	public LEDController() {
		ledColor = new Color(0, 0, 0);
	}
	
	public void writeColor() {
		LEDRegOutThread regThread = new LEDRegOutThread(clock, dataOut, ledColor);
		regThread.start();
	}
}
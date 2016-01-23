package org.usfirst.frc.team2522.robot;


import edu.wpi.first.wpilibj.*;

import java.lang.*;
import java.awt.*;

public class LEDRegOutThread extends Thread {
	
	private Color ledColor;
	private DigitalOutput clock;
	private DigitalOutput dataOut;
	
	public LEDRegOutThread(DigitalOutput clock, DigitalOutput dataOut, Color ledColor) {
		this.clock = clock;
		this.dataOut = dataOut;
		this.ledColor = ledColor;
	}
	
	public void run() {
		writeColor(clock, dataOut, ledColor.getRed());
		writeColor(clock, dataOut, ledColor.getGreen());
		writeColor(clock, dataOut, ledColor.getBlue());
	}
	
	private void writeColor(DigitalOutput clock, DigitalOutput dataOut, int data) {
		for(int i = 0; i < 8; i++) {
			clock.set(false);
			sleep(0, 100);
			
			dataOut.set((data & (0b00000001 << i)) > 0);
			
			clock.set(true);
			sleep(0, 100);
		}
	}
	
	private void sleep(int mili, int nano) {
		try {
			Thread.currentThread();
			Thread.sleep(mili, nano);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}

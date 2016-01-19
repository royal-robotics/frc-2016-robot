package org.usfirst.frc.team2522.robot;


import edu.wpi.first.wpilibj.*;

import java.lang.*;

public class LEDRegOutThread extends Thread {
	
	private Byte rData;
	private Byte gData;
	private Byte bData;
	private DigitalOutput clock;
	private DigitalOutput dataOut;
	
	public LEDRegOutThread(DigitalOutput clock, DigitalOutput dataOut, Byte rData, Byte gData, Byte bData) {
		this.clock = clock;
		this.dataOut = dataOut;
		this.rData = rData;
		this.gData = gData;
		this.bData = bData;
	}
	
	public void run() {
		writeColor(clock, dataOut, rData);
		writeColor(clock, dataOut, gData);
		writeColor(clock, dataOut, bData);
	}
	
	private void writeColor(DigitalOutput clock, DigitalOutput dataOut, Byte data) {
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

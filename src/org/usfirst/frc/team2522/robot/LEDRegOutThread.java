package org.usfirst.frc.team2522.robot;


import edu.wpi.first.wpilibj.*;

public class LEDRegOutThread extends Thread {
	
	private LEDColor ledColor;
	private DigitalOutput clock;
	private DigitalOutput dataOut;
	
	public LEDRegOutThread(DigitalOutput clock, DigitalOutput dataOut, LEDColor ledColor) {
		this.clock = clock;
		this.dataOut = dataOut;
		this.ledColor = ledColor;
	}
	
	public void run() {
		writeColor(clock, dataOut, ledColor.getBlue());
		writeColor(clock, dataOut, ledColor.getGreen());
		writeColor(clock, dataOut, ledColor.getRed());
	}
	
	private void writeColor(DigitalOutput clock, DigitalOutput dataOut, int data) {
		for(int i = 0; i < 8; i++) {
			clock.set(false);
			sleep(0, 10);
			
			dataOut.set((data & (0b00000001 << i)) > 0);
			
			clock.set(true);
			sleep(0, 10);
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

package org.usfirst.frc.team2522.robot;

import java.util.*;
import java.lang.*;

public class BitFiddlingTest {

	public static void main(String[] args) {
		//test1();
		//test2();
		test3();
	}
	
	//Blocking method with sleeps (BAD)
	public static void test1() {	
		Byte red = (byte) 0b11000011;
		System.out.println(red);
		for(int i = 0; i < 8; i++) {
			if(((byte)red & (0b00000001 << i)) > 0) {
				System.out.print(1);
				System.out.print(" ");
			} else {

				System.out.print(0);
				System.out.print(" ");
			}
			System.out.println("");	
		}
	}
	
	//Non-blocking TimerTask Method (problem: can't be less than milisecond)
	public static void test2() {
		Timer timer = new Timer();
		timer.schedule(new TimerTask()
		{
		    public void run()
		    {
		            System.out.println("ASYNC PRINT TEST!");
		    }
		}, 1 );
		
		System.out.println("THIS SHOULD PRINT FIRST");
	}
	
	//Non-blocking new Thread sleep method (best)
	public static void test3() {
		Thread ThreadTest = (new Thread() {
			public void run() {
				Byte red = (byte) 0b11000011;
				for(int i = 0; i < 8; i++) {
					if(((byte)red & (0b00000001 << i)) > 0) {
						System.out.println(1);
					} else {
						System.out.println(0);
					}
					try {
						Thread.currentThread();
						Thread.sleep(300);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				System.out.println("End of Thread!");
			}
		});
		ThreadTest.start();
		System.out.println("After");
	}
}

package org.usfirst.frc.team2522.robot;

public final class AutonomousController
{
	static String autoModeString = "LowBarAndShoot";
	static int autoMode = 0;
	static int autoStep = 0;
	
	
	/***
	 * Return whatever automode is set from the river station during setup.
	 * 
	 * @return
	 */
	public static int getAutoMode()
	{
		autoMode = 0;  // should read this from the control system.
		autoModeString = "LowBarAndShoot";
		
		return autoMode;
	}
	
	public static void RunAutonomous(Robot robot)
	{
		switch(autoMode)
		{
			case 0:
				LowBarAndShootAuto(robot);
				break;
			default:
				driveStop(robot);
				break;
		}
	}
	
	public static void LowBarAndShootAuto(Robot robot)
	{
		switch(autoStep)
		{
			case 0:
				if (armMove(robot, -15.0))
				{
					autoStep++;
				}
				break;
			case 1:
				if (driveForward(robot, 0.0, 60.0))
				{
					autoStep++;
				}
				break;
			case 2:
				if (drivePivot(robot, 45.0))
				{
					autoStep++;
				}
				break;
			case 3:
				if (armMove(robot, 65.0))
				{
					autoStep++;
				}
				break;
			case 4:
				if (shootBall(robot))
				{
					autoStep++;
				}
				break;
			case 5:
				if (armMove(robot, ArmController.HOME_ANGLE))
				{
					autoStep++;
				}
				break;
			default:
				driveStop(robot);
				break;
		}
	}
	
	
	public static void driveResetEncoders(Robot robot)
	{
		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();
	}
	
	public static void driveResetBearing(Robot robot)
	{
		robot.mxp.reset();
	}
	
	public static void driveStop(Robot robot)
	{
		robot.myDrive.tankDrive(0.0, 0.0);
	}

	/***
	 * This method will continue to set the motors 
	 * @param robot
	 * @param bearing
	 * @param distance
	 * @return
	 */
	public static boolean driveForward(Robot robot, double bearing, double distance)
	{
		if (robot.leftDriveEncoder.getDistance() >= distance)
		{
			driveStop(robot);
			return true;
		}
		else
		{
			double speed = 0.35;
			double error = bearing - robot.mxp.getAngle();
			robot.myDrive.tankDrive(speed + (0.10 * error / 10.0), speed - (0.10 * error / 10.0));
			return false;
		}
	}
	
	/***
	 * Pivot until bearing is matched, the gyro should be reset before calling this and the bearing provided should 
	 * be a value from -180 to +180
	 * 
	 * @param robot	The robot being pivoted
	 * @param bearing	The Gyro setting we are pivoting too.
	 * @return True if we have reached the desired heading, otherwise false.
	 */
	public static boolean drivePivot(Robot robot, double bearing)
	{
		double error = bearing - robot.mxp.getAngle();
		
		if (error < 0.5 && error > -0.5)
		{
			driveStop(robot);
			return true;
		}
		else
		{
			double pivotSpeed = 0.30;
			pivotSpeed += 0.4 * (Math.abs(error) / 90.0);
			if (error < 0.0)
			{
				robot.myDrive.tankDrive(+pivotSpeed, -pivotSpeed);
			}
			else
			{
				robot.myDrive.tankDrive(-pivotSpeed, +pivotSpeed);
			}
			return false;
		}
	}
	
	/***
	 * 
	 * @param robot
	 * @param targetAngle
	 */
	public static boolean armMove(Robot robot, double targetAngle)
	{
		if (robot.armController.getTargetAngle() != targetAngle)
		{
			robot.armController.setTargetAngle(targetAngle);
		}
		
		return robot.armController.onTarget();
	}
	
	/***
	 * 
	 * @param robot
	 * @return
	 */
	public static boolean shootBall(Robot robot)
	{
		// TODO: implement spinning up motors and shooting.
		return true;
	}
}

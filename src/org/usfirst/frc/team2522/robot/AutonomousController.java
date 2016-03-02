package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

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
	public static int getAutoMode(Robot robot)
	{
		if (robot.leftstick.getZ() > 0.5)
		{
			autoMode = 0;
			autoModeString = "DoNothing";
		}
		else if  ((robot.leftstick.getZ() <= 0.5) && (robot.leftstick.getZ() > 0.0))
		{
			autoMode = 1;
			autoModeString = "DriveForward";
		}
		else if  ((robot.leftstick.getZ() <= 0.0) && (robot.leftstick.getZ() > -0.5))
		{
			autoMode = 2;
			autoModeString = "LowBarNoShot";
		}
		else
		{
			autoMode = 3;
			autoModeString = "LowBarAndShoot";
		}

		return autoMode;
	}
	
	public static void RunAutonomous(Robot robot)
	{
		switch(autoMode)
		{
			case 1:
				DriveForwardAuto(robot);
				break;
			case 2:
				LowBarNoShotAuto(robot);
				break;
			case 3:
				LowBarAndShootAuto(robot);
				break;
			default:
				driveStop(robot);
				break;
		}
	}
	
	/***
	 * 
	 * @param robot
	 */
	public static void DriveForwardAuto(Robot robot)
	{
		switch(autoStep)
		{
			case 0:
				if (driveForward(robot, 0.0, 0.80, 185.0))
				{
					autoStep++;
				}
				break;
			default:
				driveStop(robot);
				break;			
		}
	}
	
	/***
	 * 
	 * @param robot
	 */
	public static void LowBarNoShotAuto(Robot robot)
	{
		switch(autoStep)
		{
			case 0:
			{
				if (robot.leftDriveEncoder.getDistance() < 96.0)
				{
					armMove(robot, -15.0);
				}
				else
				{
					armMove(robot, ArmController.HOME_ANGLE);
				}

				if (driveForward(robot, 0.0, 0.75, 245.0))
				{
					autoStep++;
				}
			
				break;
			}
			case 1:
				if (drivePivot(robot, 65.0, 0.65))
				{
					autoStep++;
				}
				break;
			default:
				driveStop(robot);
				break;
		}
	}
	
	/***
	 * 
	 * @param robot
	 */
	public static void LowBarAndShootAuto(Robot robot)
	{
		switch(autoStep)
		{
		case 0:
		{
			if (robot.leftDriveEncoder.getDistance() < 96.0)
			{
				armMove(robot, -15.0);
			}
			else
			{
				armMove(robot, 53.0);
			}

			if (driveForward(robot, 0.0, 0.75, 235.0))
			{
				autoStep++;
			}
		
			break;
		}
		case 1:
			if (drivePivot(robot, 65.0, 0.65))
			{
				autoStep++;
			}
			break;
		case 2:
			if (shootBall(robot, 1950.0))
			{
				autoStep++;
			}
			break;
//		case 3:
//			if (armMove(robot, ArmController.HOME_ANGLE))
//			{
//				autoStep++;
//			}
//			break;
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
	public static boolean driveForward(Robot robot, double bearing, double speed, double distance)
	{
		if (robot.leftDriveEncoder.getDistance() >= distance)
		{
			driveStop(robot);
			return true;
		}
		else
		{
			double error = bearing - robot.mxp.getAngle();
			if (error > 180.0) {
				error -= 360.0;
			} else if (error < -180.0) {
				error += 360.0;
			}
			
			robot.myDrive.tankDrive(-speed - (0.15 * error / 10.0), -speed + (0.15 * error / 10.0));
			
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
	public static boolean drivePivot(Robot robot, double bearing, double power)
	{
		double error = bearing - robot.mxp.getAngle();
		if (error > 180.0) {
			error -= 360.0;
		} else if (error < -180.0) {
			error += 360.0;
		}
		
		if (error < 1 && error > -1.0)
		{
			driveStop(robot);
			return true;
		}
		else
		{
			double pivotSpeed = power;
			//pivotSpeed += 0.4 * (Math.abs(error) / 90.0);
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
		
		//return robot.armController.onTarget();
		return Math.abs(targetAngle - robot.armController.getAngle()) < 3.0;
	}
	
	/***
	 * 
	 * @param robot
	 * @return
	 */
	public static boolean shootBall(Robot robot, double rpms)
	{
		if (robot.leftShooterWheel.get() != 1.0)
		{
			robot.leftShooterWheel.set(1.0);
			robot.rightShooterWheel.set(-1.0);
		}
		
		if ((robot.rightShooterWheel.getSpeed() > rpms) && 
				(Math.abs(robot.armController.getTargetAngle() - robot.armController.getAngle()) < 3.0))
		{
			robot.kicker.set(DoubleSolenoid.Value.kForward);
			robot.ballLoaded = false;
			return true;
		}
		
		return false;
	}
}

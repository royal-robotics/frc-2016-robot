package org.usfirst.frc.team2522.robot;

import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.GetImageSizeResult;
import com.ni.vision.NIVision.Rect;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class AutonomousController
{
	public static final double LOW_GOAL_TRAVERSE_ANGLE = -20.0;	// degrees
	
	public static final double LOW_GOAL_SHOT_ANGLE = 60.0;		// degrees
	public static final double LOW_GOAL_SHOT_POWER = 0.56;		// 
	public static final double LOW_GOAL_SHOT_RPMS = 2700.0;		// practice bot value

	public static final double DRIVE_FORWARD_SHOT_ANGLE = 60.0;		// degrees
	public static final double DRIVE_FORWARD_SHOT_POWER = 0.56; 	// 
	public static final double DRIVE_FORWARD_SHOT_RPMS = 2500.0;	// practice bot value
	
	public static final double BATTER_EDGE = 48;
	
	public static final double TARGET_RANGE[] = {
			12.0 + BATTER_EDGE,
		   	24.0 + BATTER_EDGE, 
		   	36.0 + BATTER_EDGE, 
		   	48.0 + BATTER_EDGE, 
		   	60.0 + BATTER_EDGE,
	};
	
	public static final double TARGET_WIDTH[] = {
			144.0,
			133.0,
			122.0,
			112.0,
			102.0,
	};
	
	public static final double TARGET_RATIO[] = {
			2.5,
			2.0,
			1.968,
			1.867,
			1.789,
	};
	
	public static final double TARGET_SHOT_RPMS[] = {
			2500.0,
			2500.0,
			2500.0,
			2500.0,
			2500.0,
	};
	
	public static final double TARGET_SHOT_ANGLE[] = {
			60.0,
			60.0,
			60.0,
			60.0,
			60.0,
	};
	
	public static final double TARGET_SHOT_POWER[] = {
			0.56,
			0.56,
			0.56,
			0.56,
			0.56,
	};
	

	static String autoModeString = "DoNothing";
	static int autoMode = 0;
	static int autoStep = 0;
	
	static Timer shotDelayTimer = new Timer();
	static double trackingAngle = 180.0;		
	

	
	/***
	 * Return whatever auto mode is set from the river station during setup.
	 * 
	 * @return
	 */
	public static int getAutoMode(Robot robot)
	{
		double value = (robot.leftstick.getZ() + 1.0);
		double increment = 2.0 / 5.0;
		
		if (value >= (4.0 * increment))
		{
			autoMode = 0;
			autoModeString = "DoNothing";
		}
		else if  (value >= 3.0 * increment)
		{
			autoMode = 4;
			autoModeString = "P5: DriveForwardTurnLeftAndShoot";
		}
		else if  (value >= 2.0 * increment)
		{
			autoMode = 3;
			autoModeString = "P3-4: DriveForwardAndShoot";
		}
		else if  (value >= 1.0 * increment)
		{
			autoMode = 2;
			autoModeString = "P2: DriveForwardTurnRightAndShoot";
		}
		else
		{
			autoMode = 1;
			autoModeString = "P1: LowBarAndShoot";
		}

		return autoMode;
	}
	
	public static void RunAutonomous(Robot robot)
	{
		switch(autoMode)
		{
			case 1:
				LowBarAndShootAuto(robot);
				break;
			case 2:
				DriveForwardTurnRightAndShoot(robot);
				break;
			case 3:
				DriveForwardAndShootAuto(robot);
				break;
			case 4:
				DriveForwardTurnLeftAndShoot(robot);
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
	public static void DriveForwardAndShootAuto(Robot robot)
	{
		switch(autoStep)
		{
			case 0:
			{
				if (driveForward(robot, 0.0, 0.80, 170.0))
				{
					autoStep++;
					trackingAngle = 180.0;	// reset tracking target before we start tracking
				}

				if (robot.leftDriveEncoder.getDistance() > 110.0)
				{
					armMove(robot, DRIVE_FORWARD_SHOT_ANGLE);
				}

				if (robot.leftDriveEncoder.getDistance() > 160.0)
				{
					if (robot.useRPMs)
					{
						if (robot.leftShooterWheel.get() != DRIVE_FORWARD_SHOT_RPMS)
						{
							robot.leftShooterWheel.set(DRIVE_FORWARD_SHOT_RPMS);
							robot.rightShooterWheel.set(-DRIVE_FORWARD_SHOT_RPMS);
						}
					}
					else
					{
						if (robot.leftShooterWheel.get() != DRIVE_FORWARD_SHOT_POWER)
						{
							robot.leftShooterWheel.set(DRIVE_FORWARD_SHOT_POWER);
							robot.rightShooterWheel.set(-DRIVE_FORWARD_SHOT_POWER);
						}
					}
				}
				
				break;
			}
			case 1:
			{
				if (trackTarget(robot))
				{
					shotDelayTimer.reset();
					autoStep++;
				}
				break;
			}
			case 2:
			{
				if (shootBall(robot, DRIVE_FORWARD_SHOT_RPMS, 2.0))
				{
					autoStep++;
				}
				break;
			}
			default:
				driveStop(robot);
				break;			
		}
	}

	/***
	 * 
	 * @param robot
	 */
	public static void DriveForwardTurnLeftAndShoot(Robot robot)
	{
		switch(autoStep)
		{
			case 0:
			{
				if (driveForward(robot, 0.0, 0.80, 170.0))
				{
					autoStep++;
				}

				if (robot.leftDriveEncoder.getDistance() > 110.0)
				{
					armMove(robot, DRIVE_FORWARD_SHOT_ANGLE);
				}

				if (robot.leftDriveEncoder.getDistance() > 160.0)
				{
					if (robot.useRPMs)
					{
						if (robot.leftShooterWheel.get() != DRIVE_FORWARD_SHOT_RPMS)
						{
							robot.leftShooterWheel.set(DRIVE_FORWARD_SHOT_RPMS);
							robot.rightShooterWheel.set(-DRIVE_FORWARD_SHOT_RPMS);
						}
					}
					else
					{
						if (robot.leftShooterWheel.get() != DRIVE_FORWARD_SHOT_POWER)
						{
							robot.leftShooterWheel.set(DRIVE_FORWARD_SHOT_POWER);
							robot.rightShooterWheel.set(-DRIVE_FORWARD_SHOT_POWER);
						}
					}
				}
				
				break;
			}
			case 1:
			{
				if (drivePivot(robot, -30.0, 0.65))
				{
					autoStep++;
					trackingAngle = 180.0;	// reset tracking target before we start tracking
				}
								
				break;
			}
			case 2:
			{
				if (trackTarget(robot))
				{
					shotDelayTimer.reset();
					autoStep++;
				}
				break;
			}
			case 3:
			{
				if (shootBall(robot, DRIVE_FORWARD_SHOT_RPMS, 2.0))
				{
					autoStep++;
				}
				break;
			}
			default:
				driveStop(robot);
				break;			
		}
	}

	/***
	 * 
	 * @param robot
	 */
	public static void DriveForwardTurnRightAndShoot(Robot robot)
	{
		switch(autoStep)
		{
			case 0:
			{
				if (driveForward(robot, 0.0, 0.80, 170.0))
				{
					autoStep++;
				}

				if (robot.leftDriveEncoder.getDistance() > 110.0)
				{
					armMove(robot, DRIVE_FORWARD_SHOT_ANGLE);
				}

				if (robot.leftDriveEncoder.getDistance() > 160.0)
				{
					if (robot.useRPMs)
					{
						if (robot.leftShooterWheel.get() != DRIVE_FORWARD_SHOT_RPMS)
						{
							robot.leftShooterWheel.set(DRIVE_FORWARD_SHOT_RPMS);
							robot.rightShooterWheel.set(-DRIVE_FORWARD_SHOT_RPMS);
						}
					}
					else
					{
						if (robot.leftShooterWheel.get() != DRIVE_FORWARD_SHOT_POWER)
						{
							robot.leftShooterWheel.set(DRIVE_FORWARD_SHOT_POWER);
							robot.rightShooterWheel.set(-DRIVE_FORWARD_SHOT_POWER);
						}
					}
				}
				
				break;
			}
			case 1:
			{
				if (drivePivot(robot, 30.0, 0.65))
				{
					autoStep++;
					trackingAngle = 180.0;	// reset tracking target before we start tracking
				}
								
				break;
			}
			case 2:
			{
				if (trackTarget(robot))
				{
					shotDelayTimer.reset();
					autoStep++;
				}
				break;
			}
			case 3:
			{
				if (shootBall(robot, DRIVE_FORWARD_SHOT_RPMS, 2.0))
				{
					autoStep++;
				}
				break;
			}
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
				armMove(robot, LOW_GOAL_TRAVERSE_ANGLE);
			}
			else
			{
				armMove(robot, LOW_GOAL_SHOT_ANGLE);
			}
			
			if (driveForward(robot, 0.0, 0.75, 239.0))
			{
				autoStep++;
			}
		
			break;
		}
		case 1:
			if (drivePivot(robot, 67.0, 0.65))
			{
				autoStep++;
				trackingAngle = 180.0;	// reset tracking target before we start tracking
			}
			
			if (robot.useRPMs)
			{
				if (robot.leftShooterWheel.get() != LOW_GOAL_SHOT_RPMS)
				{
					robot.leftShooterWheel.set(LOW_GOAL_SHOT_RPMS);
					robot.rightShooterWheel.set(-LOW_GOAL_SHOT_RPMS);
				}
			}
			else
			{
				if (robot.leftShooterWheel.get() != LOW_GOAL_SHOT_POWER)
				{
					robot.leftShooterWheel.set(LOW_GOAL_SHOT_POWER);
					robot.rightShooterWheel.set(-LOW_GOAL_SHOT_POWER);
				}
			}
			
			break;
		case 2:
			if (trackTarget(robot))
			{
				shotDelayTimer.reset();
				autoStep++;
			}
			break;
		case 3:
			if (shootBall(robot, LOW_GOAL_SHOT_RPMS, 2.0))
			{
				autoStep++;
			}
			break;
//		case 4:
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
	
	public static double driveGetBearing(Robot robot)
	{
		double bearing = robot.mxp.getAngle();
		
		if (bearing > 180.0) {
			bearing -= 360.0;
		}
		
		return bearing;
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
	 * @param power The drive motor power to use
	 * @return True if we have reached the desired heading, otherwise false.
	 */
	public static boolean drivePivot(Robot robot, double bearing, double power)
	{
		double error = bearing - driveGetBearing(robot);

		if (error > 180.0) {
			error -= 360.0;
		} else if (error < -180.0) {
			error += 360.0;
		}
		
		if (error < 1.0 && error > -1.0)
		{
			driveStop(robot);
			return true;
		}
		else
		{
			double pivotSpeed = power;
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
	 * Turn the robot to the right until a bearing of 180.0 has been reached.
	 * 
	 * @param robot	The robot being pivoted
	 * @param power The drive motor power to use
	 * @return True if we have reached the desired heading, otherwise false.
	 */
	public static boolean driveTurnAround(Robot robot, double power)
	{
		double bearing = robot.mxp.getAngle();
		
		if (bearing > 180.0 && bearing < 270.0)
		{
			driveStop(robot);
			return true;
		}
		else
		{
			robot.myDrive.tankDrive(-power, +power);
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
	 * Fire the robot shooter after setting the shooter motors to the specified RPMs and waiting the specified delay in seconds.
	 * 
	 * @param robot	The robot shooting.
	 * @param rpms	RPMS to set shooter wheels to.
	 * @param delay	Delay in seconds before shot is fired
	 * @return
	 */
	public static boolean shootBall(Robot robot, double rpms, double delay)
	{
		if (robot.useRPMs)
		{
			if (robot.leftShooterWheel.get() != rpms)
			{
				robot.leftShooterWheel.set(rpms);
				robot.rightShooterWheel.set(-rpms);
			}
		}
		else
		{
			if (robot.leftShooterWheel.get() == 0.0)
			{
				robot.leftShooterWheel.set(1.0 * (rpms / 4500.00));
				robot.rightShooterWheel.set(-1.0 * (rpms / 4500.00));
			}
		}
		
		if (shotDelayTimer.get() == 0)
		{
			shotDelayTimer.start();
		}

		if (shotDelayTimer.get() > delay)
		{
			robot.kicker.set(DoubleSolenoid.Value.kForward);
			robot.ballLoaded = false;
			return true;
		}
		
		return false;
	}
	
	/***
	 * Pivot the robot onto the target if a target is found.
	 * 
	 * @param robot
	 * @return true if robot is on target, otherwise false
	 */
	public static boolean trackTarget(Robot robot)
	{
		if (trackingAngle == 180.0)
		{
			driveStop(robot);
			double temp = AutonomousController.getTrackingAngle(robot);
			if (temp != 180.0)
			{
				trackingAngle = AutonomousController.driveGetBearing(robot) - temp;
			}
		}
		else
		{
			if (AutonomousController.drivePivot(robot, trackingAngle, 0.50))
			{
				trackingAngle = 180.0;
				driveStop(robot);
				return true;
			}
		}
		
		return false;
	}
	
	/***
	 * 
	 * @param robot
	 * @return
	 */
	public static double getTrackingAngle(Robot robot)
	{
		double result = 180.0;
		
		ImageTarget target = getTarget(robot);
		
		if (target != null)
		{
			GetImageSizeResult size = NIVision.imaqGetImageSize(robot.frame);
			int offset = (size.width / 2) - target.X();
			SmartDashboard.putNumber("Target Offset PXL", offset);
								
			// This ratio between degrees and pixels seems to work at field distances not sure how it will work up close.
			//
			result = (double)offset * 15.0 / 140.0;
			SmartDashboard.putNumber("Target Offset DEG", result);
		}
		else
		{
			SmartDashboard.putNumber("Target Offset PXL", -1);
			SmartDashboard.putNumber("Target Offset DEG", 180.0);
		}
    	
    	return result;
	}
	
	/***
	 * 
	 * @param robot
	 * @return
	 */
	public static double getTrackingRange(Robot robot)
	{
    	return getTrackingRange(getTarget(robot));
	}
	
	/***
	 * 
	 * @param target
	 * @return
	 */
	public static double getTrackingRange(ImageTarget target)
	{
		double result = 0.0;
		
		if (target != null)
		{
			result = TARGET_RANGE[0] + ((double)target.Width() * (TARGET_RANGE[TARGET_RANGE.length-1] - TARGET_RANGE[0]) / (TARGET_WIDTH[TARGET_RANGE.length-1] - TARGET_WIDTH[0]));
			SmartDashboard.putNumber("Target Range", result);
		}
		else
		{
			SmartDashboard.putNumber("Target Range", -1);
		}

    	return result;
	}
	
	/***
	 * 
	 * @param robot
	 * @return
	 */
	public static ImageTarget getTarget(Robot robot)
	{
		ImageTarget result = null;

    	if (robot.camera != null)
    	{
	    	NIVision.IMAQdxGrab(robot.session, robot.frame, 1);	// grab the raw image frame from the camera

    		robot.REFLECTIVE_RED_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("red low"), (int)SmartDashboard.getNumber("red high"));
    		robot.REFLECTIVE_GREEN_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("green low"), (int)SmartDashboard.getNumber("green high"));
    		robot.REFLECTIVE_BLUE_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("blue low"), (int)SmartDashboard.getNumber("blue high"));
	    	
	    	NIVision.imaqColorThreshold(robot.binaryFrame, robot.frame, 255, NIVision.ColorMode.RGB, robot.REFLECTIVE_RED_RANGE, robot.REFLECTIVE_GREEN_RANGE, robot.REFLECTIVE_BLUE_RANGE);
	    	
	    	//filter out small particles
			float areaMin = (float)SmartDashboard.getNumber("Target Area Min %", 0.05);
			robot.criteria[0].lower = areaMin;
			
			int numTargets = NIVision.imaqParticleFilter4(robot.particalFrame, robot.binaryFrame, robot.criteria, robot.filterOptions, null);

			SmartDashboard.putNumber("Target Candidates", numTargets);

			if(numTargets > 0)
			{
				//Measure particles and sort by particle size eliminating particles that don't match our profile.
				//
				Vector<ImageTarget> targets = new Vector<ImageTarget>();
				for(int i = 0; i < numTargets; i++)
				{
					ImageTarget target = new ImageTarget(
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT)
						);

					//par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(robot.particalFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					//par.Area = NIVision.imaqMeasureParticle(robot.particalFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					
					if (target.IsValidTarget())
					{
						Rect rect = new Rect(target.Top, target.Left, target.Height(), target.Width());
						if (targets.size() == 0)
						{
							//NIVision.imaqOverlayRect(robot.frame, rect, NIVision.RGB_RED, DrawMode.DRAW_VALUE, null);
							NIVision.imaqDrawShapeOnImage(robot.frame, robot.frame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);
						}
						else
						{
							//NIVision.imaqOverlayRect(robot.frame, rect, NIVision.RGB_BLACK, DrawMode.DRAW_VALUE, null);
							NIVision.imaqDrawShapeOnImage(robot.frame, robot.frame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);
						}
						
						targets.add(target);
					}
				}
				
				numTargets = targets.size();

				if (targets.size() > 0)
				{
					targets.sort(null);
					
					result = targets.elementAt(0);
					
					SmartDashboard.putNumber("Target X", result.X());
					SmartDashboard.putNumber("Target Y", result.Y());
					SmartDashboard.putNumber("Target W", result.Width());
					SmartDashboard.putNumber("Target H", result.Height());
					SmartDashboard.putNumber("Target Ratio", (double)result.Width() / (double)result.Height());
				}
			}

			SmartDashboard.putNumber("Target Count", numTargets);
    	}
		
    	if (result != null)
    	{
			SmartDashboard.putNumber("Target X", result.X());
			SmartDashboard.putNumber("Target Y", result.Y());
			SmartDashboard.putNumber("Target W", result.Width());
			SmartDashboard.putNumber("Target H", result.Height());
			SmartDashboard.putNumber("Target Ratio", (double)result.Width() / (double)result.Height());
    	}
    	else
    	{
			SmartDashboard.putNumber("Target X", -1);
			SmartDashboard.putNumber("Target Y", -1);
			SmartDashboard.putNumber("Target W", -1);
			SmartDashboard.putNumber("Target H", -1);
			SmartDashboard.putNumber("Target Ratio", -1);
    	}
    	
		return result;
	}
}

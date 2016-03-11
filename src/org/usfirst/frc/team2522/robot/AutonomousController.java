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
	public static final double LOW_GOAL_SHOT_ANGLE = 50.0;		// degrees
	public static final double LOW_GOAL_SHOT_POWER = 0.56;		// ?? rpms on practice robot.
	public static final double LOW_GOAL_SHOT_RPMS = 2700.0;		// not sure if accurate.

	public static final double DRIVE_FORWARD_SHOT_ANGLE = 50.0;
	public static final double DRIVE_FORWARD_SHOT_POWER = 0.56; 	// 3000 rpms on practice robot.
	public static final double DRIVE_FORWARD_SHOT_RPMS = 2700.0;	// 3200 rpms on practice robot.

	static String autoModeString = "LowBarAndShoot";
	static int autoMode = 0;
	static int autoStep = 0;
	
	static Timer shotDelayTimer = new Timer();
	
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
			autoModeString = "DriveForwardAndShoot";
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
				DriveForwardAndShootAuto(robot);
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
	public static void DriveForwardAndShootAuto(Robot robot)
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

				if (robot.leftDriveEncoder.getDistance() > 150.0)
				{
					if (robot.leftShooterWheel.get() != DRIVE_FORWARD_SHOT_POWER)
					{
						robot.leftShooterWheel.set(DRIVE_FORWARD_SHOT_POWER);
						robot.rightShooterWheel.set(-DRIVE_FORWARD_SHOT_POWER);
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
				if (shootBall(robot, DRIVE_FORWARD_SHOT_RPMS))
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
	public static void LowBarNoShotAuto(Robot robot)
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
			}
			
			if (robot.leftShooterWheel.get() != LOW_GOAL_SHOT_POWER)
			{
				robot.leftShooterWheel.set(LOW_GOAL_SHOT_POWER);
				robot.rightShooterWheel.set(-LOW_GOAL_SHOT_POWER);
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
			if (shootBall(robot, LOW_GOAL_SHOT_RPMS))
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
		double error = bearing - robot.mxp.getAngle();
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
	 * @param robot
	 * @return
	 */
	public static boolean shootBall(Robot robot, double rpms)
	{
		if (robot.rightShooterWheel.get() == 0.0)
		{
			robot.leftShooterWheel.set(1.0);
			robot.rightShooterWheel.set(-1.0);
		}
		
		if (robot.rightShooterWheel.getSpeed() >= rpms)
		{
			if (shotDelayTimer.get() == 0)
			{
				shotDelayTimer.start();
			}

			if (shotDelayTimer.get() > 2.0)
			{
				robot.kicker.set(DoubleSolenoid.Value.kForward);
				robot.ballLoaded = false;
				return true;
			}
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
		driveResetBearing(robot);
		
		double trackingAngle = getTrackingAngle(robot);
		SmartDashboard.putNumber("Target Pivot", trackingAngle);
		
		if (trackingAngle == 180.0)
		{
			driveStop(robot);
		}
		else if (trackingAngle > 1.0 || trackingAngle < -1.0)
		{
			drivePivot(robot, trackingAngle, 0.48);
		}
		else
		{
			driveStop(robot);
			return true;
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
		
    	if (robot.camera != null)
    	{
	    	NIVision.IMAQdxGrab(robot.session, robot.frame, 1);	// grab the raw image frame from the camera

    		robot.REFLECTIVE_RED_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("red low"), (int)SmartDashboard.getNumber("red high"));
    		robot.REFLECTIVE_GREEN_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("green low"), (int)SmartDashboard.getNumber("green high"));
    		robot.REFLECTIVE_BLUE_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("blue low"), (int)SmartDashboard.getNumber("blue high"));
	    	
	    	NIVision.imaqColorThreshold(robot.binaryFrame, robot.frame, 255, NIVision.ColorMode.RGB, robot.REFLECTIVE_RED_RANGE, robot.REFLECTIVE_GREEN_RANGE, robot.REFLECTIVE_BLUE_RANGE);
	    	
	    	//robot.camera.setImage(robot.binaryFrame);

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
					ImageTarget par = new ImageTarget(
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
							NIVision.imaqMeasureParticle(robot.particalFrame, i, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT)
						);

					//par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(robot.particalFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					//par.Area = NIVision.imaqMeasureParticle(robot.particalFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					
					if (par.IsValidTarget())
					{
						targets.add(par);
					}
				}
				
				numTargets = targets.size();

				if (targets.size() > 0)
				{
					targets.sort(null);
					
					ImageTarget target = targets.elementAt(0);
					
					Rect rect = new Rect(target.Top, target.Left, target.Height(), target.Width());
					//NIVision.imaqOverlayRect(frame, rect, NIVision.RGB_RED, DrawMode.DRAW_VALUE, null);
					NIVision.imaqDrawShapeOnImage(robot.frame, robot.frame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);
					
					// TODO: Calculate Target Angle.
					GetImageSizeResult size = NIVision.imaqGetImageSize(robot.frame);
					int offset = (size.width / 2) - target.X();
										
					SmartDashboard.putNumber("Target X", target.X());
					SmartDashboard.putNumber("Target Y", target.Y());
					SmartDashboard.putNumber("Target Ratio", (double)target.Width() / (double)target.Height());
					SmartDashboard.putNumber("Target Offset", offset);

					if (offset > 3) {
						result = -5.0;
					}
					else if (offset < -3) {
						result = +5.0;
					}
					else {
						result = 0.0;
					}
				}
				else
				{
					SmartDashboard.putNumber("Target X", -1);
					SmartDashboard.putNumber("Target Y", -1);
					SmartDashboard.putNumber("Target Ratio", -1);
					SmartDashboard.putNumber("Target Offset", -1);
				}
			}
			else
			{
				SmartDashboard.putNumber("Target X", -1);
				SmartDashboard.putNumber("Target Y", -1);
				SmartDashboard.putNumber("Target Ratio", -1);
				SmartDashboard.putNumber("Target Offset", -1);
			}

			SmartDashboard.putNumber("Target Count", numTargets);
    	}
    	
    	return result;
	}
}

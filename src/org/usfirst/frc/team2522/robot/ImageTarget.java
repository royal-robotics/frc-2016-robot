package org.usfirst.frc.team2522.robot;

import java.util.Comparator;

public class ImageTarget implements Comparator<ImageTarget>, Comparable<ImageTarget>{
	int Top;
	int Left;
	int Bottom;
	int Right;

	double ImageArea;
	double PercentAreaToImageArea;
	
	public ImageTarget(double top, double left, double bottom, double right)
	{
		this.Top = (int)top;
		this.Left = (int)left;
		this.Bottom = (int)bottom;
		this.Right = (int)right;
	}
	
	public int compareTo(ImageTarget r)
	{
		return (int)(r.Area() - this.Area());
	}
	
	public int compare(ImageTarget r1, ImageTarget r2)
	{
		return (int)(r1.Area() - r2.Area());
	}
	
	public int Width()
	{
		return this.Right - this.Left;
	}
	
	public int Height()
	{
		return this.Bottom - this.Top;
	}
	
	public int Area()
	{
		return this.Height() * this.Width();
	}
	
	public int X()
	{
		return this.Left + (this.Width() / 2);
	}
	
	public int Y()
	{
		return this.Top + (this.Height() / 2);
	}
	
	public boolean IsValidTarget()
	{
		double ratio = (double)this.Width() / (double)this.Height();
		
		if (ratio > 1.2 && ratio < 3.0)
		{
			return true;
		}
		
		return false;
	}
	
	/***
	 * Get the range in inches to the target calculated from the image size.
	 * 
	 * @return The range in inches that we are from the target calculated from the target width.
	 */
	public double Range()
	{
		return 0.0;
	}
	
	/***
	 * Get the angle offset in degrees to the target.
	 * 
	 * @return The rotation angle needed to line up on the target.
	 */
	public double Angle()
	{
		return 0.0;
	}
}

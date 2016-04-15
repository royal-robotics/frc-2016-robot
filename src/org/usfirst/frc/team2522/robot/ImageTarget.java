package org.usfirst.frc.team2522.robot;

import java.util.Comparator;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.GetImageSizeResult;
import com.ni.vision.NIVision.HistogramReport;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.Rect;

import edu.wpi.first.wpilibj.image.*;

public class ImageTarget implements Comparator<ImageTarget>, Comparable<ImageTarget>{
	int Top;
	int Left;
	int Bottom;
	int Right;

	double ImageArea;
	double PercentAreaToImageArea;
	double ImageEqAspectRatio;
	
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
	
	public boolean IsValidTarget(Image binary)
	{
		if (Width() > 180.0 || Width() < 45.0) {
			return false;
		}
		
		/*
		Image ideal = null;
		NIVision.imaqDuplicate(ideal, binary);
		setIdealUShape(ideal);
		return IsValidAspectRatio() && IsShape(binary, ideal, 0.75);
		*/
		
		return IsValidAspectRatio();
	}
	
	private boolean IsValidAspectRatio()
	{
		double ratio = (double)this.Width() / (double)this.Height();
				
		if (ratio > 1.2 && ratio < 3.0)
		{
			return true;
		}
		
		return false;
	}
	
	private boolean IsShape(Image original, Image ideal, double thresholdPercent)
	{
		Image absoluteDifference = null;
		NIVision.imaqAbsoluteDifference(absoluteDifference, ideal, original);
		
		GetImageSizeResult size = NIVision.imaqGetImageSize(absoluteDifference);
		int totalPixels = size.width * size.height;
		
		//This is setup for greyscale histogram (binary image), color would work better for absolute difference
		//Using itself as the mask shouldn't effect Histogram
		HistogramReport hist = NIVision.imaqHistogram(absoluteDifference, 1, 254, 255, absoluteDifference);
		int correctPixels = hist.histogram[0];
		double correctPercent = ((double) correctPixels) / ((double) totalPixels);
		
		return correctPercent >= thresholdPercent;
	}
	
	private void setIdealUShape(Image ideal)
	{	
		GetImageSizeResult size = NIVision.imaqGetImageSize(ideal);
		
		int numVertLines = (2 / (12 + 8)) / size.width;
		int numHoriLines = (2 / (12 + 2)) / size.height;
		
		NIVision.Point start = null;
		NIVision.Point end = null;
		
		for(int i = 0; i < numVertLines; i++)
		{
			start = new NIVision.Point(i + Left , 0 + Top);
			end = new NIVision.Point(i + Left, size.height + Top);
			NIVision.imaqDrawLineOnImage(ideal, ideal, DrawMode.DRAW_VALUE, start, end, 255); //Should be green
			
			start = new NIVision.Point(size.width - i + Left, 0 + Top);
			end = new NIVision.Point(size.width - i + Left, size.height + Top);
			NIVision.imaqDrawLineOnImage(ideal, ideal, DrawMode.DRAW_VALUE, start, end, 255); //should be green
		}
		
		for(int i = 0; i < numHoriLines; i++)
		{
			start = new NIVision.Point(0 + Left, 0 + Top);
			end = new NIVision.Point(size.width + Left, i + Top);
			NIVision.imaqDrawLineOnImage(ideal, ideal, DrawMode.DRAW_VALUE, start, end, 255); //should be green
		}
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

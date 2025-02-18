package org.firstinspires.ftc.teamcode.autoversion2;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class YellowObjectPipeline extends OpenCvPipeline {
    private Mat outputFrame = new Mat();
    private final Mat hsvFrame = new Mat();
    private final Mat mask = new Mat();
    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);
    private Point centroid = null;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to HSV color space
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Create a mask for yellow color
        Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);

        // Find contours in the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour and calculate its centroid
        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(contour);
                if (moments.get_m00() != 0) {
                    centroid = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());
                }
            }
        }

        // Draw the centroid on the output frame for visualization (optional)
        if (centroid != null) {
            Imgproc.circle(input, centroid, 5, new Scalar(255, 0, 0), -1); // Draw a red circle at the centroid
        }

        outputFrame.release(); // Release previous frame if necessary
        outputFrame = mask.clone(); // Clone the mask to outputFrame

        return outputFrame; // Return the processed frame
    }

    public Point getCentroid() {
        return centroid; // Return the current centroid
    }
}


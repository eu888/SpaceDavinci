package org.firstinspires.ftc.teamcode.autoversion2;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class yellowPipeline extends OpenCvPipeline {
    private Mat hsvImage = new Mat();
    private Mat yellowMask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    // Define the lower and upper bounds for yellow in HSV
    private static final Scalar LOWER_YELLOW = new Scalar(20, 100, 100);
    private static final Scalar UPPER_YELLOW = new Scalar(30, 255, 255);

    // Parameters for the frame size and camera FOV
    private final int frameWidth;
    private final int frameHeight;
    private final double cameraFOV;
    private final double offset;

    // Minimum area for contours to be considered valid
    private static final double MIN_CONTOUR_AREA = 500.0; // Adjust this value based on your needs

    public yellowPipeline(int frameWidth, int frameHeight, double cameraFOV, double offset) {
        this.frameWidth = frameWidth;
        this.frameHeight = frameHeight;
        this.cameraFOV = cameraFOV;
        this.offset = offset;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to HSV color space
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Create a mask for yellow colors
        Core.inRange(hsvImage, LOWER_YELLOW, UPPER_YELLOW, yellowMask);

        // Clear previous contours
        contours.clear();
        Mat hierarchy = new Mat();

        // Find contours in the yellow mask
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours based on area
        contours.removeIf(contour -> Imgproc.contourArea(contour) < MIN_CONTOUR_AREA);

        // Draw contours and calculate angles
        drawContours(input, contours);

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            Point center = new Point(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0);

            double angle = calculateAngle(center.x);
            boolean isStraightAhead = isObjectStraightAhead(angle);

            // Display angle and straight ahead status
            Imgproc.putText(input, "Angle: " + angle + " deg", new Point(center.x, center.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, "Straight: " + isStraightAhead, new Point(center.x, center.y + 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
        }

        return input;
    }

    private void drawContours(Mat frame, @NonNull List<MatOfPoint> contours) {
        for (MatOfPoint contour : contours) {
            Imgproc.drawContours(frame, List.of(contour), -1, new Scalar(0, 255, 0), 2);
        }
    }

    private double calculateAngle(double objectX) {
        double centerX = frameWidth / 2.0;
        double anglePerPixel = cameraFOV / frameWidth;
        double angle = (objectX - centerX) * anglePerPixel;
        return angle;
    }

    private boolean isObjectStraightAhead(double angle) {
        return Math.abs(angle) <= offset;
    }

    public List<MatOfPoint> getContours() {
        return contours;
    }
}
package org.firstinspires.ftc.teamcode.autoversion2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    private Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    private List<MatOfPoint> contours = new ArrayList<>();

    // Define the lower and upper bounds for yellow in HSV
    private static final Scalar LOWER_YELLOW = new Scalar(15, 80, 80);
    private static final Scalar UPPER_YELLOW = new Scalar(35, 255, 255);

    // Parameters for the frame size and camera FOV
    private final int frameWidth;
    private final int frameHeight;
    private final double cameraFOV;
    private final double offset;
    private final double focalLength = 700; // Example focal length, adjust based on calibration
    private final double realObjectHeight = 8.5; // Real-world height of specimen in cm
    private final double straightAheadTolerance = 5.0; // Allowable error in degrees

    // Minimum area for contours to be considered valid
    private static final double MIN_CONTOUR_AREA = 100;

    public int straightAheadCount = 0;

    public yellowPipeline(int frameWidth, int frameHeight, double cameraFOV, double offset) {
        this.frameWidth = frameWidth;
        this.frameHeight = frameHeight;
        this.cameraFOV = cameraFOV;
        this.offset = offset;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvImage, LOWER_YELLOW, UPPER_YELLOW, yellowMask);

        // Apply morphological operations
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, morphKernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, morphKernel);

        contours.clear();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contours.removeIf(contour -> Imgproc.contourArea(contour) < MIN_CONTOUR_AREA);

        straightAheadCount = 0;

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            Point center = new Point(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0);
            double angle = calculateAngle(center.x);
            boolean isStraightAhead = isObjectStraightAhead(angle);
            double distance = calculateDistance(boundingRect.height);

            if (isStraightAhead) {
                straightAheadCount++;
            }

            // Draw the contour and annotate information
            Imgproc.drawContours(input, List.of(contour), -1, new Scalar(0, 255, 0), 2);
            Imgproc.putText(input, "Angle: " + String.format("%.1f", angle) + " deg", new Point(center.x, center.y - 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, "Dist: " + String.format("%.1f", distance) + " cm", new Point(center.x, center.y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, "Straight: " + isStraightAhead, new Point(center.x, center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
        }
        return input;
    }

//    public void logTelemetry() {
//        telemetry.addData("Samples in front: ", straightAheadCount);
//        telemetry.update();
//    }

    private double calculateAngle(double objectX) {
        double centerX = frameWidth / 2.0;
        double anglePerPixel = cameraFOV / frameWidth;
        return (objectX - centerX) * anglePerPixel;
    }

    private boolean isObjectStraightAhead(double angle) {
        return Math.abs(angle) <= offset + straightAheadTolerance;
    }

    private double calculateDistance(double objectPixelHeight) {
        return (realObjectHeight * focalLength) / objectPixelHeight;
    }

    public List<MatOfPoint> getContours() {
        return contours;
    }
}

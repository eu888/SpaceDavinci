package org.firstinspires.ftc.teamcode.autoversion2;

import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;
import android.annotation.SuppressLint;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
/**
 * The pipeline for yellow samples.
 */
public class yellowPipeline extends OpenCvPipeline {
    private final Mat hsvImage = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    private final List<MatOfPoint> contours = new ArrayList<>();
    private final int frameWidth;
    private final int frameHeight;
    private final double cameraFOV;
    private final double offset;
    public int straightAheadCount = 0;
    /**
     * Parameters of the camera. All declare in <code>org.firstinspires.ftc.teamcode.autoversion2.robotData</code>.
     * @param frameWidth the width of the camera in <code>pixels</code>.
     * @param frameHeight the height of the camera in <code>pixels</code>.
     * @param cameraFOV the FOV of the camera.
     * @param offset the offset of the camera to the center of the robot <code>degrees</code>(°).
     */
    public yellowPipeline(int frameWidth, int frameHeight, double cameraFOV, double offset) {
        this.frameWidth = frameWidth;
        this.frameHeight = frameHeight;
        this.cameraFOV = cameraFOV;
        this.offset = offset;
    }
    /**
     * The main process object detection and annotating the results on the the frame
     * <p>Here separates the yellow objects and does morphological operation to reduce the noise and clean the mask.
     * It finds the contours and removes smaller ones based on a minim threshold. It then annotates the results
     * near the sample.</p>
     * @param input the camera feed that will be processed
     * @return The annotated input image with detected contours, angles, distances, and straight-ahead status of the yellow objects.
     * @throws IllegalArgumentException if the input image is null or not properly initialized.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvImage, LOWER_YELLOW, UPPER_YELLOW, yellowMask);

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

            Imgproc.drawContours(input, List.of(contour), -1, new Scalar(0, 255, 0), 2);
            Imgproc.putText(input, "Angle: " + String.format("%.1f", angle) + " deg", new Point(center.x, center.y - 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, "Dist: " + String.format("%.1f", distance) + " cm", new Point(center.x, center.y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, "Straight: " + isStraightAhead, new Point(center.x, center.y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
        }
        return input;
    }
    /**
     * Here it calculates the angel between the sample and camera
     * @param objectX is the horizontal size of the sample
     * @return the angle in degrees
     */
    private double calculateAngle(double objectX) {
        double centerX = frameWidth / 2.0;
        double anglePerPixel = cameraFOV / frameWidth;
        return (objectX - centerX) * anglePerPixel;
    }
    /**
     * based of the angle of the samples it determines if it is forward and in the range of tolerance and there can be added an offset
     * @param angle the angel between the camera and sample
     * @return if it is in front or not
     */
    private boolean isObjectStraightAhead(double angle) {
        return Math.abs(angle) <= offset + POSITION_TOLERANCE;
    }
    /**
     * Calculates the distance from the camera to the object
     * @param objectPixelHeight the height of the object in pixels
     * @return the distance in <code>cm</code>
     */
    private double calculateDistance(double objectPixelHeight) {
        return (OBJECT_HEIGHT * FOCAL_LENGTH) / objectPixelHeight;
    }
    /**
     * Get contours
     * @return <code>contours</code>
     */
    public List<MatOfPoint> getContours() {
        return contours;
    }

    /**
     * Get <code>frameHeight</code>
     * @deprecated Is No longer used and planed for future removal.
     * @return <code>frameHeight</code>
     */
    @Deprecated
    public int getFrameHeight() {
        return frameHeight;
    }
}
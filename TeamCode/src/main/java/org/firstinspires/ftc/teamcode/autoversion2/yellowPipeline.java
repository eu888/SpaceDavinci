package org.firstinspires.ftc.teamcode.autoversion2;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.jetbrains.annotations.Contract;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class yellowPipeline  extends OpenCvPipeline {
    private Mat hsvImage = new Mat();
    private Mat yellowMask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    private static final Scalar LOWER_YELLOW = new Scalar(20, 50, 50);
    private static final Scalar UPPER_YELLOW = new Scalar(30, 255, 255);
    private final int frameWith;
    private final int frameHeight;
    private final double cameraFOV;
    private final double offset;

    public yellowPipeline(int frameWith, int frameHeight, double cameraFOV, double offset){
        this.frameWith = frameWith;
        this.frameHeight = frameHeight;
        this.cameraFOV = cameraFOV;
        this.offset = offset;

    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvImage, LOWER_YELLOW, UPPER_YELLOW, yellowMask);

        contours.clear();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        drawContours(input, contours);

        for (MatOfPoint contour : contours){
            Rect boundingRect = Imgproc.boundingRect(contour);
            Point center = new Point(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0);

            double angle = calculateAngle(center.x);
            boolean isStraitAhead = isObjectStraitAhead(angle);

            Imgproc.putText(input, "Angle" + angle +"deg", new Point(center.x, center.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, "Strait" + angle, new Point(center.x, center.y + 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

        }

        return input;
    }
    private void drawContours(Mat frame, @NonNull List<MatOfPoint> contours){
        for(MatOfPoint contour : contours){
            Imgproc.drawContours(frame, List.of(contour), -1, new Scalar(0, 255, 0), 2);
        }
    }
    private double calculateAngle(double objectX){
        double centerX = frameWith / 2.0;
        double anglePerPixel = cameraFOV / frameWith;
        double angle = (objectX - centerX) * anglePerPixel;
        return angle;
    }
    private boolean isObjectStraitAhead(double angle){
        return Math.abs(angle) <= offset;
    }

    public List<MatOfPoint> getContours(){
        return contours;
    }
}

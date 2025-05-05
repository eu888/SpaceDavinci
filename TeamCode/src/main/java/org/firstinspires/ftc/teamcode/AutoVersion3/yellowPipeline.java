package org.firstinspires.ftc.teamcode.AutoVersion3;

import org.firstinspires.ftc.teamcode.autoversion2.robotData.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class yellowPipeline extends OpenCvPipeline {
    public Point yellowCenter = null;

    @Override
    public Mat processFrame(Mat input){
        Mat hsv = new Mat();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lower = new Scalar(20, 100, 100);
        Scalar upper = new Scalar(30, 255, 255);

        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if(!contours.isEmpty()){
            MatOfPoint largest = Collections.max(contours, Comparator.comparingDouble(Imgproc::contourArea));
            Moments moments = Imgproc.moments(largest);
            int cx = (int)(moments.m10 / moments.m00);
            int cy = (int)(moments.m01 / moments.m00);
            yellowCenter = new Point(cx, cy);

            Imgproc.circle(input, yellowCenter, 5, new Scalar(255, 0, 0), -1);
        }

        return input;
    }

}

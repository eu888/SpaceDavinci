package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class pipelineBlueTest extends OpenCvPipeline{
    Mat hsvImage = new Mat();
    Mat maskBlue = new Mat();
    Mat thresholded = new Mat();
    Mat edges = new Mat();
    public static double threshold1 = 0;
    public static double threshold2 = 255;
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBlue = new Scalar(100, 150, 50);
        Scalar upperBlue = new Scalar(140, 255, 255);

        Core.inRange(hsvImage, lowerBlue, upperBlue, maskBlue);
        Imgproc.threshold(maskBlue, thresholded, 0, 255, Imgproc.THRESH_BINARY);
        Imgproc.Canny(thresholded, edges, threshold1, threshold2);
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 50, 50, 10);
        for (int i = 0; i < lines.rows(); i++) {
            double[] line = lines.get(i, 0);
            Point pt1 = new Point(line[0], line[1]);
            Point pt2 = new Point(line[2], line[3]);
            Imgproc.line(input, pt1, pt2, new Scalar(255, 0, 0), 2); // Blue lines
        }
        return input;
    }
}
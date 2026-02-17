package org.firstinspires.ftc.teamcode.libs;

import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private final long nativeAprilTagPtr;
    private final double tagSize;
    private final double fx, fy, cx, cy;
    private final ArrayList<AprilTagDetection> latestDetections = new ArrayList<>();
    final Mat gray = new Mat();

    public AprilTagDetectionPipeline(double tagSize, double fx, double fy, double cx, double cy) {
        this.tagSize = tagSize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 1);
    }

    @Override
    public synchronized Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);
        ArrayList<AprilTagDetection> detections =
                AprilTagDetectorJNI.runAprilTagDetectorSimple(
                        nativeAprilTagPtr,
                        gray,
                        tagSize, fx, fy, cx, cy
                );
        latestDetections.clear();
        if (detections != null) {
            latestDetections.addAll(detections);
        }
        return input;
    }

    public synchronized ArrayList<AprilTagDetection> getLatestDetections() {
        return latestDetections;
    }

    @Override
    protected void finalize() {
        AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
    }
}


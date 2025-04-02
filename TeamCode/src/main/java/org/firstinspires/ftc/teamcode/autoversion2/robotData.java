package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class robotData {
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final int CAMERA_FOV = 60;
    public static final double moveDistance1 = 7;
    public static final double OBJECT_HEIGHT = 8.5; //cm
    public static final double FOCAL_LENGTH = 700;
    public static final double POSITION_TOLERANCE = 5.0; //grade
    public static final double MIN_CONTOUR_AREA = 100;
    public static final Scalar LOWER_YELLOW = new Scalar(15, 80, 80);
    public static final Scalar UPPER_YELLOW = new Scalar(35, 255, 255);
}

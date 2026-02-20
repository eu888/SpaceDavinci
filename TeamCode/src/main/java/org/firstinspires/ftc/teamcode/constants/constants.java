package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public final class constants {
    //for camera
    public static final double fx = 578.272;
    public static final double fy = 578.272;
    public static final double cx = 402.145;
    public static final double cy = 221.506;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final double tagSize = 0.2065;
    public static final double launchVelocityOn = 1600;
    public static final double intakePower = 1;
    public static final double servoBallLiftUp = 0.94;
    public static final double servoBallLiftDown = 0.52;
    public static final double P = 50;
    public static final double F = 16;
    public static final double D = 5;
    public static final Pose2d startPoseRedLower = new Pose2d(60,-24,Math.PI);
    public static final double difHeight =0.75-0.4;
}

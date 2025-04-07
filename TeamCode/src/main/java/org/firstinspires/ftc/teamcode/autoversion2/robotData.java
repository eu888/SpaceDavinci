package org.firstinspires.ftc.teamcode.autoversion2;
import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Scalar;
/**
 * The constants for the robot.
 */
@Config
public final class robotData {
    /**
     * Global setting for camera.
     */
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final int CAMERA_FOV = 60;
    public static final double FOCAL_LENGTH = 700;

    /**
     * Global settings for the pipelines
     */
    public static final double OBJECT_HEIGHT = 8.5;
    public static final double POSITION_TOLERANCE = 5.0;
    public static final double MIN_CONTOUR_AREA = 100;

    /**
     * Settings for the yellow pipeline
     */
    public static final Scalar LOWER_YELLOW = new Scalar(15, 80, 80);
    public static final Scalar UPPER_YELLOW = new Scalar(35, 255, 255);

    /**
     * Movement Settings for Autonomous
     * **/
    public static final double moveDistance1 = 7;
    /**
     * Servo positions
     */
    public static final double SERVO_CLAW_CLOSED = 1.0;
    public static final double SERVO_CLAW_OPEN = 0.69;

    public static final double SERVO_LOWER_ARM_IN = 0;
    public static final double SERVO_LOWER_ARM_MEDIUM_OUT = 0.7;
    public static final double SERVO_LOWER_ARM_OUT = 1;
    public static final double SERVO_UPPER_ARM_OUT = 0;
    public static final double SERVO_UPPER_ARM_IN = 1;
    public static final double SERVO_UPPER_CLAW_OPEN = 0.69;
    public static final double SERVO_UPPER_CLAW_CLOSED = 1;

}

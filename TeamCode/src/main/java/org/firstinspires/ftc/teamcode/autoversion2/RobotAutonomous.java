package org.firstinspires.ftc.teamcode.autoversion2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.autoversion2.lib.*;


public class RobotAutonomous {
    private final OpenCvCamera webcam;
    private YellowObjectPipeline pipeline; // Declare the pipeline
    private final double threshold = 10; // Define a threshold for alignment
    private DcMotor motorLB, motorLF, motorRB, motorRF; // Declare motors
    private DcMotor[] group1; // Group for left motors
    private DcMotor[] group2; // Group for right motors

    public RobotAutonomous() {
        // Initialize motors
        motorLB = hardwareMap.get(DcMotor.class, "motorLB");
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");

        // Initialize motor groups
        group1 = new DcMotor[]{motorLF, motorRB}; // Left motors
        group2 = new DcMotor[]{motorLB, motorRF}; // Right motors

        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        pipeline = new YellowObjectPipeline();
        webcam.setPipeline(pipeline);

        // Open the camera device asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera error
                System.err.println("Camera error: " + errorCode);
            }
        });
    }

    public void alignToYellowObject() {
        Point objectPosition = pipeline.getCentroid(); // Get the centroid of the yellow object

        if (objectPosition != null) {
            double centerX = (double) 640 / 2; // Assuming the frame width is 640, center is at 320
            double offsetX = objectPosition.x - centerX;

            if (Math.abs(offsetX) > threshold) {
                if (offsetX > 0) {
                    move(group1, group2, 0.5, "left"); // Move left
                } else {
                    move(group1, group2, 0.5, "right"); // Move right
                }
            } else {
                move(group1, group2, 0.5, "forward"); // Move forward
            }
        }
    }
}
package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.*;
import org.opencv.android.OpenCVLoader;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.autoversion2.yellowPipeline.*;

import java.util.List;

@Config
@Autonomous(name = "Yellow Detect Test")
public class detectYellowSpeciments extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    OpenCvCamera webcam;
    yellowPipeline pipeline;
    public static double mov = 7;
    private static int step = 0;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new yellowPipeline(640, 480, 60, 0);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
        dashboard.startCameraStream(webcam, 30);
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        // Slow down movement by reducing the velocity values for the trajectory
        double slowSpeed = 0.3; // Adjust the speed factor (0.0 to 1.0)

        // Keep moving left until a specimen is detected in front of the camera
        while (opModeIsActive()) {
            List<MatOfPoint> detectContours = pipeline.getContours();

            if (pipeline.straightAheadCount == 0) {  // If no specimen detected in front
                // Move left (strafe) a small distance (e.g., 5 units) at a time with reduced speed
                Actions.runBlocking(drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(beginPose.position.x, beginPose.position.y + mov)) // Adjust Y for left movement
                        .build());
            }    else {
                // If a specimen is detected in front of the camera, stop moving
                Actions.runBlocking(drive.actionBuilder(beginPose).endTrajectory().build());
                break;  // Exit the loop
            }

            telemetry.addData("Samples Detected", pipeline.getContours().size());
            telemetry.addData("Straight Ahead Count", pipeline.straightAheadCount);
            telemetry.update();
        }
    }
}

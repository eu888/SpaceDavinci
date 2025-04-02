package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.List;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;
@Autonomous(name = "Yellow Detect Test")
public class detectYellowSpecimens extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    OpenCvCamera webcam;
    yellowPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new yellowPipeline(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV, 0);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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

        while (opModeIsActive()) {
            List<MatOfPoint> detectContours = pipeline.getContours();

            if (pipeline.straightAheadCount == 0) {
                Actions.runBlocking(drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(beginPose.position.x, beginPose.position.y + moveDistance1))
                        .build());
            }    else {
                Actions.runBlocking(drive.actionBuilder(beginPose).endTrajectory().build());
                break;
            }

            telemetry.addData("Samples Detected", pipeline.getContours().size());
            telemetry.addData("Straight Ahead Count", pipeline.straightAheadCount);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.List;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;

import androidx.annotation.NonNull;

@Autonomous(name = "Yellow Detect Test")
public class detectYellowSpecimens extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    OpenCvCamera webcam;
    yellowPipeline pipeline;
    DcMotor motorRB, motorRF, motorLF, motorLB;

    @Override
    public void runOpMode() {
        motorRB = hardwareMap.get(DcMotor.class, "mrb");
        motorRF = hardwareMap.get(DcMotor.class, "mrf");
        motorLF = hardwareMap.get(DcMotor.class, "mlf");
        motorLB = hardwareMap.get(DcMotor.class, "mlb");
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
                motorRB.setPower(0.2);
                motorRF.setPower(-0.2);
                motorLF.setPower(0.2);
                motorLB.setPower(-0.2);
            }    else {
                motorRB.setPower(0.0);
                motorRF.setPower(0.0);
                motorLF.setPower(0.0);
                motorLB.setPower(0.0);
                return;
            }

            telemetry.addData("Samples Detected", pipeline.getContours().size());
            telemetry.addData("Straight Ahead Count", pipeline.straightAheadCount);
            telemetry.update();
        }
    }

    /**
     * Alight to a sample
     * <p>Here you need a pipeline that can count the objects strait in front</p>
     * @param pipeline your pipeline
     */
    private void alightToSample(@NonNull yellowPipeline pipeline){
        if (pipeline.straightAheadCount == 0) {
            motorRB.setPower(-0.2);
            motorRF.setPower(0.2);
            motorLF.setPower(-0.2);
            motorLB.setPower(0.2);
        }    else {
            motorRB.setPower(0.0);
            motorRF.setPower(0.0);
            motorLF.setPower(0.0);
            motorLB.setPower(0.0);
            return;
        }
    }
}

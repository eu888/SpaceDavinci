package org.firstinspires.ftc.teamcode.TeleOp;

//Standard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

//Self made libs
import static org.firstinspires.ftc.teamcode.constants.constants.*;
import static org.firstinspires.ftc.teamcode.libs.TeleOpLibs.*;

//For RoadRunner
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

//For AprilTags
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.libs.AprilTagDetectionPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name = "TeleOp")
@Config
public class TeleOp1 extends LinearOpMode {
    DcMotor motorLauncher, motorIntake;
    DcMotor motorUpperLeft, motorUpperRight, motorLowerLeft, motorLowerRight;
    Servo servoBallLift;
    Pose2d beginPose = new Pose2d(0, 0, 0);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    FtcDashboard dashboard;

    @Override
    public void runOpMode(){
        dashboard = FtcDashboard.getInstance();

        //Drive Base
        motorUpperLeft = hardwareMap.get(DcMotor.class,"motorUpperLeft");
        motorUpperRight = hardwareMap.get(DcMotor.class,"motorUpperRight");
        motorLowerLeft = hardwareMap.get(DcMotor.class, "motorLowerLeft");
        motorLowerRight = hardwareMap.get(DcMotor.class, "motorLowerRight");

        //AUX motors
        motorLauncher = hardwareMap.get(DcMotor.class, "motorLauncher");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");

        //Servos
        servoBallLift = hardwareMap.get(Servo.class, "servoBallLift");

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //ElapsedTime
        ElapsedTime elapsedTime = new ElapsedTime();

        //Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        dashboard.startCameraStream(camera, 30);

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()){
            double motorPower = 0.8;

            if(gamepad1.triangle){
                servoBallLift.setPosition(servoBallLiftDown);
            } else if (gamepad1.circle) {
                motorLauncher.setPower(launchPower);
            } else if (gamepad1.square) {
                servoBallLift.setPosition(servoBallLiftUp);
                waitFor(elapsedTime, 600, opModeIsActive());
                servoBallLift.setPosition(servoBallLiftDown);
            } else if (gamepad1.right_bumper) {
                 motorPower = 0.6;
            } else if (gamepad1.dpad_down) {
                motorIntake.setPower(-intakePower);
            } else if (gamepad1.dpad_up) {
                motorIntake.setPower(intakePower);
            } else if (gamepad1.dpad_right) {
                motorIntake.setPower(0);
            } else if  (gamepad1.dpad_left) {
                motorLauncher.setPower(idlePower);
            }

            double drives = -gamepad1.left_stick_y* motorPower;
            double strafe = -gamepad1.left_stick_x* motorPower;
            double turn = -gamepad1.right_stick_x* motorPower;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(drives, strafe), turn));

            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    telemetry.addLine(String.format("Detected tag ID=%d", tag.id));
                    telemetry.addLine(String.format("X: %.2f m", tag.pose.x));
                    telemetry.addLine(String.format("Y: %.2f m", tag.pose.y));
                    telemetry.addLine(String.format("Z: %.2f m", tag.pose.z));
                }
            }
            telemetry.update();
        }
    }


}

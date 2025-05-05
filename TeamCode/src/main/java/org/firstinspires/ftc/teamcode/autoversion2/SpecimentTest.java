package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoVersion3.yellowPipeline;
import org.firstinspires.ftc.teamcode.AutoVersion3.yellowPipeline.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;

import androidx.annotation.NonNull;

/**
 * Autonomous
 */
@Autonomous(name = "Autonomous")
public class SpecimentTest extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    yellowPipeline pipeline = new yellowPipeline();
    OpenCvCamera webcam;
    DcMotor motorRB, motorRF, motorLF, motorLB, armL, armR;
    Servo servoLowerArm, servoLowerClaw, servoRol, servoUpperArm, servoUpperArmRol, servoUpperClaw;
    Pose2d startPose = new Pose2d(30, -60, 0);

    @Override
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        DigitalChannel limitBtn;

        motorRB = hardwareMap.get(DcMotor.class, "mrb");
        motorRF = hardwareMap.get(DcMotor.class, "mrf");
        motorLF = hardwareMap.get(DcMotor.class, "mlf");
        motorLB = hardwareMap.get(DcMotor.class, "mlb");
        armL = hardwareMap.get(DcMotor.class, "par1");
        armR = hardwareMap.get(DcMotor.class, "perp");
        servoLowerArm = hardwareMap.get(Servo.class, "servoLowerArm");
        servoLowerClaw = hardwareMap.get(Servo.class, "servoLowerClaw");
        servoRol = hardwareMap.get(Servo.class, "servoRol");
        servoUpperArm = hardwareMap.get(Servo.class, "servoUpperArm");
        servoUpperArmRol = hardwareMap.get(Servo.class, "servoUpperArmRol");
        servoUpperClaw = hardwareMap.get(Servo.class, "servoUpperClaw");
        limitBtn = hardwareMap.get(DigitalChannel.class, "limitBtn");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam1", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Optional: handle the error, maybe display telemetry
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()){

//            Actions.runBlocking(drive.actionBuilder(startPose).lineToX(20).lineToY(-50).build());
            alignToYellowSample(drive, drive.localizer.getPose());
        }
    }

    public void alignToYellowSample(MecanumDrive drive, Pose2d currentPose) {
        if (pipeline.yellowCenter != null) {
            double frameCenterX = 160;
            double errorX = pipeline.yellowCenter.x - frameCenterX;

            if (Math.abs(errorX) > 15) {
                double correction = errorX * 0.005;

                Actions.runBlocking(drive.actionBuilder(currentPose)
                        .strafeTo(new Vector2d(correction, currentPose.position.y))
                        .build());
            }
        }
    }


}

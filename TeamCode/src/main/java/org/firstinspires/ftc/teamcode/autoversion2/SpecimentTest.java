package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.List;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;
/**
 * Autonomous
 */
@Autonomous(name = "Autonomous")
public class SpecimentTest extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
//    OpenCvCamera webcam;
    DcMotor motorRB, motorRF, motorLF, motorLB, armL, armR;
    Servo servoLowerArm, servoLowerClaw, servoRol, servoUpperArm, servoUpperArmRol, servoUpperClaw;
//    yellowPipeline pipeline = new yellowPipeline(CAMERA_WIDTH, CAMERA_HEIGHT,CAMERA_FOV, 0);
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
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new yellowPipeline(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV, 0);
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", errorCode);
//                telemetry.update();
//            }
//        });
//        dashboard.startCameraStream(webcam, 30);

        waitForStart();

        while (opModeIsActive()){
//            List<MatOfPoint> detectContours = pipeline.getContours();

            Actions.runBlocking(drive.actionBuilder(startPose).lineToX(20).lineToY(-50).build());
        }
    }

}

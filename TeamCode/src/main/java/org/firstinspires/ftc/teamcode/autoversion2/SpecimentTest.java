package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.AutoVersion3.yellowPipeline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;

/**
 * Autonomous
 */
@Autonomous(name = "AutonomousAss")
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

        dashboard.startCameraStream(webcam, 30);

        waitForStart();

        while (opModeIsActive()){
            if(pipeline.yellowCenter != null){
                double frameCenter = (double) CAMERA_WIDTH / 2;
                double errorX = pipeline.yellowCenter.x -frameCenter;

                if(Math.abs(errorX) < 30){
                        telemetry.addLine("detected");
                        telemetry.update();
                        break;
                }
            }

            Pose2d currentPose = drive.localizer.getPose();
            Pose2d stepLeft = new Pose2d(currentPose.position.x, currentPose.position.y + 1.5, currentPose.heading.log());

            Action strafeStep = drive.actionBuilder(currentPose).strafeTo(stepLeft.position).build();
            Actions.runBlocking(strafeStep);
        }
    }
}

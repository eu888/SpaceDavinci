package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import static org.firstinspires.ftc.teamcode.constants.constants.*;
import org.firstinspires.ftc.teamcode.libs.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.libs.Tag;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;


@Autonomous(name="Auto Red", group="RED")
public class AutoRed  extends LinearOpMode {
    MecanumDrive  drive;
    ElapsedTime time;

    DcMotor motorIntakeA, motorIntakeB;
    DcMotorEx motorLauncherA, motorLauncherB;

    PIDFCoefficients pidfCoefficients;

    Servo servoBallLift;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Tag currentTag;
    OpenCvCamera camera;
    FtcDashboard dashboard;

    @Override
    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap, startPoseRedLower);
        time = new ElapsedTime();
        pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize,fx,fy,cx,cy);
        dashboard = FtcDashboard.getInstance();
        currentTag = new Tag();

        //Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int i) {
                telemetry.addData("Camera error: ", i);
                telemetry.update();
            }
        });


        dashboard.startCameraStream(camera, 60);

        //AUX motors
        motorLauncherA = hardwareMap.get(DcMotorEx.class, "motorLauncherA");
        motorLauncherB = hardwareMap.get(DcMotorEx.class, "motorLauncherB");
        motorIntakeA = hardwareMap.get(DcMotor.class, "motorIntakeA");
        motorIntakeB = hardwareMap.get(DcMotor.class, "motorIntakeB");

        motorIntakeA.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLauncherA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        motorLauncherB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //Servos
        servoBallLift = hardwareMap.get(Servo.class, "servoBallLift");

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();
        if(!detections.isEmpty()){
            for(AprilTagDetection tag : detections){
                currentTag.tagId = tag.id;
                currentTag.X = tag.pose.x;
                currentTag.Z = tag.pose.z;
            }
        }

        double distance = Math.sqrt(Math.pow(currentTag.Z,2) - Math.pow(difHeight, 2));
        double forwardError = distance-0.5;
        double strafeError = currentTag.X;

        double fieldX = strafeError*Math.cos(drive.localizer.getPose().heading.real) - forwardError*Math.sin(drive.localizer.getPose().heading.real);
        double fieldY = strafeError*Math.sin(drive.localizer.getPose().heading.real) + forwardError*Math.cos(drive.localizer.getPose().heading.real);

        double targetX = drive.localizer.getPose().position.x + fieldX;
        double targetY = drive.localizer.getPose().position.y + fieldY;
        double targetHeading = drive.localizer.getPose().heading.real;//TODO: add heading error

        Vector2d targetVector = new Vector2d(targetX, targetY);
        Actions.runBlocking(drive.actionBuilder(startPoseRedLower)
                .strafeToSplineHeading(targetVector, Math.toRadians(targetHeading))
                .build());

        telemetry.addLine(String.format("Detected tag ID=%d", currentTag.tagId));
        telemetry.addLine(String.format("X: %.2f m", currentTag.X));
        telemetry.addLine(String.format("Y: %.2f m", currentTag.Y));
        telemetry.addLine(String.format("Z: %.2f m", currentTag.Z));
        telemetry.update();



    }


}

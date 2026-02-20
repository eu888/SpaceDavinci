package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.MecanumDrive;
import static org.firstinspires.ftc.teamcode.constants.constants.*;
import static org.firstinspires.ftc.teamcode.libs.TeleOpLibs.*;


@Autonomous(name="Auto Blue", group="Blue")
public class AutoBlue  extends LinearOpMode {
    MecanumDrive  drive;
    ElapsedTime time;

    DcMotor motorIntakeA, motorIntakeB;
    DcMotorEx motorLauncherA, motorLauncherB;

    PIDFCoefficients pidfCoefficients;

    Servo servoBallLift;
    Pose2d startPose = new Pose2d(61.3, 24, Math.PI);


    @Override
    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap, new Pose2d(61.3, 24, Math.PI));
        time = new ElapsedTime();
        pidfCoefficients = new PIDFCoefficients(P, 0, D, F);

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
        servoBallLift.setPosition(servoBallLiftDown);
        waitFor(this, time, 500);
        motorIntakeA.setPower(-intakePower);
        motorIntakeB.setPower(-intakePower);
        motorLauncherA.setVelocity(launchVelocityOn);
        motorLauncherB.setVelocity(launchVelocityOn);
        Actions.runBlocking( drive.actionBuilder(startPose)
                .strafeToSplineHeading(new Vector2d(-23,22), Math.toRadians(140))
                .strafeToLinearHeading(new Vector2d(-35,35), Math.toRadians(135))
                .waitSeconds(0.7)
                .stopAndAdd(() -> servoBallLift.setPosition(servoBallLiftUp))
                .waitSeconds(1.3)
                .stopAndAdd(() -> servoBallLift.setPosition(servoBallLiftDown))
                .waitSeconds(0.2)
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12.5,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12.5,58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12.5,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-35,35), Math.toRadians(135))
                .waitSeconds(0.7)
                .stopAndAdd(() -> servoBallLift.setPosition(servoBallLiftUp))
                .waitSeconds(1.3)
                .stopAndAdd(() -> servoBallLift.setPosition(servoBallLiftDown))
                .waitSeconds(0.2)
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(11.4,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(11.4,58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(11.4,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-35,35), Math.toRadians(135))
                .waitSeconds(0.7)
                .stopAndAdd(() -> servoBallLift.setPosition(servoBallLiftUp))
                .waitSeconds(1.3)
                .stopAndAdd(() -> servoBallLift.setPosition(servoBallLiftDown))
                .waitSeconds(0.2)
                .strafeToSplineHeading(new Vector2d(-23,23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-9,23), Math.toRadians(90))
                .build() );
    }


}

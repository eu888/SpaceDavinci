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


    @Override
    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap, new Pose2d(60,-24,Math.PI));
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
        waitFor(this, time, 250);
        Actions.runBlocking(drive.actionBuilder(new Pose2d(-64, -20, Math.PI))
                .strafeToSplineHeading(new Vector2d(-40,-20), Math.toRadians(135))
                .build());

        waitFor(this, time, 1500);
        servoBallLift.setPosition(servoBallLiftUp);
        waitFor(this, time, 2700);
        servoBallLift.setPosition(servoBallLiftDown);
        waitFor(this, time, 200);
    }


}

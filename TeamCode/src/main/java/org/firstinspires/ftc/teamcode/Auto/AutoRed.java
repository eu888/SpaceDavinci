package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="1")
public class AutoRed  extends LinearOpMode {


    @Override
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60,24,Math.PI));

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(drive.actionBuilder(new Pose2d(60, 24, Math.PI))
                .strafeToSplineHeading(new Vector2d(0,15), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12.3, 46.7), Math.toRadians(90))
                .build());

    }


}

package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class testAuto extends LinearOpMode{
    @Override
    public void runOpMode(){
        Pose2d beginPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(drive.actionBuilder(beginigPose)
                            lineTo(new Vector2d(30, 0)) 
                            .turn(Math.toRadians(90))
                            .lineTo(new Vector2d(30, 30)) 
                            .build());
    }
}

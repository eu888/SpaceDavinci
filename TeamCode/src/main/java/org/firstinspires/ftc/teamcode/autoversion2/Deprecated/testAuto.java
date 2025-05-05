package org.firstinspires.ftc.teamcode.autoversion2.Deprecated;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * This was a test <code>OpMode</code> to try and make an Auto with <code>RoadRunner 1.0</code>
 * @deprecated This will be removed in future.
 */
@Deprecated
@Disabled
@TeleOp(name = "movTest")
public final class testAuto extends LinearOpMode{
    @Override
    public void runOpMode(){
        Pose2d beginPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(drive.actionBuilder(beginPose)
                .lineToY(30)
                .build());
    }


    public static class retragere implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            return false;
        }
    }
}

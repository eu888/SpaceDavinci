package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Config
@TeleOp
public class test extends LinearOpMode {
    public static int target,targetext = 0;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorB = hardwareMap.get(DcMotor.class, "mb");
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        target = 0;
        motorB.setTargetPosition(target);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dashboard = FtcDashboard.getInstance();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad2.square)
            {
                target = targetext;
            }
            motorB.setTargetPosition(target);
            motorB.setPower(0.7);
            telemetry.addData("motor", motorB.getCurrentPosition());
            telemetry.update();
        }
    }
}

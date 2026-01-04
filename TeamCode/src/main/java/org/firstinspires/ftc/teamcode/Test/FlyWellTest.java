package org.firstinspires.ftc.teamcode.Test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name="FlyWell test", group = "test")
@Config
public class FlyWellTest extends OpMode {

    public DcMotorEx flyWellMotor;
    PIDFCoefficients pidfCoefficients;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    public double curTargetVelocity = highVelocity;
    public static double curVelocity;

    double F = 0;
    double P = 0;
    double D = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        flyWellMotor = hardwareMap.get(DcMotorEx.class, "motorLauncher");
        flyWellMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        flyWellMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop(){
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.squareWasPressed()){
            D += stepSizes[stepIndex];
        }
        if (gamepad1.crossWasPressed()){
            D -= stepSizes[stepIndex];
        }

        pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        flyWellMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flyWellMotor.setVelocity(curTargetVelocity);

        curVelocity = flyWellMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;
        telemetry.addData("Target Velocity: ", curTargetVelocity);
        telemetry.addData("Current Velocity: ", curVelocity);
        packet.put("Current Velocity: ", curVelocity);
        telemetry.addData("Error: ", "%.2f", error);
        telemetry.addData("P: ", "%.4f", P);
        telemetry.addData("F: ", "%.4f", F);
        telemetry.addData("D: ", "%.4f", D);
        telemetry.addData("Step size: ", "%.4f", stepSizes[stepIndex]);
        telemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }
}

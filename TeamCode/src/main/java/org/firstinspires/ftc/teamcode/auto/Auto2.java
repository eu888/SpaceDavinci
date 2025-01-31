package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.command.ProfiledPIDCommand;

@Autonomous(name="Auto")
public class Auto2 extends LinearOpMode{
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;
    PIDController pidController = new PIDController(Kp, Ki, Kd);
    Motor motorLB,motorLF,motorRB,motorRF,motorB;
    DcMotorEx motorE;
    Servo sr1;
    int limit = 1225;

    @Override
    public void runOpMode() throws InterruptedException{
        motorRB = hardwareMap.get(Motor.class, "mrb");
        motorRF = hardwareMap.get(Motor.class, "mrf");
        motorLF = hardwareMap.get(Motor.class, "mlf");
        motorLB = hardwareMap.get(Motor.class, "mlb");
        motorB = hardwareMap.get(Motor.class, "mb");
        motorE = hardwareMap.get(DcMotorEx.class, "me");
        motorRB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sr1 = hardwareMap.get(Servo.class, "sr1");

        pidController.setTolerance(5, 10);

        MecanumDrive mecanumDrive = new MecanumDrive(motorLF, motorRF, motorLB, motorRB);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Sr1", sr1.getPosition());
            telemetry.update();



        }
    }
}




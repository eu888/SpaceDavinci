package org.firstinspires.ftc.teamcode.autoversion2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="TestAutoVersion1")
public class Auto extends LinearOpMode{
    DcMotor motorLB,motorLF,motorRB,motorRF,motorB, motorE;
    Servo sr1, sr2, sr5;
    int limit = 1225;

    @Override
    public void runOpMode() throws InterruptedException{
        motorRB = hardwareMap.get(DcMotor.class, "mrb");
        motorRF = hardwareMap.get(DcMotor.class, "mrf");
        motorLF = hardwareMap.get(DcMotor.class, "mlf");
        motorLB = hardwareMap.get(DcMotor.class, "mlb");
        motorB = hardwareMap.get(DcMotor.class, "mb");
        motorE = hardwareMap.get(DcMotor.class, "me");
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        sr1 = hardwareMap.get(Servo.class, "sr1");
        sr2 = hardwareMap.get(Servo.class, "sr2");
        sr5 = hardwareMap.get(Servo.class, "sr5");

        encoderRest();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        RobotAutonomous robot = new RobotAutonomous();

        while (opModeIsActive()){
            robot.alignToYellowObject();
        }
    }

    public void encoderRest(){
        motorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

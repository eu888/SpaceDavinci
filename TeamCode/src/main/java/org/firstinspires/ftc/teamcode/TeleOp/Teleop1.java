package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.autoversion2.lib.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Test")
@Config
public class Teleop1 extends LinearOpMode{



    DcMotor motorLB,motorLF,motorRB,motorRF,motorB, motorE;
    Servo sr1, sr2, sr3, sr4, sr5;
    public static int target = 0;
    public static double servomarein = 0.54255;
    public static double servomareout = 0.4924;
    public static double servobratjos = 0.79275;    FtcDashboard dashboard;

    public static double servobratsus = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        dashboard = FtcDashboard.getInstance();

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
        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);


        sr1 = hardwareMap.get(Servo.class, "sr1");
        sr2 = hardwareMap.get(Servo.class, "sr2");
        sr3 = hardwareMap.get(Servo.class, "sr3");
        sr4 = hardwareMap.get(Servo.class, "sr4");
        sr5 = hardwareMap.get(Servo.class, "sr5");

        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        target = 0;
        motorB.setTargetPosition(target);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        encoderReset();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            telemetry.addData("Motor B", motorB.getCurrentPosition());
            telemetry.addData("Motor E", motorE.getCurrentPosition());
            telemetry.addData("Servo sr1", sr1.getPosition());
            telemetry.addData("Servo sr2", sr2.getPosition());
            telemetry.addData("Servo sr3", sr3.getPosition());
            telemetry.update();

            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
            rotX = rotX * 1.1;
            double rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double LBPower = -((rotY - rotX - rx) / denominator) * 0.485;
            double LFPower = -((rotY + rotX - rx) / denominator) * 0.485;
            double RFPower = -((rotY - rotX + rx) / denominator) * 0.485;
            double RBPower = -((rotY + rotX + rx) / denominator) * 0.485;

            if (gamepad1.options){
                imu.resetYaw();
            }

            motorRB.setPower(-(lx+ly-rx)*0.45);
            motorRF.setPower(-(-lx+ly-rx)*0.45);
            motorLF.setPower(-(lx+ly+rx)*0.45);
            motorLB.setPower(-(-lx+ly+rx)*0.45);
            motorE.setPower(0.0);
            motorB.setTargetPosition(target);
            motorB.setPower(0.7);


            if(gamepad2.square){
                target = -1900;
                sr5.setPosition(0.51);
                sr4.setPosition(0.1);
                sr1.setPosition(0.6);
            }
            if(gamepad2.circle){
                sr5.setPosition(servomareout);
                sleep(800);
                sr4.setPosition(0.34);
                sleep(250);
                target = 0;
                sr5.setPosition(servomarein);
                sr3.setPosition(0.53);
                sleep(1300);
                sr1.setPosition(servobratjos);
                sr2.setPosition(0.1);
            }
            if(gamepad2.triangle){
                sr2.setPosition(0.34);
                sleep(150);
                sr4.setPosition(0.1);
            }
            if(gamepad2.cross){
                sr1.setPosition(servobratsus);
                sleep(1500);

                sr2.setPosition(0.1);
            }
            if (gamepad2.dpad_left){
                motorE.setPower(1);
            }
            if (gamepad2.dpad_right) {
                motorE.setPower(-1);
            }
            if(gamepad2.left_bumper){
                sr1.setPosition(servobratjos);
            }
            if (gamepad2.right_bumper){
                sr1.setPosition(servobratsus);
            }
            if(gamepad2.right_trigger !=0 ){
                sr3.setPosition(0.41);
            }
            if(gamepad2.left_trigger != 0){
                sr3.setPosition(0.65);
            }
            if(gamepad2.right_bumper){
                sr3.setPosition(0.53);
            }
            if(gamepad2.right_stick_button){
                sr4.setPosition(0);
            }

            motorB.setTargetPosition(target);
            motorB.setPower(0.75);

        }
    }

    public void encoderReset(){
        motorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

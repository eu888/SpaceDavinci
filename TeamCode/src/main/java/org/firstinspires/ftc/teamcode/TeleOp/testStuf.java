package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;
import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="TestNigger")
public class testStuf extends LinearOpMode{



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

        while (opModeIsActive()){
            telemetry.addData("Motor B", motorB.getCurrentPosition());
            telemetry.addData("Motor E", motorE.getCurrentPosition());
            telemetry.addData("Servo sr1", sr1.getPosition());
            telemetry.addData("Servo sr2", sr2.getPosition());
            telemetry.addData("Servo sr3", sr5.getPosition());
            telemetry.update();

            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
            rotX = rotX * 1.1;
            double rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double LBPower = -((rotY - rotX + rx) / denominator) * 0.485;
            double LFPower = -((rotY + rotX + rx) / denominator) * 0.485;
            double RFPower = -((rotY - rotX - rx) / denominator) * 0.485;
            double RBPower = -((rotY + rotX - rx) / denominator) * 0.485;

            if (gamepad1.options){
                imu.resetYaw();
            }

            motorRB.setPower(-(-lx+ly+rx)*0.5);
            motorRF.setPower(-(lx+ly+rx)*0.5);
            motorLF.setPower(-(-lx+ly-rx)*0.5);
            motorLB.setPower(-(lx+ly-rx)*0.5);
            motorE.setPower(0.0);
            motorB.setPower(0.0);



            if (gamepad2.dpad_up) {
                motorB.setPower(0.37);
            } else if (gamepad2.dpad_down) {
                if(motorE.getCurrentPosition() < limit){
                    motorB.setPower(-0.37);
                }else {
                    motorB.setPower(0.0);
                }
            } else if (gamepad2.dpad_right) {
                int motorBPosition = motorB.getCurrentPosition();

                if (motorBPosition < -88) {
                    if (motorE.getCurrentPosition() <= limit) {
                        motorE.setPower(0.5);
                    } else {
                        motorE.setPower(0);
                    }
                } else {
                    motorE.setPower(0.5);
                }
            } else if (gamepad2.dpad_left) {
                motorE.setPower(-0.5);//1284
            } else if (gamepad2.triangle) {
                sr5.setPosition(0.499);//mare
            } else if(gamepad2.cross){
                sr5.setPosition(0.555);//mare
            }  else if(gamepad2.square){
                sr5.setPosition(0.53);//jos_rol
            } else if(gamepad2.circle){
                sr5.setPosition(0.3);//all_cl
            } else if(gamepad2.left_trigger != 0){
                sr5.setPosition(0.83);//brat
            } else if (gamepad2.right_trigger != 0) {
                sr5.setPosition(0.0);//zero
            } else if(gamepad2.right_stick_button){
                encoderRest();
            }

        }
    }

    public void encoderRest(){
        motorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

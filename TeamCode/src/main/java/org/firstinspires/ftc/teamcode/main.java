package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Ver_2.7.2")
public class main extends OpMode{
    DcMotor motorLB,motorLF,motorRB,motorRF,motorB, motorE;
    Servo sr1, sr2;
    double sr1Position = 0.5;
    int a = 1;
    double sm = 0.375;

    @Override
    public void init(){

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

        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);


        motorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sr1 = hardwareMap.get(Servo.class, "sr1");
        sr2 = hardwareMap.get(Servo.class, "sr2");

        telemetry.addData("Hardware", "Initialized");

    }

    @Override
    public void loop() {
        int limit = 1225;

        telemetry.addData("Motor RB", motorRB.getCurrentPosition());
        telemetry.addData("Motor Rf", motorRF.getCurrentPosition());
        telemetry.addData("Motor LF", motorLF.getCurrentPosition());
        telemetry.addData("Motor LB", motorLB.getCurrentPosition());
        telemetry.addData("Motor B", motorB.getCurrentPosition());
        telemetry.addData("Motor E", motorE.getCurrentPosition());
        telemetry.addData("Servo sr1", sr1.getPosition());
        telemetry.addData("Servo sr2", sr2.getPosition());
        telemetry.update();

        float lx = gamepad1.left_stick_x;
        float ly = gamepad1.left_stick_y;
        float rx = gamepad1.right_stick_x;

        if (gamepad1.dpad_up){
            motorLB.setPower(sm);
            motorLF.setPower(sm);
            motorRF.setPower(sm);
            motorRB.setPower(sm);
        }else if(gamepad1.dpad_down){
            motorLB.setPower(-sm);
            motorLF.setPower(-sm);
            motorRF.setPower(-sm);
            motorRB.setPower(-sm);
        }else if(gamepad1.dpad_left){
            motorLB.setPower(sm);
            motorLF.setPower(-sm);
            motorRF.setPower(sm);
            motorRB.setPower(-sm);
        }else if(gamepad1.dpad_right){
            motorLB.setPower(-sm);
            motorLF.setPower(sm);
            motorRF.setPower(-sm);
            motorRB.setPower(sm);
        } else if (gamepad2.triangle) {
            sr1.setPosition(0.2);
        } else if (gamepad2.cross) {
            sr1.setPosition(0.82);
        } else if (gamepad2.dpad_up) {
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
        } else if (gamepad2.square) {
            sr2.setPosition(0.19);
        } else if (gamepad2.circle) {
            sr2.setPosition(0.09);
        } else if (gamepad2.right_bumper) {
            sr1.setPosition(0.5);
        } else if (gamepad2.left_bumper) {
            sr1.setPosition(0.77);
        } else {
            motorRB.setPower(-(-lx+ly)*sm);
            motorRF.setPower(-(lx+ly-rx)*sm);
            motorLF.setPower(-(-lx+ly+rx)*sm);
            motorLB.setPower(-(lx+ly)*sm);
            motorE.setPower(0.0);
            motorB.setPower(0.0);

        }




    }

}
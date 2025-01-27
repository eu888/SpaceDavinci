package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto2", group = "Red")
public class Auto2 extends OpMode{
    DcMotor motorLB,motorLF,motorRB,motorRF,motorB, motorE;
    double power = 0.35;

    @Override
    public void init(){
        motorSetup();
    }

    @Override
    public void loop() {
        forwards(power);
        sleep(2320);
        allStop();
        left(power);
        sleep(470);
        allStop();
        backwards(power);
        sleep(2270);
        allStop();
        forwards(power);
        sleep(2270);
        left(power);
        sleep(535);
        allStop();
        backwards(power);
        sleep(2270);
        forwards(power);
        sleep(2270);
        left(power);
        sleep(560);
        allStop();
        backwards(power);
        sleep(2120);
        allStop();

        requestOpModeStop();
    }

    public void motorSetup(){
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

    }

    private void runMotors(@NonNull DcMotor[] motors, double power){
        for(DcMotor motor : motors){
            motor.setPower(power);
        }
    }

    private void stopMotors(@NonNull DcMotor[] motors){
        for(DcMotor motor : motors){
            motor.setPower(0.0);
        }
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void forwards(double power){
        runMotors(new DcMotor[]{motorRF, motorLB}, power);
        runMotors(new DcMotor[]{motorRB, motorLF}, power);
    }

    private void backwards(double power){
        runMotors(new DcMotor[]{motorRF, motorLB}, -power);
        runMotors(new DcMotor[]{motorRB, motorLF}, -power);
    }

    private void right(double power){
        runMotors(new DcMotor[]{motorRF, motorLB}, -power);
        runMotors(new DcMotor[]{motorRB, motorLF}, power);
    }

    private void left(double power){
        runMotors(new DcMotor[]{motorRF, motorLB}, power);
        runMotors(new DcMotor[]{motorRB, motorLF}, -power);
    }

    private void allStop(){
        stopMotors(new DcMotor[]{motorRB, motorRF, motorLF, motorLB});
    }
}


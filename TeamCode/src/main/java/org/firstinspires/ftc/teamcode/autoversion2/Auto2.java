package org.firstinspires.ftc.teamcode.autoversion2;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Auto")
public class Auto2 extends LinearOpMode{
    DcMotor motorLB,motorLF,motorRB,motorRF,motorB;
    DcMotorEx motorE;
    Servo sr1;
    int limit = 1225;

    @Override
    public void runOpMode() throws InterruptedException{
        motorRB = hardwareMap.get(DcMotor.class, "mrb");
        motorRF = hardwareMap.get(DcMotor.class, "mrf");
        motorLF = hardwareMap.get(DcMotor.class, "mlf");
        motorLB = hardwareMap.get(DcMotor.class, "mlb");
        motorB = hardwareMap.get(DcMotor.class, "mb");
        motorE = hardwareMap.get(DcMotorEx.class, "me");
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);


        sr1 = hardwareMap.get(Servo.class, "sr1");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Sr1", sr1.getPosition());
            telemetry.update();

            DcMotor[] group1 = {motorLF, motorRB};
            DcMotor[] group2 = {motorLB, motorRF};
            DcMotor[] all = {motorRF, motorLB, motorLF, motorRB};

            runMotors(group1, 0.5);
            runMotors(group2, -0.5);
            sleep(1400);
            stopMotors(all);
            requestOpModeStop();

        }
    }
    private void runMotors(@NonNull DcMotor[] motors, double power){
        for(DcMotor motor : motors){
            motor.setPower(power);
        }
    }

    public void stopMotors(@NonNull DcMotor[] motors){
        for(DcMotor motor : motors){
            motor.setPower(0.0);
        }
    }


}




package org.firstinspires.ftc.teamcode.AutoVersion3;


import static org.firstinspires.ftc.teamcode.autoversion2.robotData.SERVO_UPPER_ARM_IN;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.SERVO_UPPER_CLAW_OPEN;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Try")
public class Auto extends LinearOpMode {
    ElapsedTime elapsedTime = new ElapsedTime();
    DcMotor armL, armR, motorBR1, motorBR2;
    Servo servoLowerArm, servoLowerClaw, servoRol, servoUpperArm, servoUpperArmRol, servoUpperClaw;
    Pose2d startPose = new Pose2d(-30, -60, Math.toRadians(90));
    DigitalChannel limitBtn;

    @Override
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        motorBR1 = hardwareMap.get(DcMotor.class, "par0");
        motorBR2 = hardwareMap.get(DcMotor.class, "mbr");
        armL = hardwareMap.get(DcMotor.class, "par1");
        armR = hardwareMap.get(DcMotor.class, "perp");
        servoLowerArm = hardwareMap.get(Servo.class, "servoLowerArm");
        servoLowerClaw = hardwareMap.get(Servo.class, "servoLowerClaw");
        servoRol = hardwareMap.get(Servo.class, "servoRol");
        servoUpperArm = hardwareMap.get(Servo.class, "servoUpperArm");
        servoUpperArmRol = hardwareMap.get(Servo.class, "servoUpperArmRol");
        servoUpperClaw = hardwareMap.get(Servo.class, "servoUpperClaw");
        limitBtn = hardwareMap.get(DigitalChannel.class, "limitBtn");
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limitBtn.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();


        Action action1 = drive.actionBuilder(startPose).lineToY(-50).strafeToLinearHeading(new Vector2d(-82,-53), Math.toRadians(45)).build();
        Actions.runBlocking(action1);
        motorBR1.setPower(1.0);
        motorBR2.setPower(1.0);
        waitFor(1500);
        motorBR1.setPower(0.03);
        motorBR2.setPower(0.03);
        servoUpperArm.setPosition(0.3);
        servoUpperArmRol.setPosition(0.47);
        waitFor(2000);
        servoUpperClaw.setPosition(SERVO_UPPER_CLAW_OPEN);
        waitFor(500);
        servoUpperArm.setPosition(SERVO_UPPER_ARM_IN-0.4);
        waitFor(30000-1500-2000-500);

    }

    /**
     * This is a wait function that uses elapseTime
     * @param milliseconds the time in milliseconds to wait.
     */
    private void waitFor(double milliseconds){
        elapsedTime.reset();
        while (opModeIsActive() && elapsedTime.milliseconds() < milliseconds){
            telemetry.addData("Wait", elapsedTime.milliseconds());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;


@TeleOp(name="Test")
@Config
public class Teleop1 extends LinearOpMode{
    ElapsedTime elapsedTime = new ElapsedTime();
    DigitalChannel limitBtn;
    double Speed=1;
    Servo servoLowerArm, servoLowerClaw, servoRol, servoUpperArm, servoUpperArmRol, servoUpperClaw;
    DcMotor motorRB, motorRF, motorLF, motorLB, motorBR1, motorBR2, armL, armR;
    @Override
    public void runOpMode() {
        motorRB = hardwareMap.get(DcMotor.class, "mrb");
        motorRF = hardwareMap.get(DcMotor.class, "mrf");
        motorLF = hardwareMap.get(DcMotor.class, "mlf");
        motorLB = hardwareMap.get(DcMotor.class, "mlb");
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
        motorBR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limitBtn.setMode(DigitalChannel.Mode.INPUT);

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        boolean toClose = false;

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            boolean isNotPressed = limitBtn.getState();
            telemetry.addData("Brat extins", isNotPressed);
            telemetry.addData("Brat ticks", motorBR1.getCurrentPosition());
            telemetry.update();

            Speed = gamepad1.right_trigger > 0 ? 4 :
                    1;

            double drives = (-gamepad1.left_stick_y*0.5)/Speed;
            double strafe = (-gamepad1.left_stick_x*0.5)/Speed;
            double turn = (-gamepad1.right_stick_x*0.5)/Speed;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(drives, strafe), turn));

            if (gamepad1.dpad_down) {
                motorBR2.setPower(-0.8);
                motorBR1.setPower(-0.8);
            } else if (gamepad1.dpad_up) {
                motorBR2.setPower(1);
                motorBR1.setPower(1);
            } else if (gamepad2.dpad_left) {
                armR.setPower(1);
                armL.setPower(1);
            } else if (gamepad2.dpad_right) {
                armR.setPower(-1);
                armL.setPower(-1);
            } else if (gamepad2.square) {
                servoUpperArm.setPosition(SERVO_UPPER_ARM_IN);
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_OPEN);
                while (gamepad2.square) {
                    servoLowerClaw.setPosition(SERVO_CLAW_OPEN);
                    servoLowerArm.setPosition(SERVO_LOWER_ARM_MEDIUM_OUT);
                    armR.setPower(-0.75);
                    armL.setPower(-0.75);
                }
            } else if (gamepad2.cross) {
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_OPEN);
                servoUpperArm.setPosition(1-0.125);
                servoUpperArmRol.setPosition(0.51);
                servoLowerArm.setPosition(SERVO_LOWER_ARM_OUT);
                waitFor(100);
                servoLowerClaw.setPosition(SERVO_CLAW_CLOSED);
                waitFor(320);
                servoLowerArm.setPosition(SERVO_LOWER_ARM_IN);
                servoRol.setPosition(0);
                toClose = true;
            } else if (toClose) {
                armR.setPower(0.75);
                armL.setPower(0.75);
                if(!isNotPressed){
                    toClose = false;
                }
            } else if (gamepad2.dpad_up) {
                servoUpperArm.setPosition(SERVO_UPPER_ARM_OUT);
            } else if (gamepad2.triangle) {
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_OPEN);
            } else if (gamepad2.circle) {
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_CLOSED);
            } else if (gamepad2.left_bumper) {
                servoUpperArmRol.setPosition(0.513);
            } else if (gamepad2.right_bumper) {
                servoUpperArmRol.setPosition(0.49);
            } else if(gamepad2.dpad_down){
                servoLowerArm.setPosition(SERVO_LOWER_ARM_IN+0.2);
                waitFor(200);
                servoUpperArm.setPosition(0);
            } else if (gamepad2.left_trigger != 0) {
                servoLowerClaw.setPosition(SERVO_CLAW_OPEN);
            } else if(gamepad2.right_trigger != 0){
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_CLOSED);
                waitFor(200);
                servoLowerClaw.setPosition(SERVO_CLAW_OPEN);
            } else if (gamepad2.left_stick_button) {
                servoUpperArm.setPosition(0.417);
            } else if(gamepad1.square){
                servoRol.setPosition(0.2);
            } else if (gamepad1.circle) {
                servoRol.setPosition(0.4);
            } else if (gamepad1.triangle) {
                servoRol.setPosition(0);
            }
            motorBR2.setPower(0.02);
            motorBR1.setPower(0.02);
            armL.setPower(0);
            armR.setPower(0);

        }
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

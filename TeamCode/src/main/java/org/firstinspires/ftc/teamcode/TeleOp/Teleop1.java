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
import org.firstinspires.ftc.teamcode.MecanumDrive;
import static org.firstinspires.ftc.teamcode.autoversion2.robotData.*;


@TeleOp(name="Test")
@Config
public class Teleop1 extends LinearOpMode{
    public boolean isRunning = false;
    @Override
    public void runOpMode() {
        DcMotor motorRB, motorRF, motorLF, motorLB, motorBR1, motorBR2, armL, armR;
        Servo servoLowerArm, servoLowerClaw, servoRol;
        DigitalChannel limitBtn;

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
        limitBtn = hardwareMap.get(DigitalChannel.class, "limitBtn");

        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            boolean isNotPressed = limitBtn.getState();
            telemetry.addData("Brat extins", isNotPressed);
            telemetry.update();

            double drives = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(drives, strafe), turn));

            if (gamepad1.dpad_down) {
                motorBR2.setPower(-1);
                motorBR1.setPower(-1);
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
                while (gamepad2.square) {
                    servoLowerClaw.setPosition(SERVO_CLAW_OPEN);
                    servoLowerArm.setPosition(SERVO_LOWER_ARM_MEDIUM_OUT);
                    armR.setPower(-1);
                    armL.setPower(-1);
                }
            } else if (gamepad2.circle) {
                servoLowerClaw.setPosition(SERVO_CLAW_OPEN);
            } else if (gamepad2.cross) {
                servoLowerArm.setPosition(SERVO_LOWER_ARM_OUT);
                servoLowerClaw.setPosition(SERVO_CLAW_CLOSED);
                if (gamepad2.cross && !isRunning) {
                    isRunning = true;
                } else if (!isNotPressed) {
                    isRunning = false;
                    servoLowerArm.setPosition(SERVO_LOWER_ARM_IN);
                } else if (isRunning) {
                    armR.setPower(1);
                    armL.setPower(1);
                }
            } else if (gamepad2.dpad_up) {
                servoRol.setPosition(0);
            }


            motorBR2.setPower(0);
            motorBR1.setPower(0);
            armL.setPower(0);
            armR.setPower(0);

        }

    }
}

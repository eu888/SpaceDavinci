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
    /**
     * Init the elapseTime
     */
    ElapsedTime elapsedTime = new ElapsedTime();
    double Speed=0.6;
    /**
     * Init the motors, servos and limit button
     */
    DigitalChannel limitBtn;
    Servo servoLowerArm, servoLowerClaw, servoRol, servoUpperArm, servoUpperArmRol, servoUpperClaw;
    DcMotor motorRB, motorRF, motorLF, motorLB, motorBR1, motorBR2,armL, armR;
    final double POS1 = 0.64;
    final double POS2 = 0.36;
    final double POS3 = 0.5;


    @Override
    public void runOpMode() {
        /// Here we define in witch ports we connect our devices
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
        /// Here we setup the modes of the motors
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limitBtn.setMode(DigitalChannel.Mode.INPUT);
        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
        /// Here we setup the motors to brake if the power is zero
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /// Here we setup the roadrunner variables
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        boolean toClose = false;
        boolean toChange = false;
        int rol = 0;
        double pos = 0.5;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            /// Here we setup some telemetry and som other values
            boolean isNotPressed = limitBtn.getState();
            telemetry.addData("Brat extins", isNotPressed);
            telemetry.addData("Brat ticks", motorBR2.getCurrentPosition());
            telemetry.update();
            /// Values for control using roadrunner
            double drives = -gamepad2.left_stick_y*Speed;
            double strafe = -gamepad2.left_stick_x*Speed;
            double turn = -gamepad2.right_stick_x*Speed;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(drives, strafe), turn));
            /// The controls of the main lift
            if (gamepad2.dpad_left) {
                armR.setPower(1);
                armL.setPower(1);
            } else if (gamepad2.dpad_right) {
                armR.setPower(-1);
                armL.setPower(-1);
            }
            /// The controls for pick up samples and others
            else if (gamepad2.square) {
                servoUpperArm.setPosition(SERVO_UPPER_ARM_IN-0.37);
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_OPEN);
                while (gamepad2.square) {
                    servoLowerClaw.setPosition(SERVO_CLAW_OPEN);
                    servoLowerArm.setPosition(SERVO_LOWER_ARM_MEDIUM_OUT);
                    armR.setPower(-0.75);
                    armL.setPower(-0.75);
                }
            } else if (gamepad2.cross) {
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_OPEN);
                servoUpperArm.setPosition(SERVO_UPPER_ARM_IN-0.26);
                servoUpperArmRol.setPosition(0.5);
                servoLowerArm.setPosition(SERVO_LOWER_ARM_OUT);
                waitFor(100);
                servoLowerClaw.setPosition(SERVO_CLAW_CLOSED);
                waitFor(320);
                servoLowerArm.setPosition(SERVO_LOWER_ARM_IN);
                servoRol.setPosition(POS3);
                toClose = true;
            } else if (toClose) {
                armR.setPower(0.75);
                armL.setPower(0.75);
                if(!isNotPressed){
                    toClose = false;
                    toChange = true;
                }
                if (toChange){
                    waitFor(450);
                    servoUpperClaw.setPosition(SERVO_UPPER_CLAW_CLOSED);
                    waitFor(350);
                    servoLowerClaw.setPosition(SERVO_CLAW_OPEN);
                }
            } else if(gamepad2.dpad_up){
                motorBR1.setPower(1);
                motorBR2.setPower(1);
            } else if(gamepad2.dpad_down){
                motorBR1.setPower(-1);
                motorBR2.setPower(-1);
            } else if(gamepad2.circle){
                servoUpperArm.setPosition(0.3);
                servoUpperArmRol.setPosition(0.47);
                waitFor(2000);
                servoUpperClaw.setPosition(SERVO_UPPER_CLAW_OPEN);
            } else if(gamepad1.circle){
                servoLowerClaw.setPosition(SERVO_CLAW_CLOSED);
                servoRol.setPosition(0.5);
            } else if (gamepad2.triangle) {
                if (rol == 0 && gamepad2.triangle) {
                    pos = POS1;
                    rol = 1;
                } else if (rol == 1 && gamepad2.triangle) {
                    pos = POS2;
                    rol = 2;
                } else if (rol == 2 && gamepad2.triangle) {
                    pos = POS3;
                    rol = 0;
                }
                servoRol.setPosition(pos);
            }


            motorBR1.setPower(0.02);
            motorBR2.setPower(0.02);
            armR.setPower(0);
            armL.setPower(0);
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

    /**
     * detects if a number is negative
     * @param number your number to check
     * @return if it is or not negative
     */
    @Deprecated
    public static boolean isNegative(int number){
        return number < 0;
    }
}

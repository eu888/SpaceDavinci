package org.firstinspires.ftc.teamcode.TeleOp;

//Standard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

//Self made libs
import static org.firstinspires.ftc.teamcode.constants.constants.*;
import static org.firstinspires.ftc.teamcode.libs.TeleOpLibs.*;

//For RoadRunner
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name = "TeleOp", group = "TeleOp")
@Config
public class TeleOp1 extends LinearOpMode {
    DcMotor motorIntakeA, motorIntakeB;
    DcMotorEx motorLauncherA, motorLauncherB;
    DcMotor motorUpperLeft, motorUpperRight, motorLowerLeft, motorLowerRight;
    Servo servoBallLift;

    Pose2d beginPose = new Pose2d(0, 0, 0);
    FtcDashboard dashboard;
    PIDFCoefficients pidfCoefficients;
    boolean activated;

    @Override
    public void runOpMode(){
        dashboard = FtcDashboard.getInstance();

        //PIDF coefficients
        pidfCoefficients = new PIDFCoefficients(P, 0, D, F);

        //Drive Base
        motorUpperLeft = hardwareMap.get(DcMotor.class,"motorUpperLeft");
        motorUpperRight = hardwareMap.get(DcMotor.class,"motorUpperRight");
        motorLowerLeft = hardwareMap.get(DcMotor.class, "motorLowerLeft");
        motorLowerRight = hardwareMap.get(DcMotor.class, "motorLowerRight");

        //AUX motors
        motorLauncherA = hardwareMap.get(DcMotorEx.class, "motorLauncherA");
        motorLauncherB = hardwareMap.get(DcMotorEx.class, "motorLauncherB");
        motorIntakeA = hardwareMap.get(DcMotor.class, "motorIntakeA");
        motorIntakeB = hardwareMap.get(DcMotor.class, "motorIntakeB");

        motorIntakeA.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLauncherA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        motorLauncherB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //Servos
        servoBallLift = hardwareMap.get(Servo.class, "servoBallLift");

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        activated=false;

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()){
            double motorPower = 0.8;

            if(!activated){
                servoBallLift.setPosition(servoBallLiftDown);
                activated=true;
            }

            if (gamepad1.square) {
                new Thread(() ->{
                    try {
                        servoBallLift.setPosition(servoBallLiftUp);
                        Thread.sleep(1500);
                        servoBallLift.setPosition(servoBallLiftDown);
                    }
                    catch (InterruptedException e){
                        Thread.currentThread().interrupt();
                    }
                }).start();

            }
            if (gamepad1.right_bumper) {
                 motorPower = 0.6;
            }
            if (gamepad1.dpad_down) {
                motorIntakeA.setPower(-intakePower);
                motorIntakeB.setPower(-intakePower);
            }
            if (gamepad1.dpad_up) {
                motorIntakeA.setPower(intakePower);
                motorIntakeB.setPower(intakePower);
            }
            if (gamepad1.dpad_right) {
                motorIntakeA.setPower(0);
                motorIntakeB.setPower(0);
            }
            if  (gamepad1.dpad_left) {
                servoBallLift.setPosition(servoBallLiftDown);
            }
            if (gamepad1.circle) {
                motorLauncherA.setVelocity(launchVelocityOn);
                motorLauncherB.setVelocity(launchVelocityOn);
            }

            telemetry.addData("Battery voltage", getBatteryVoltage(hardwareMap));

            double drives = -gamepad1.left_stick_y* motorPower;
            double strafe = -gamepad1.left_stick_x* motorPower;
            double turn = -gamepad1.right_stick_x* motorPower;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(drives, strafe), turn));
            telemetry.addData("vel A: ", motorLauncherA.getVelocity());
            telemetry.addData("vel B: ", motorLauncherB.getVelocity());
            telemetry.update();
        }
    }


}

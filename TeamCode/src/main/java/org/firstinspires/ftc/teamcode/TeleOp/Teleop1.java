package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.teamcode.autoversion2.lib.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Test")
@Config
public class Teleop1 extends LinearOpMode{
    @Override
    public void runOpMode(){
        DcMotor motorRB, motorRF, motorLF, motorLB;

        motorRB = hardwareMap.get(DcMotor.class, "mrb");
        motorRF = hardwareMap.get(DcMotor.class, "mrf");
        motorLF = hardwareMap.get(DcMotor.class, "mlf");
        motorLB = hardwareMap.get(DcMotor.class, "mlb");

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double lx = gamepad1.left_stick_x, ly = -gamepad1.right_stick_x, rx = gamepad1.left_stick_y;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = lx*Math.cos(-botHeading)-ly*Math.sin(-botHeading);
            rotX*=1.1;
            double rotY = lx*Math.sin(-botHeading)-ly*Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY)+Math.abs(rotX)+Math.abs(rx),1);
            double motorLBSpeed = (rotY-rotX+rx)/denominator;
            double motorLFSpeed = (rotY+rotX+rx)/denominator;
            double motorRFSpeed = (rotY-rotX-rx)/denominator;
            double motorRBSpeed = (rotY+rotX-rx)/denominator;

            motorLB.setPower(motorLBSpeed);
            motorLF.setPower(motorLFSpeed);
            motorRF.setPower(motorRFSpeed);
            motorRB.setPower(motorRBSpeed);

        }
    }
}

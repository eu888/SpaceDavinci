package org.firstinspires.ftc.teamcode.autoversion2;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;

import java.util.Objects;

//import javax.annotation.Nullable;

public class lib {
    public static void runMotors(@NonNull DcMotor[] motors, double power){
        for(DcMotor motor : motors){
            motor.setPower(power);
        }
    }

    public static void stopMotors(@NonNull DcMotor[] motors){
        for(DcMotor motor : motors){
            motor.setPower(0.0);
        }
    }

    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public static void move(@NonNull DcMotor[] motors1, @NonNull DcMotor[] motors2, double power, String direction){
        if(Objects.equals(direction, "left")){
            runMotors(motors1, -power);
            runMotors(motors2, power);
        } else if(Objects.equals(direction, "right")){
            runMotors(motors1, power);
            runMotors(motors2, -power);
        }else if(Objects.equals(direction, "forward")){
            runMotors(motors1, power);
            runMotors(motors2, power);
        }else if(Objects.equals(direction, "backward")){
            runMotors(motors1, -power);
            runMotors(motors2, -power);
        }
    }
    public void spin(@NonNull DcMotor[] motors , double power, boolean spinClockWise){
        if(spinClockWise){
            for(int i = 0;i < 2;i++){
                for(DcMotor motor : motors){
                    motor.setPower(power);
                }
            }
            for(DcMotor motor : motors){
                motor.setPower(-power);
            }
        }
        for(int i = 0;i < 2;i++){
            for(DcMotor motor : motors){
                motor.setPower(-power);
            }
        }
        for(DcMotor motor : motors){
            motor.setPower(+power);
        }
    }

    public static void runByEncoder(@NonNull DcMotor motor,int target){
        motor.setPower(0.5);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void imuInit(@NonNull IMU imu, IMU.Parameters parameters){
        imu.initialize(parameters);
    }

}


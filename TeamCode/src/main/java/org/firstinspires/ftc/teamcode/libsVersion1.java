package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
public class libsVersion1 {
    public static boolean isApproximatelyEqual(double a, double b, double tolerance){
        return Math.abs(a - b) <= tolerance;
    }
    public static void go_to(@NonNull DcMotor motor, int a){
        motor.setTargetPosition(a);
        motor.setPower(0.8);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public static void encoder_reset(@NonNull DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void stop(@NonNull DcMotor motor){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void runMotor(DcMotor motor, int a) {
        Thread thread = new Thread(() -> go_to(motor, a));
        thread.start();
        try {
            thread.join();
        }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
    public static void  sleep(int time){
        try {
            Thread.sleep(time);
        }catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
}

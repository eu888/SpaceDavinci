package org.firstinspires.ftc.teamcode.libs;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class TeleOpLibs extends Thread{
    /**
     * This is a wait function that uses elapseTime
     * @param opMode used to get opModeIsActive() to check if the TeleOp is active.
     * @param elapsedTime will use an {@link ElapsedTime} created in your TeleOp.
     * @param milliseconds The time to wait in milliseconds.
     */
    public static void waitFor(@NonNull LinearOpMode opMode, @NonNull ElapsedTime elapsedTime, int milliseconds){
        try {
            elapsedTime.reset();
            while(opMode.opModeIsActive() && elapsedTime.milliseconds() < milliseconds){
                opMode.telemetry.addData("Wait", elapsedTime.milliseconds());
                opMode.telemetry.update();
                opMode.idle();
            }
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Gets battery voltage
     * @param hardwareMap used to get the voltage sensor.
     * @return the battery voltage.
     */
    public static double getBatteryVoltage(@NonNull HardwareMap hardwareMap){
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor voltageSensor : hardwareMap.voltageSensor){
            double voltage = voltageSensor.getVoltage();
            if (voltage > 0){
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}

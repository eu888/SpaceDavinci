package org.firstinspires.ftc.teamcode.libs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class TeleOpLibs {
    /**
     * This is a wait function that uses elapseTime
     * @param elapsedTime will use an {@link ElapsedTime} created in your TeleOp.
     * @param milliseconds The time to wait in milliseconds.
     * @param isActive use opModeIsActive() to check if the TeleOp is active.
     */
    public static void waitFor(@NonNull ElapsedTime elapsedTime, int milliseconds, boolean isActive){
        elapsedTime.reset();
        while(isActive && elapsedTime.milliseconds() < milliseconds){
            telemetry.addData("Wait", elapsedTime.milliseconds());
            telemetry.update();
        }
    }
}

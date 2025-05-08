package org.firstinspires.ftc.teamcode.libs;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorSetup{
    private DcMotor[] motors;
    private final String[] names;
    private final HardwareMap hardwareMap;
    public MotorSetup(@NonNull HardwareMap hardwareMap, @NonNull String[] names) {
        this.hardwareMap = hardwareMap;
        this.names = names;
        this.motors = new DcMotor[names.length];
    }

    public void Setup(){
        for (int i = 0; i < names.length; i++){
            DcMotor motor = hardwareMap.get(DcMotor.class, names[i]);
            motors[i] = motor;
        }
    }
}

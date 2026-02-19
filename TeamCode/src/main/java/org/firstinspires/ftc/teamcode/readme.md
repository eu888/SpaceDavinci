# Code Documentation
---
## 1. Configuration

1. Hardware Configuration
    ```java
    // Drive Motors (4x DcMotor)
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;   
   
   //Self made libs
    import static org.firstinspires.ftc.teamcode.constants.constants.*;
   
    DcMotor motorUpperLeft, motorUpperRight, motorLowerLeft, motorLowerRight;

    // Auxiliary Motors
    DcMotorEx motorLauncherA, motorLauncherB;  // High-speed drive
    DcMotor motorIntakeA, motorIntakeB;        // Reversible intake

    // Servo Control
    Servo servoBallLift;
   
   PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, D, F);

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
    ```
package org.firstinspires.ftc.teamcode.autoversion2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.android.OpenCVLoader;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.autoversion2.yellowPipeline.*;

import java.util.List;

@Autonomous(name = "Yellow Detect Test")
public class detectYellowSpeciments extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    OpenCvCamera webcam;
    yellowPipeline pipeline;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new yellowPipeline(640, 480, 60, 0);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
        dashboard.startCameraStream(webcam, 30);

        waitForStart();

        while(opModeIsActive()){
            List<MatOfPoint> detectContours = pipeline.getContours();
            telemetry.addData("Detected Objects", detectContours.size());
            telemetry.update();
        }
    }
}

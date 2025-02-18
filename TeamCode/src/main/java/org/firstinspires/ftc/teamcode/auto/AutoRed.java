package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.autoversion2.lib.*;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


@Autonomous(name="Auto3.2.4", group = "Red")
public class AutoRed extends OpMode {
    public int closestObjectId = -1;
    public int relativeX;
    public int relativeY;
    Mat hsv = new Mat();
    Mat mask1 = new Mat();
    Mat mask2 = new Mat();
    Mat combinedMask = new Mat();
    Mat hierarchy = new Mat();
    private OpenCvCamera webcam;
    private List<TrackedObject> trackedObjects = new ArrayList<>();
    private static final double MIN_CONTOUR_AREA = 500.0;
    private static final double MAX_DISTANCE = 50.0;

    DcMotor motorLB,motorLF,motorRB,motorRF,motorB, motorE;
    DcMotor[] group1 = {motorLF, motorRB};
    DcMotor[] group2 = {motorLB, motorRF};
    DcMotor[] all = {motorRF, motorRB, motorLB, motorLF};
    Servo sr1, sr2;
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new YellowObjectDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        motorSetup();
    }

    @Override
    public void loop() {

    }

    public List<TrackedObject> getDetectedYellowObjects() {
        return trackedObjects;
    }

    class YellowObjectDetectionPipeline extends OpenCvPipeline   {
        @Override
        public Mat processFrame(Mat input) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow1 = new Scalar(20, 100, 100); 
            Scalar upperYellow1 = new Scalar(30, 255, 255); 
            Scalar lowerYellow2 = new Scalar(10, 100, 100); 
            Scalar upperYellow2 = new Scalar(20, 255, 255); 

            
            Core.inRange(hsv, lowerYellow1, upperYellow1, mask1);
            Core.inRange(hsv, lowerYellow2, upperYellow2, mask2);
            Core.add(mask1, mask2, combinedMask);

            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<TrackedObject> currentFrameObjects = new ArrayList<>();

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area >= MIN_CONTOUR_AREA) {
                    Rect rect = Imgproc.boundingRect(contour);
                    if (isNearbyColorDifferent(input, rect)) {
                        currentFrameObjects.add(new TrackedObject(rect));
                    }
                }
            }

            trackedObjects = updateTrackedObjects(trackedObjects, currentFrameObjects);

            TrackedObject nearestObject = null;
            double minDistance = Double.MAX_VALUE;
            int imageCenterX = input.cols() / 2;
            int imageCenterY = input.rows() / 2;

            for (TrackedObject obj : trackedObjects) {
                Rect rect = obj.getRect();
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 1);

                Imgproc.putText(input, "ID: " + obj.getId(), new Point(rect.x, rect.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.35, new Scalar(0, 255, 0), 2);

                int objectCenterX = rect.x + rect.width / 2;
                int objectCenterY = rect.y + rect.height / 2;

                relativeX = objectCenterX - imageCenterX;
                relativeY = objectCenterY - imageCenterY;

                goToCenter(28);

                double distance = Math.sqrt(Math.pow(objectCenterX - imageCenterX, 2) + Math.pow(objectCenterY - imageCenterY, 2));

                if (distance < minDistance) {
                    minDistance = distance;
                    closestObjectId = obj.getId();
                    nearestObject = obj;
                }

                String positionText = String.format("Rel Pos: (%d, %d)", relativeX, relativeY);
                Imgproc.putText(input, positionText + " ID: " + obj.getId(), new Point(rect.x, rect.y - 25), Imgproc.FONT_HERSHEY_SIMPLEX, 0.35, new Scalar(255, 0, 0), 2);
            }

            if (nearestObject != null) {
                Rect nearestRect = nearestObject.getRect();
                telemetry.addData("Nearest Object Position (Relative): (", relativeX + ", " + relativeY + ")");
                telemetry.addData("Closest Object ID: ", closestObjectId);
                telemetry.update();
            }

            return input;
        }

        private boolean isNearbyColorDifferent(@NonNull Mat input, @NonNull Rect rect) {
            int xStart = Math.max(0, rect.x - 10);
            int yStart = Math.max(0, rect.y - 10);
            int xEnd = Math.min(input.cols(), rect.x + rect.width + 10);
            int yEnd = Math.min(input.rows(), rect.y + rect.height + 10);

            for (int y = yStart; y < yEnd; y++) {
                for (int x = xStart; x < xEnd; x++) {
                    double[] pixel = input.get(y, x);
                    if (isNotYellow(pixel)) {
                        return true;
                    }
                }
            }
            return false;
        }

        private boolean isNotYellow(@NonNull double[] pixel) {
            double r = pixel[0];
            double g = pixel[1];
            double b = pixel[2];

            return !(r > 100 && g > 100 && b < 100);
        }

        @NonNull
        private List<TrackedObject> updateTrackedObjects(List<TrackedObject> previous, @NonNull List<TrackedObject> current) {
            Map<Integer, Boolean> usedIds = new HashMap<>();
            List<TrackedObject> updated = new ArrayList<>();

            for (TrackedObject currentObj : current) {
                TrackedObject closestPrevious = null;
                double closestDistance = Double.MAX_VALUE;

                for (TrackedObject prevObj : previous) {
                    if (!usedIds.getOrDefault(prevObj.getId(), false)) {
                        double distance = currentObj.distanceTo(prevObj);
                        if (distance < closestDistance && distance < MAX_DISTANCE) {
                            closestDistance = distance;
                            closestPrevious = prevObj;
                        }
                    }
                }

                if (closestPrevious != null) {
                    currentObj.setId(closestPrevious.getId());
                    usedIds.put(closestPrevious.getId(), true);
                } else {
                    currentObj.setId(previous.size() + updated.size());
                }
                updated.add(currentObj);
            }
            return updated;
        }


    }

    class TrackedObject {
        private Rect rect;
        private int id;

        public TrackedObject(Rect rect) {
            this.rect = rect;
            this.id = -1;
        }

        public Rect getRect() {
            return rect;
        }

        public int getId() {
            return id;
        }

        public void setId(int id) {
            this.id = id;
        }

        public double distanceTo(@NonNull TrackedObject other) {
            Point center1 = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
            Point center2 = new Point(other.getRect().x + other.getRect().width / 2.0, other.getRect().y + other.getRect().height / 2.0);
            return Math.sqrt(Math.pow(center1.x - center2.x, 2) + Math.pow(center1.y - center2.y, 2));
        }
    }

    public void motorSetup(){
        motorRB = hardwareMap.get(DcMotor.class, "mrb");
        motorRF = hardwareMap.get(DcMotor.class, "mrf");
        motorLF = hardwareMap.get(DcMotor.class, "mlf");
        motorLB = hardwareMap.get(DcMotor.class, "mlb");
        motorB = hardwareMap.get(DcMotor.class, "mb");
        motorE = hardwareMap.get(DcMotor.class, "me");

        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        sr1 = hardwareMap.get(Servo.class, "sr1");
        sr2 = hardwareMap.get(Servo.class, "sr2");

    }



    public void telemetry(){
        telemetry.addData("Motor RB", motorRB.getCurrentPosition());
        telemetry.addData("Motor Rf", motorRF.getCurrentPosition());
        telemetry.addData("Motor LF", motorLF.getCurrentPosition());
        telemetry.addData("Motor LB", motorLB.getCurrentPosition());
        telemetry.addData("Motor B", motorB.getCurrentPosition());
        telemetry.addData("Motor E", motorE.getCurrentPosition());
        telemetry.addData("Servo sr1", sr1.getPosition());
        telemetry.addData("Servo sr2", sr2.getPosition());
    }

    public void goToCenter(int centerX){
        if(relativeX < centerX){
            move(group1, group2, 0.5, "left");
        } else if (relativeX > centerX) {
            move(group1, group2, 0.5, "right");
        } else {
            stopMotors(all);
        }
    }

}
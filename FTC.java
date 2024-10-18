package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.CameraMonitorViewId;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class IntoTheDeepAutoOptimized extends LinearOpMode {

    private DcMotor leftMotor, rightMotor, armMotor;
    private Servo grabServo;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Gyroscope gyroscope;
    private OpenCvCamera camera;
    private ZoneDetectionPipeline pipeline;

    private double kp = 0.1, ki = 0.01, kd = 0.05;
    private double targetPosition = 0;

    private static final double MAX_POWER = 1.0, MIN_POWER = 0.1;
    private static final long SAFETY_TIMEOUT = 6000;

    private ElapsedTime elapsedTime;
    private String allianceColor = "unknown";
    private boolean isObstacleDetected = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addData("Status", "Initialized and waiting for start...");
        telemetry.update();
        waitForStart();
        elapsedTime.reset();
        detectAllianceColor();
        executeAutonomous();
        adjustStrategy();
        safetyCheck();
        stopAllMotors();
    }

    public void initializeHardware() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        grabServo = hardwareMap.get(Servo.class, "grab_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        gyroscope = hardwareMap.get(Gyroscope.class, "gyro");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elapsedTime = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvCamera.CameraDirection.BACK, 0);
        pipeline = new ZoneDetectionPipeline();
        camera.setPipeline(pipeline);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    public void detectAllianceColor() {
        if (colorSensor.blue() > colorSensor.red()) {
            allianceColor = "blue";
        } else if (colorSensor.red() > colorSensor.blue()) {
            allianceColor = "red";
        }
        telemetry.addData("Alliance Color Detected", allianceColor);
        telemetry.update();
    }

    public void executeAutonomous() throws InterruptedException {
        telemetry.addData("Executing Autonomous", "");
        telemetry.update();
        moveForwardPID(2000);
        moveArm(1.0, 1000);
        grabSample();

        String detectedZone = pipeline.getDetectedZone();
        if (detectedZone.equals("ZoneA")) {
            moveToZone("ZoneA");
        } else if (detectedZone.equals("ZoneB")) {
            moveToZone("ZoneB");
        } else if (detectedZone.equals("ZoneC")) {
            moveToZone("ZoneC");
        }

        releaseSample();
        navigateToAscentZone();
    }

    public void moveForwardPID(long timeInMillis) throws InterruptedException {
        double error = targetPosition - leftMotor.getCurrentPosition();
        double lastError = error;
        double integral = 0;
        double derivative;
        double power;

        elapsedTime.reset();
        while (elapsedTime.milliseconds() < timeInMillis) {
            error = targetPosition - leftMotor.getCurrentPosition();
            integral += error;
            derivative = error - lastError;
            power = kp * error + ki * integral + kd * derivative;
            power = Range.clip(power, MIN_POWER, MAX_POWER);

            double gyroReading = gyroscope.getAngularOrientation().firstAngle;
            if (Math.abs(gyroReading) > 5) {
                leftMotor.setPower(power + (gyroReading * 0.01));
                rightMotor.setPower(power - (gyroReading * 0.01));
            } else {
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
            lastError = error;

            telemetry.addData("PID Power", power);
            telemetry.addData("Current Position", leftMotor.getCurrentPosition());
            telemetry.addData("Gyro Reading", gyroReading);
            telemetry.update();

            sleep(10);
        }

        stopAllMotors();
    }

    public void moveArm(double power, long timeInMillis) throws InterruptedException {
        armMotor.setPower(power);
        sleep(timeInMillis);
        armMotor.setPower(0);
    }

    public void grabSample() {
        grabServo.setPosition(0.5);
        sleep(500);
    }

    public void releaseSample() {
        grabServo.setPosition(1.0);
        sleep(500);
    }

    public void moveToZone(String zone) throws InterruptedException {
        if (zone.equals("ZoneA")) {
            moveForwardPID(3000);
        } else if (zone.equals("ZoneB")) {
            moveForwardPID(3000);
            turnLeft(0.5, 1000);
        } else if (zone.equals("ZoneC")) {
            moveForwardPID(3000);
            turnRight(0.5, 1000);
        }
    }

    public void turnLeft(double power, long timeInMillis) throws InterruptedException {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
        sleep(timeInMillis);
        stopAllMotors();
    }

    public void turnRight(double power, long timeInMillis) throws InterruptedException {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
        sleep(timeInMillis);
        stopAllMotors();
    }

    public void safetyCheck() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        if (distance < 30) {
            telemetry.addData("Obstacle Detected", "Shutting down motors");
            telemetry.update();
            stopAllMotors();
            isObstacleDetected = true;
        }
    }

    public void adjustStrategy() {
        if (elapsedTime.seconds() > 90) {
            telemetry.addData("Strategy Adjustment", "Moving to endgame");
            telemetry.update();
            navigateToAscentZone();
        }
    }

    public void navigateToAscentZone() throws InterruptedException {
        telemetry.addData("Navigating to Ascent Zone", "");
        telemetry.update();
        moveForwardPID(5000);
    }

    public void stopAllMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
    }

    public static class ZoneDetectionPipeline extends OpenCvPipeline {
        private String detectedZone = "unknown";

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvImage = new Mat();
            Mat mask = new Mat();

            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            Scalar lowerZoneA = new Scalar(100, 150, 0);
            Scalar upperZoneA = new Scalar(140, 255, 255);

            Scalar lowerZoneB = new Scalar(20, 100, 100);
            Scalar upperZoneB = new Scalar(30, 255, 255);

            Scalar lowerZoneC = new Scalar(0, 100, 100);
            Scalar upperZoneC = new Scalar(10, 255, 255);

            Core.inRange(hsvImage, lowerZoneA, upperZoneA, mask);
            List<MatOfPoint> contoursA = new ArrayList<>();
            Imgproc.findContours(mask, contoursA, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if (!contoursA.isEmpty()) {
                detectedZone = "ZoneA";
                Imgproc.drawContours(input, contoursA, -1, new Scalar(0, 255, 0), 2);
            }

            mask.release();
            Core.inRange(hsvImage, lowerZoneB, upperZoneB, mask);
            List<MatOfPoint> contoursB = new ArrayList<>();
            Imgproc.findContours(mask, contoursB, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if (!contoursB.isEmpty()) {
                detectedZone = "ZoneB";
                Imgproc.drawContours(input, contoursB, -1, new Scalar(0, 255, 0), 2);
            }

            mask.release();
            Core.inRange(hsvImage, lowerZoneC, upperZoneC, mask);
            List<MatOfPoint> contoursC = new ArrayList<>();
            Imgproc.findContours(mask, contoursC, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if (!contoursC.isEmpty()) {
                detectedZone = "ZoneC";
                Imgproc.drawContours(input, contoursC, -1, new Scalar(0, 255, 0), 2);
            }

            hsvImage.release();
            mask.release();

            return input;
        }

        public String getDetectedZone() {
            return detectedZone;
        }
    }
}

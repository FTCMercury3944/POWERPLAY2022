package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 *  Autonomous mode system:
 *
 *  1. Detect parking zone
 *     Take picture of parking zone
 *     Determine image from signal sleeve
 *  2. Drop off preloaded cone
 *     Move to closest 3-point, 4-point, or 5-point
 *     Stop and place cone
 *  LOOP START
 *      3. Grab cone
 *         Move to cone stack
 *         Grab cone from top of stack
 *      4. Drop off cone
 *         Move to closest 4-point or 5-point
 *         Stop and place cone
 *  LOOP END
 *  5. Park in correct zone
 *     Get location of previously-evaluated parking zone
 *     Move to that location
 */

@Autonomous(name="Auton")
public class Auton extends LinearOpMode
{
    private AprilTagDetection tagOfInterest;
    private OpenCvCamera camera;

    // Variables for color detection
    private int yellowDetections = 0;
    private boolean isYellowBuffer = false;
    private boolean poleEnded = false;
    private float[] hsvValues = new float[3];

    // Side length of square tag in meters
    private static final double TAG_SIZE = 0.166;

    // Brightness gain for color sensor
    private static final float GAIN = 20;

    // Lens intrinsics for Logitech C920 webcam
    // (FX, FY) is focal length in pixels
    // (CX, CY) is principal point in pixels
    private static final double FX = 1428.470;
    private static final double FY = 1428.470;
    private static final double CX = 630.909;
    private static final double CY = 358.721;

    // Tag IDs 1, 2, and 3 from the 36H11 family
    private static final int LEFT = 1;
    private static final int MIDDLE = 2;
    private static final int RIGHT = 3;

    private final ElapsedTime RUNTIME = new ElapsedTime();
    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.5;
    private static final double DETECTION_SPEED = 0.1;

    @Override
    public void runOpMode()
    {
        // Setup webcam
        int cameraID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraID);

        // Begin detection pipeline
        DetectionPipeline pipeline = new DetectionPipeline(TAG_SIZE, FX, FY, CX, CY);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                // Nonexistent error handling, required for async
            }
        });

        // Initialize color sensor gain
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(GAIN);

        // Wait for game to begin
        waitForStart();

        // Detect AprilTag
        /*
        boolean tagFound = false;
        while (isStarted() && !tagFound && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if (currentDetections.size() != 0)
            {
                for (AprilTagDetection tag : currentDetections)
                {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }

            sleep(20);
        }
         */

        /*
        // Move for auton
        if (tagOfInterest == null || tagOfInterest.id == LEFT)
        {
            // Left or default
            driveSideways(-14);
            driveForward(16);
        }
        else if (tagOfInterest.id == MIDDLE)
        {
            // Middle
            driveForward(22.5);
        }
        else
        {
            // Right
            driveSideways(14);
            driveForward(16);
        }
         */
    }

    /**
     *  Calls encoderDrive() to move forward a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel forward;
     *                  positive is forward
     */
    public void driveForward(double distance)
    {
        encoderDrive(DRIVE_SPEED, distance, 5, false, false);
    }

    /**
     *  Calls encoderDrive() to move sideways a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right
     */
    public void driveSideways(double distance)
    {
        encoderDrive(DRIVE_SPEED, distance, 5, true, false);
    }

    /**
     *  Calls encoderDrive() to move sideways until the center of a pole is reached.
     *  Moves right.
     */
    public void detectPole()
    {
        encoderDrive(DETECTION_SPEED, 100, 10, true, true); // second parameter overshoots for safety
    }

    /**
     *  Uses encoders to drive a specified distance in a specified direction at a specified speed.
     *
     *  @param voltage The power (-1.0 to 1.0) to send to each motor
     *  @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right or forward (depending on sidewaysSwitch)
     *  @param timeout The time, in seconds, that a robot should have completed the
     *                 routine by; if it hasn't, abort the routine
     *  @param sidewaysSwitch If true, move left/right; if false, more forward/backward
     *  @param toScanPole If true, move slowly and scan for a pole
     */
    public void encoderDrive(double voltage, double distance, double timeout, boolean sidewaysSwitch, boolean toScanPole)
    {
        // Calculate distance
        int counts = (int)(-distance * COUNTS_PER_INCH);

        // Determine directional variables
        int directionalCounts = counts;
        double directionalVoltage = Math.abs(voltage);
        if (sidewaysSwitch)
        {
            directionalCounts = -counts;
            directionalVoltage = -voltage;
        }

        // Initialize the DC motors for each wheel
        // Declare op-mode members
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse right side
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Determine new target position, and pass to motor controller
        int newLeftFrontTarget = leftFront.getCurrentPosition() + counts;
        int newRightFrontTarget = rightFront.getCurrentPosition() + directionalCounts;
        int newLeftRearTarget = leftRear.getCurrentPosition() + directionalCounts;
        int newRightRearTarget = rightRear.getCurrentPosition() + counts;

        // Pass target positions to motor controllers
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);

        // Begin running routine
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion
        RUNTIME.reset();
        leftFront.setPower(-voltage);
        rightFront.setPower(directionalVoltage);
        leftRear.setPower(directionalVoltage);
        rightRear.setPower(-voltage);

        // Keep running while the routine are still active, the timeout has not
        // elapsed, and both motors are still running
        long startTime = System.nanoTime();
        while (opModeIsActive() &&
              (RUNTIME.seconds() < timeout) &&
              (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) &&
              (!poleEnded && toScanPole)) {
            getYellows();
        }
        long duration = System.nanoTime() - startTime;

        // Second run back when scanning for pole
        if (toScanPole)
        {
            int backtrackLeftFrontTarget = leftFront.getCurrentPosition() - counts;
            int backtrackRightFrontTarget = rightFront.getCurrentPosition() - directionalCounts;
            int backtrackLeftRearTarget = leftRear.getCurrentPosition() - directionalCounts;
            int backtrackRightRearTarget = rightRear.getCurrentPosition() - counts;

            // Pass target positions to motor controllers
            leftFront.setTargetPosition(backtrackLeftFrontTarget);
            rightFront.setTargetPosition(backtrackRightFrontTarget);
            leftRear.setTargetPosition(backtrackLeftRearTarget);
            rightRear.setTargetPosition(backtrackRightRearTarget);

            // Begin running routine
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion
            RUNTIME.reset();
            leftFront.setPower(-voltage);
            rightFront.setPower(directionalVoltage);
            leftRear.setPower(directionalVoltage);
            rightRear.setPower(-voltage);

            while (opModeIsActive() &&
                  (RUNTIME.seconds() < timeout) &&
                  (RUNTIME.seconds() < (duration / 2.0)) &&
                  (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {
                // Stall for backtracking
            }
        }

        // Stop all motion
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        // Stop running to position
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *  Gets whether yellow is visible to the color sensor and adds to the detection count.
     */
    public void getYellows()
    {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        boolean isYellow = 28 < hsvValues[0] && hsvValues[0] < 36;

        if ((isYellow && isYellowBuffer) || (isYellowBuffer && yellowDetections > 0))
        {
            yellowDetections++;
            poleEnded = false;
        }
        else if (yellowDetections > 0)
        {
            poleEnded = true;
        }

        isYellowBuffer = isYellow;

        telemetry.addData("detections", "%d", yellowDetections);
        telemetry.addData("yellow?", "%s", isYellow ? "true" : "false");
        telemetry.addLine()
                 .addData("red", "%.3f", colors.red)
                 .addData("green", "%.3f", colors.green)
                 .addData("blue", "%.3f", colors.blue);
        telemetry.addLine()
                 .addData("hue", "%.3f", hsvValues[0])
                 .addData("saturation", "%.3f", hsvValues[1])
                 .addData("value", "%.3f", hsvValues[2]);
        telemetry.update();
    }
}

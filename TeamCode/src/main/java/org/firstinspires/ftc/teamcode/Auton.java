package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    // Side length of square tag in meters
    private static final double TAG_SIZE = 0.166;

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
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;

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

        // Wait for game to begin
        waitForStart();

        // Detect AprilTag
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

        // Move for auton
        if (tagOfInterest == null || tagOfInterest.id == LEFT)
        {
            // Left or default
            driveSideways(-12);
            driveForward(22.5);
        }
        else if (tagOfInterest.id == MIDDLE)
        {
            // Middle
            driveForward(22.5);
        }
        else
        {
            // Right
            driveSideways(-12);
            driveForward(22.5);
        }
    }

    /**
     *  Calls encoderDrive() to move forward a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel forward;
     *                  positive is forward
     */
    public void driveForward(double distance)
    {
        encoderDrive(DRIVE_SPEED, distance, 5, false);
    }

    /**
     *  Calls encoderDrive() to move forward a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right
     */
    public void driveSideways(double distance)
    {
        encoderDrive(DRIVE_SPEED, distance, 5, true);
    }

    /**
     *  Uses encoders to drive a specified distance in a specified direction
     *  at a specified speed.
     *
     *  @param voltage The power (-1.0 to 1.0) to send to each motor
     *  @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right or forward (depending on sidewaysSwitch)
     *  @param timeout The time, in seconds, that a robot should have completed the
     *                 routine by; if it hasn't, abort the routine
     *  @param sidewaysSwitch If true, move left/right; if false, more forward/backward
     */
    public void encoderDrive(double voltage, double distance, double timeout, boolean sidewaysSwitch)
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
        while (opModeIsActive() &&
                (RUNTIME.seconds() < timeout) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {
            // Blocking; no internal code
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
}

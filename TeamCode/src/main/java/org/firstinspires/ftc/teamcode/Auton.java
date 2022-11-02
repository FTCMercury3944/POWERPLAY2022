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
import java.util.Locale;

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

@Autonomous
public class Auton extends LinearOpMode
{
    private AprilTagDetection tagOfInterest;
    private OpenCvCamera camera;

    // Side length of square tag in meters
    // TODO: Calibrate
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

    // Drive train specs
    // Wheel diameter in inches
    private static final double COUNTS_PER_REVOLUTION = 537.7;
    private static final double GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER = 4.0;

    private static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER * Math.PI);

    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        telemetry.setMsTransmissionInterval(50);

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

        // Initialize the DC motors for each wheel
        // Declare op-mode members
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse right side
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Show end point
        if(tagOfInterest != null)
        {
            telemetry.addLine(String.format(Locale.ENGLISH, "Parking zone %s!", numToZone(tagOfInterest.id)));
        }
        else
        {
            telemetry.addLine("No tag found.");
        }

        telemetry.update();

        // TODO: Put in method
        int leftFrontTarget;
        int leftRearTarget;
        int rightFrontTarget;
        int rightRearTarget;

        double leftInches = 48.0;
        double rightInches = 48.0;
        double timeoutS = 5.0;

        // Ensure that the op-mode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            leftRearTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rightFrontTarget = leftFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            rightRearTarget = leftFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(leftFrontTarget);
            leftRear.setTargetPosition(leftRearTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            rightRear.setTargetPosition(rightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(DRIVE_SPEED));
            leftRear.setPower(Math.abs(DRIVE_SPEED));
            rightFront.setPower(Math.abs(DRIVE_SPEED));
            rightRear.setPower(Math.abs(DRIVE_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && leftRear.isBusy() && (rightFront.isBusy() && rightRear.isBusy())))
            {

            }

            // Stop all motion;
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }



    /**
     *  Converts a number from 1 through 3 to the parking zone name
     *
     *  @param parkingZoneId Number representing parking zone (1, 2, or 3)
     *  @return Parking zone name (left, middle, or right)
     */
    private String numToZone(int parkingZoneId)
    {
        return new String[]{"left", "middle", "right"}[parkingZoneId - 1];
    }
}

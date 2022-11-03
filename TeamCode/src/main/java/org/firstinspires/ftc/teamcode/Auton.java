package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
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

    // DC Motors
    // TODO: Find a way to avoid NullPointerException without 500 lines of boilerplate
    private DcMotor leftFront = new DcMotor() {
        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }

        @Override
        public void setDirection(Direction direction) {

        }

        @Override
        public Direction getDirection() {
            return null;
        }

        @Override
        public void setPower(double power) {

        }

        @Override
        public double getPower() {
            return 0;
        }

        @Override
        public MotorConfigurationType getMotorType() {
            return null;
        }

        @Override
        public void setMotorType(MotorConfigurationType motorType) {

        }

        @Override
        public DcMotorController getController() {
            return null;
        }

        @Override
        public int getPortNumber() {
            return 0;
        }

        @Override
        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

        }

        @Override
        public ZeroPowerBehavior getZeroPowerBehavior() {
            return null;
        }

        @Override
        public void setPowerFloat() {

        }

        @Override
        public boolean getPowerFloat() {
            return false;
        }

        @Override
        public void setTargetPosition(int position) {

        }

        @Override
        public int getTargetPosition() {
            return 0;
        }

        @Override
        public boolean isBusy() {
            return false;
        }

        @Override
        public int getCurrentPosition() {
            return 0;
        }

        @Override
        public void setMode(RunMode mode) {

        }

        @Override
        public RunMode getMode() {
            return null;
        }
    };
    private DcMotor rightFront = new DcMotor() {
        @Override
        public MotorConfigurationType getMotorType() {
            return null;
        }

        @Override
        public void setMotorType(MotorConfigurationType motorType) {

        }

        @Override
        public DcMotorController getController() {
            return null;
        }

        @Override
        public int getPortNumber() {
            return 0;
        }

        @Override
        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

        }

        @Override
        public ZeroPowerBehavior getZeroPowerBehavior() {
            return null;
        }

        @Override
        public void setPowerFloat() {

        }

        @Override
        public boolean getPowerFloat() {
            return false;
        }

        @Override
        public void setTargetPosition(int position) {

        }

        @Override
        public int getTargetPosition() {
            return 0;
        }

        @Override
        public boolean isBusy() {
            return false;
        }

        @Override
        public int getCurrentPosition() {
            return 0;
        }

        @Override
        public void setMode(RunMode mode) {

        }

        @Override
        public RunMode getMode() {
            return null;
        }

        @Override
        public void setDirection(Direction direction) {

        }

        @Override
        public Direction getDirection() {
            return null;
        }

        @Override
        public void setPower(double power) {

        }

        @Override
        public double getPower() {
            return 0;
        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };
    private DcMotor leftRear = new DcMotor() {
        @Override
        public MotorConfigurationType getMotorType() {
            return null;
        }

        @Override
        public void setMotorType(MotorConfigurationType motorType) {

        }

        @Override
        public DcMotorController getController() {
            return null;
        }

        @Override
        public int getPortNumber() {
            return 0;
        }

        @Override
        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

        }

        @Override
        public ZeroPowerBehavior getZeroPowerBehavior() {
            return null;
        }

        @Override
        public void setPowerFloat() {

        }

        @Override
        public boolean getPowerFloat() {
            return false;
        }

        @Override
        public void setTargetPosition(int position) {

        }

        @Override
        public int getTargetPosition() {
            return 0;
        }

        @Override
        public boolean isBusy() {
            return false;
        }

        @Override
        public int getCurrentPosition() {
            return 0;
        }

        @Override
        public void setMode(RunMode mode) {

        }

        @Override
        public RunMode getMode() {
            return null;
        }

        @Override
        public void setDirection(Direction direction) {

        }

        @Override
        public Direction getDirection() {
            return null;
        }

        @Override
        public void setPower(double power) {

        }

        @Override
        public double getPower() {
            return 0;
        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };
    private DcMotor rightRear = new DcMotor() {
        @Override
        public MotorConfigurationType getMotorType() {
            return null;
        }

        @Override
        public void setMotorType(MotorConfigurationType motorType) {

        }

        @Override
        public DcMotorController getController() {
            return null;
        }

        @Override
        public int getPortNumber() {
            return 0;
        }

        @Override
        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

        }

        @Override
        public ZeroPowerBehavior getZeroPowerBehavior() {
            return null;
        }

        @Override
        public void setPowerFloat() {

        }

        @Override
        public boolean getPowerFloat() {
            return false;
        }

        @Override
        public void setTargetPosition(int position) {

        }

        @Override
        public int getTargetPosition() {
            return 0;
        }

        @Override
        public boolean isBusy() {
            return false;
        }

        @Override
        public int getCurrentPosition() {
            return 0;
        }

        @Override
        public void setMode(RunMode mode) {

        }

        @Override
        public RunMode getMode() {
            return null;
        }

        @Override
        public void setDirection(Direction direction) {

        }

        @Override
        public Direction getDirection() {
            return null;
        }

        @Override
        public void setPower(double power) {

        }

        @Override
        public double getPower() {
            return 0;
        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };

    // Drive train specs
    private static final double COUNTS_PER_REVOLUTION = 537.7;
    private static final double GEAR_REDUCTION_RATIO = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;

    private static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION * GEAR_REDUCTION_RATIO) /
                                                  (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.5;

    private final ElapsedTime CLOCK = new ElapsedTime();

    public Auton() {
    }

    @Override
    public void runOpMode()
    {
        /*
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
         */

        // Set up DC motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

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
         */

        for (int i = 0; i < 5; i++)
        {
            moveDistance(48.0, 5.0);
            moveDistance(-48.0, 5.0);
        }
    }

    /**
     *  Moves the robot a specified distance with left and right control.
     *
     *  @param leftDistance Distance (inches) to move on the left side
     *  @param rightDistance Distance (inches) to move on the right side
     *  @param timeout Timeout to continue if robot doesn't complete by then
     */
    private void moveDistance(double leftDistance, double rightDistance, double timeout)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        int leftFrontTarget = leftFront.getCurrentPosition() - (int)(leftDistance * COUNTS_PER_INCH);
        int leftRearTarget = leftRear.getCurrentPosition() - (int)(leftDistance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() - (int)(rightDistance * COUNTS_PER_INCH);
        int rightRearTarget = rightRear.getCurrentPosition() - (int)(rightDistance * COUNTS_PER_INCH);

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
        CLOCK.reset();
        leftFront.setPower(Math.abs(DRIVE_SPEED));
        leftRear.setPower(Math.abs(DRIVE_SPEED));
        rightFront.setPower(Math.abs(DRIVE_SPEED));
        rightRear.setPower(Math.abs(DRIVE_SPEED));

        while (opModeIsActive() &&
                (CLOCK.seconds() < timeout) &&
                (leftFront.isBusy() && leftRear.isBusy() &&
                 rightFront.isBusy() && rightRear.isBusy()))
        {
            // Keep motors running for duration
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

        sleep(250);
    }

    /**
     *  Moves the robot a specified distance. Can only move straight.
     *
     *  @param distance Distance (inches) to move (straight)
     *  @param timeout Timeout to continue if robot doesn't complete by then
     */
    private void moveDistance(double distance, double timeout)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Determine new target position, and pass to motor controller
        int leftFrontTarget = leftFront.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        int leftRearTarget = leftRear.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
        int rightRearTarget = rightRear.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);

        // TODO
        telemetry.addLine(String.format(Locale.ENGLISH, "LF %d LR %d RF %d RR %d", leftFrontTarget, leftRearTarget, rightFrontTarget, rightRearTarget));
        telemetry.update();

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
        CLOCK.reset();
        leftFront.setPower(Math.abs(DRIVE_SPEED));
        leftRear.setPower(Math.abs(DRIVE_SPEED));
        rightFront.setPower(Math.abs(DRIVE_SPEED));
        rightRear.setPower(Math.abs(DRIVE_SPEED));

        while (opModeIsActive() &&
                (CLOCK.seconds() < timeout) &&
                (leftFront.isBusy() && leftRear.isBusy() &&
                 rightFront.isBusy() && rightRear.isBusy()))
        {
            // Keep motors running for duration
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

        sleep(250);
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

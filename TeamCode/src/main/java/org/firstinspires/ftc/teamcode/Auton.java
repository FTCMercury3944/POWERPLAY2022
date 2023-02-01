package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

    private double globalAngle;
    private Orientation lastAngles = new Orientation();

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

    // Junction constants 0, 1, and 2
    private static final int GROUND = 0;
    private static final int LOW = 1;
    private static final int HIGH = 2;

    private final ElapsedTime RUNTIME = new ElapsedTime();
    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.5;
    private static final double LIFT_SPEED = 0.2;

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
        // TODO: Default after 5 seconds

        /*
        boolean tagFound = false;
        while (isStarted() && !tagFound && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if (currentDetections.size() != 0)
                for (AprilTagDetection tag : currentDetections)
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }

            sleep(20);
        }

        // Move for auton
        // TODO: Make routine for right
        if (tagOfInterest == null || tagOfInterest.id == LEFT)
        {
            driveSideways(-14.0);
            driveForward(16.0);
        }
        else if (tagOfInterest.id == MIDDLE)
            driveForward(22.5);
        else // tagOfInterest.id == RIGHT
        {
            driveSideways(14.0);
            driveForward(16.0);
        }
         */

        // Preload cycle
        liftMove(GROUND);
        closeClaw();

        // Move to cone
        driveSideways(-14.0);
        driveForward(29.0);
        gyroTurn(84.0); // Attempting a 90-degree turn

        // Lift and drop cone
        liftMove(HIGH);
        driveForward(2.0);
        openClaw();
    }

    /**
     *  Calls encoderDrive() to move forward a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel forward;
     *                  positive is forward
     */
    public void driveForward(double distance)
    {
        encoderDrive(distance, false);
    }

    /**
     *  Calls encoderDrive() to move sideways a specified distance.
     *
     *  @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right
     */
    public void driveSideways(double distance)
    {
        encoderDrive(distance, true);
    }

    /**
     *  Calls clawMove() to open the claw.
     */
    public void openClaw()
    {
        clawMove(1.0);
    }

    /**
     *  Calls clawMove() to close the claw.
     */
    public void closeClaw()
    {
        clawMove(0.0);
    }

    /**
     *  Uses encoders to drive a specified distance in a specified direction at a specified speed.
     *   @param distance The distance, in inches, a robot should travel sideways;
     *                  positive is right or forward (depending on sidewaysSwitch)
     *  @param sidewaysSwitch If true, move left/right; if false, more forward/backward
     */
    private void encoderDrive(double distance, boolean sidewaysSwitch)
    {
        // Calculate distance
        int counts = (int)(-distance * COUNTS_PER_INCH);

        // Determine directional variables
        int directionalCounts = counts;
        double directionalVoltage = Math.abs(DRIVE_SPEED);
        if (sidewaysSwitch)
        {
            directionalCounts = -counts;
            directionalVoltage = -DRIVE_SPEED;
        }

        // Initialize the DC motors for each wheel
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
        leftFront.setPower(-DRIVE_SPEED);
        rightFront.setPower(directionalVoltage);
        leftRear.setPower(directionalVoltage);
        rightRear.setPower(-DRIVE_SPEED);

        // Keep running while the routine are still active, the timeout has not
        // elapsed, and both motors are still running
        while (opModeIsActive() &&
              (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()))
        {}

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
     *  Uses gyroscope to turn a specified angle at a specified speed.
     *
     * @param angle The angle, in degrees, a robot should turn;
     *               positive is clockwise
     */
    private void gyroTurn(double angle)
    {
        // Initialize the DC motors for each wheel
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

        // Initialize IMU for movement tracking
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        // Restart IMU movement tracking
        resetAngle();

        // Calculate voltage for turn
        double voltL, voltR;
        if (angle > 0) // Right turn
        {
            voltL = -DRIVE_SPEED;
            voltR = DRIVE_SPEED;
        }
        else if (angle < 0) // Left turn
        {
            voltL = DRIVE_SPEED;
            voltR = -DRIVE_SPEED;
        }
        else return;

        // Set power to begin rotating
        leftFront.setPower(voltL);
        rightFront.setPower(voltR);
        leftRear.setPower(voltL);
        rightRear.setPower(voltR);

        // Keep running until angle
        if (angle > 0) // Right turn
        {
            // On a right turn, the robot must get off zero first
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() < angle) {}
        }
        else // Left turn
            while (opModeIsActive() && getAngle() > angle) {}

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

        // Wait for rotation to stop
        sleep(1000);

        // Reset angle tracking on new heading
        resetAngle();
    }

    /**
     *  Moves the claw to a specified target position and stall for a short period of time.
     *
     *  @param targetPos A value between 0.0 and 1.0, inclusive,
     *                   that the claw should o to. 1.0 is open,
     *                   0.0 is closed.
     */
    private void clawMove(double targetPos)
    {
        Servo clawServo = hardwareMap.get(Servo.class, "claw");

        RUNTIME.reset();
        while (opModeIsActive() && RUNTIME.milliseconds() < 500)
            clawServo.setPosition(targetPos);
    }

    /**
     *  Moves the lift to the height of a specified pole.
     *
     *  @param junction Either 0 (ground), 1 (low), or 2 (high).
     *                  The junction to target the height for.
     */
    private void liftMove(int junction)
    {
        int counts = new int[]{9, 200, 295}[junction];

        // Initialize the DC motors for each wheel
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "lift");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");

        // Reverse right side
        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Determine new target position
        int liftTarget = arm1.getCurrentPosition() + counts;

        // Pass target positions to motor controllers
        arm1.setTargetPosition(liftTarget);
        arm2.setTargetPosition(liftTarget);

        // Begin running routine
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion
        arm1.setPower(LIFT_SPEED);
        arm2.setPower(LIFT_SPEED);

        // Keep running while the routine are still active, the timeout has not
        // elapsed, and both motors are still running
        while (opModeIsActive() &&
              (arm1.isBusy() && arm2.isBusy()))
        {}

        // Stop all motion
        arm1.setPower(0.0);
        arm2.setPower(0.0);

        // Stop running to position
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *  Resets the registered angle of the IMU for rotation.
     */
    private void resetAngle()
    {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0.0;
    }

    /**
     *  Gets the current angle as determined by the IMU.
     *
     *  @return The current angle of the robot
     */
    private double getAngle()
    {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle -= deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }
}

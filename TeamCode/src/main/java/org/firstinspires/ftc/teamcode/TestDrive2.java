package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 *  A basic driving op-mode for testing the chassis.
 *  Robot-oriented.
 *
 *  Uses linear algebra to calculate voltage multipliers for mecanums.
 */

@TeleOp(name="TestDrive2")
public class TestDrive2 extends LinearOpMode
{
    private final static int RESET_THRESHOLD = 20;

    private final static int[] HEIGHTS = {100, 500, 1500, 2500};
    private final static double LIFT_SPEED = 0.5;

    private final static int GROUND = 0;
    private final static int LOW = 1;
    private final static int MEDIUM = 2;
    private final static int HIGH = 3;

    private static boolean armMoving = false;

    @Override
    public void runOpMode()
    {
        // Send status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the DC motors for each wheel
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "lift");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");
        Servo clawServo = hardwareMap.get(Servo.class, "claw");
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "switch");

        // Reverse right side and arm motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Resist external force to motor
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for driver to press play
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Determine junction
            // High is up, ground is down, medium is left, low is down
            int junction = -1;
            junction += gamepad1.dpad_down ? 1 : 0;
            junction += gamepad1.dpad_left ? 2 : 0;
            junction += gamepad1.dpad_right ? 3 : 0;
            junction += gamepad1.dpad_up ? 4 : 0;

            // Move lift to position
            if (junction <= 0 && !armMoving)
            {
                // Set flag
                armMoving = true;

                int finalJunction = junction; // Make effectively final for lambda function
                new Thread(() -> liftMove(finalJunction)).start();
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // Calculate voltage multipliers for claw
            // Left bumper is close; right bumper is open
            // TODO: When both pressed, favor most recently pushed
            if (gamepad1.right_bumper)
                clawServo.setPosition(1.0);
            else if (gamepad1.left_bumper)
                clawServo.setPosition(0.0);

            // Calculate voltage multipliers for arm
            // Left trigger is up; right trigger is down
            // TODO: At no pressure, keep still
            double voltArm = 0;
            if (gamepad1.right_trigger > 0.05)
                voltArm = gamepad1.right_trigger;
            if (gamepad1.left_trigger > 0.05)
                voltArm = -gamepad1.left_trigger * 0.7;

            if (limitSwitch.isPressed() && Math.abs(arm1.getCurrentPosition()) > RESET_THRESHOLD)
            {
                arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm1.setPower(0.0);
                arm2.setPower(0.0);
            }
            else
            {
                arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm1.setPower(voltArm);
                arm2.setPower(voltArm);
            }
        }
    }

    /**
     *  Moves the lift to the height of a specified pole.
     *
     *  @param junction Either 0 (ground), 1 (low), 2 (medium), or 3 (high).
     *                  The junction to target the height for.
     */
    private void liftMove(int junction)
    {
        // Get the number of counts per function
        int counts = HEIGHTS[junction];

        // Initialize the DC motors for each wheel
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "lift");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");

        // Determine new target position for encoder
        int armTarget = arm1.getCurrentPosition() + counts;

        // Pass target position to encoder and run routine
        arm1.setTargetPosition(armTarget);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setPower(LIFT_SPEED);

        // Keep running while the routine are still active, the timeout has not
        // elapsed, and the encoder is still running; set non-encoder power at
        // the same time
        while (opModeIsActive() && arm1.isBusy())
            arm2.setPower(LIFT_SPEED);

        // Stop all motion
        arm1.setPower(0.0);
        arm2.setPower(0.0);

        // Stop running to position
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset flag
        armMoving = false;
    }
}


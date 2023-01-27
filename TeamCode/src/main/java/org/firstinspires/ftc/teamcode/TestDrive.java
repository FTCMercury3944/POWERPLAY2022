package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *  A basic driving op-mode for testing the chassis.
 *  Robot-oriented.
 *
 *  Uses linear algebra to calculate voltage multipliers for mecanums.
 */

@TeleOp(name="TeleOp")
public class TestDrive extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // Send status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the DC motors for each wheel
        // Declare op-mode members
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "lift");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");
        Servo clawServo = hardwareMap.get(Servo.class, "claw");

        // Reverse right side
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        arm1.setDirection(DcMotor.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Operation variables
        int slowFactor;
        double voltClaw = 0.0;
        boolean slow = false;

        // Wait for driver to press play
        waitForStart();

        if (isStopRequested()) return;

        // TODO: When aligned, low vibration; when close enough, high vibration
        //
        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Calculate voltage multipliers for wheels
            // Based on Taheri, Qiao, & Ghaeminezhad (2015) in the IJCA
            //          "Kinematic Model of a Four Mecanum Wheeled Mobile Robot"
            double largest = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double voltLF = (y + x + rx) / largest;
            double voltRF = (y - x + rx) / largest;
            double voltLR = (y - x - rx) / largest;
            double voltRR = (y + x - rx) / largest;

            // Slow mode activation upon pressing A
            // Halves movement speed values
            if (gamepad1.a) slow = !slow;

            slowFactor = slow ? 2 : 1;

            leftFront.setPower(voltLF / slowFactor);
            rightFront.setPower(voltRF / slowFactor);
            leftRear.setPower(voltLR / slowFactor);
            rightRear.setPower(voltRR / slowFactor);

            // Calculate voltage multipliers for arm
            // Left trigger is up; right trigger is down
            // TODO: At no pressure, keep still
            double voltArm = 0;
            if (gamepad1.right_trigger > 0.05)
                voltArm = gamepad1.right_trigger;
            if (gamepad1.left_trigger > 0.05)
                voltArm = -gamepad1.left_trigger * 0.7;

            // Calculate voltage multipliers for claw
            // Left bumper is close; right bumper is open
            // TODO: When both pressed, favor most recently pushed
            if (gamepad1.left_bumper) voltClaw = 0;
            if (gamepad1.right_bumper) voltClaw = 1;

            // Power motors and servo
            arm1.setPower(voltArm);
            arm2.setPower(voltArm);
            clawServo.setPosition(voltClaw);
        }
    }
}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 *  Our driver-controlled, robot-oriented op-mode.
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
        DcMotor leftArm = hardwareMap.get(DcMotor.class, "arm2");
        DcMotor rightArm = hardwareMap.get(DcMotor.class, "lift");
        CRServo clawServo = hardwareMap.get(CRServo.class, "claw");

        // Reverse right side
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for driver to press play
        waitForStart();

        if (isStopRequested())
        {
            return;
        }

        boolean slow = false;
        boolean isHeld = false;
        double slowFactor = 1.0;

        while (opModeIsActive())
        {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double upForce = gamepad1.left_trigger;
            double downForce = gamepad1.right_trigger;
            boolean clawClose = gamepad1.left_bumper;
            boolean clawOpen = gamepad1.right_bumper;
            boolean slowToggled = gamepad1.a;

            // Slow-mode when A is pressed (halves motor speeds)
            if (slowToggled && !isHeld)
            {
                // Button is being pressed for the first time
                // Set slow-mode
                slow = !slow;
                slowFactor = slow ? 0.5 : 1.0;
                isHeld = true;
            }
            else if (!slowToggled && isHeld)
            {
                isHeld = false;
            }

            // Calculate voltage multipliers for wheels
            // Based on Taheri, Qiao, & Ghaeminezhad (2015) in the IJCA
            //          "Kinematic Model of a Four Mecanum Wheeled Mobile Robot"
            double largest = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double voltLF = (y + x + rx) / largest;
            double voltRF = (y - x + rx) / largest;
            double voltLR = (y - x - rx) / largest;
            double voltRR = (y + x - rx) / largest;

            // Calculate voltage multipliers for arm
            // Left trigger is up; right trigger is down
            // TODO: At no pressure, keep still
            // TODO: Allow second driver's control
            // TODO: Make button presets for low, medium, and high (ground?)
            double voltArm = upForce - (downForce * 0.7);

            // Calculate voltage multipliers for claw
            // Left bumper is close; right bumper is open
            // TODO: When both pressed, favor most recently pushed
            double voltClaw = 0.0;
            if (clawClose)
            {
                voltClaw = 1;
            }
            if (clawOpen)
            {
                voltClaw = -1;
            }

            // Power motors and servo
            leftFront.setPower(voltLF * slowFactor);
            rightFront.setPower(voltRF * slowFactor);
            leftRear.setPower(voltLR * slowFactor);
            rightRear.setPower(voltRR * slowFactor);
            leftArm.setPower(voltArm * slowFactor);
            rightArm.setPower(voltArm * slowFactor);
            clawServo.setPower(voltClaw * slowFactor);
        }
    }
}
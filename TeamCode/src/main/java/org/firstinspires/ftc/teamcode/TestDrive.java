package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        // TODO: clawServo

        // Reverse right side
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for driver to press play
        waitForStart();

        if (isStopRequested())
        {
            return;
        }

        while (opModeIsActive())
        {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            boolean upButton = gamepad1.dpad_up;
            boolean downButton = gamepad1.dpad_down;

            // Calculate voltage multipliers
            // Based on Taheri, Qiao, & Ghaeminezhad (2015) in the IJCA
            //          "Kinematic Model of a Four Mecanum Wheeled Mobile Robot"
            double voltLF = y - x - rx;
            double voltRF = y + x + rx;
            double voltLR = y + x - rx;
            double voltRR = y - x + rx;

            // Adjust calculations for values greater than the maximum (1.0)
            double largest = 1.0;
            largest = Math.max(largest, Math.abs(voltLF));
            largest = Math.max(largest, Math.abs(voltRF));
            largest = Math.max(largest, Math.abs(voltLR));
            largest = Math.max(largest, Math.abs(voltRR));

            // Move arm up when X button is pressed
            if (upButton)
            {
                armMotor.setPower(1.0);
            }
            else if (downButton)
            {
                armMotor.setPower(-1.0);
            }
            else
            {
                armMotor.setPower(0.1);
            }

            // Move wheels
            leftFront.setPower(voltLF / largest);
            rightFront.setPower(voltRF / largest);
            leftRear.setPower(voltLR / largest);
            rightRear.setPower(voltRR / largest);
        }
    }
}
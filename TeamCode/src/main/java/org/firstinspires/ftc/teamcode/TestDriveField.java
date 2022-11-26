package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 *  A basic driving op-mode for testing the chassis.
 *  Field-oriented.
 *  TODO.
 *
 *  Uses linear algebra to calculate voltage multipliers for mecanums.
 */

@TeleOp(name="TeleOpField")
public class TestDriveField extends LinearOpMode
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

        // Reverse right side
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve inertial measurement unit
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // Wait for driver to press play
        waitForStart();

        if (isStopRequested())
        {
            return;
        }

        while (opModeIsActive())
        {
            // Calculate voltage multipliers
            // Based on Taheri, Qiao, & Ghaeminezhad (2015) in the IJCA
            //          "Kinematic Model of a Four Mecanum Wheeled Mobile Robot"
            double voltLF = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            double voltRF = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double voltLR = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            double voltRR = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

            // Adjust calculations for values greater than the maximum (1.0)
            double largest = 1.0;
            largest = Math.max(largest, Math.abs(voltLF));
            largest = Math.max(largest, Math.abs(voltRF));
            largest = Math.max(largest, Math.abs(voltLR));
            largest = Math.max(largest, Math.abs(voltRR));

            leftFront.setPower(voltLF / largest);
            rightFront.setPower(voltRF / largest);
            leftRear.setPower(voltLR / largest);
            rightRear.setPower(voltRR / largest);
        }
    }
}
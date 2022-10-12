package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 *  A basic driving op-mode for testing the chassis.
 */

@TeleOp(name="TeleOp")
public class TestDrive extends LinearOpMode {
    private double powerLF, powerRF, powerLR, powerRR;

    @Override
    public void runOpMode() {
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

        // Motor-specific config
        // Resist external force to motor
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for driver to press play
        waitForStart();

        // Set power to motors based on left and right stick
        while (opModeIsActive()) {
            // Calculations (mecanum)
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double rightX = gamepad1.right_stick_x;
            double robotAngle = Math.atan2(gamepad1.left_stick_y,
                                           gamepad1.left_stick_x) - Math.PI / 4;

            powerLF = r * Math.sin(robotAngle) - rightX;
            powerRF = r * Math.cos(robotAngle) + rightX;
            powerLR = r * Math.cos(robotAngle) - rightX;
            powerRR = r * Math.sin(robotAngle) + rightX;

            leftFront.setPower(powerLF);
            rightFront.setPower(powerRF);
            leftRear.setPower(powerLR);
            rightRear.setPower(powerRR);
        }
    }
}
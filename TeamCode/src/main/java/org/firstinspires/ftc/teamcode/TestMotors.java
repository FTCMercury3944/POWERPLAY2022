package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *  A basic op-mode for testing motors.
 */

@TeleOp(name="TestMotors")
public class TestMotors extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

        while (opModeIsActive())
        {
            motor.setPower(1.0);
        }
    }
}
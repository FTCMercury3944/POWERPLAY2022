package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

    private static final double FEET_PER_METER = 3.28084;

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

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        // Setup webcam
        int cameraID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraID);

        // Begin detection pipeline
        DetectionPipeline pipeline = new DetectionPipeline(TAG_SIZE, FX, FY, CX, CY);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Nonexistent error handling, required for async
                // TODO Perhaps?
            }
        });

        // Init loop, replacing waitForStart()
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if (currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections)
                {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound)
                {
                    tagToTelemetry(tagOfInterest);
                }
                else if (tagOfInterest != null)
                {
                    tagToTelemetry(tagOfInterest);
                }

            }
            else if (tagOfInterest != null)
            {
                tagToTelemetry(tagOfInterest);
            }

            telemetry.update();
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

        // TODO: Remove (debug)
        while (opModeIsActive())
        {
            sleep(20);
        }
    }

    /**
     *  Converts a number from 1 through 3 to the parking zone name
     *
     *  @param parkingZoneId 1, 2, or 3 - number representing parking zone
     *  @return Parking zone name (left, middle, or right)
     */
    private String numToZone(int parkingZoneId)
    {
        return new String[]{"left", "middle", "right"}[parkingZoneId - 1];
    }

    /**
     *  Print out detected tag, translation, and rotation information
     *
     *  @param detection Detected tag
     */
    private void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format(Locale.ENGLISH, "Detected tag ID: %d", detection.id));
        telemetry.addLine(String.format(Locale.ENGLISH, "Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH, "Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH, "Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH, "Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format(Locale.ENGLISH, "Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format(Locale.ENGLISH, "Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

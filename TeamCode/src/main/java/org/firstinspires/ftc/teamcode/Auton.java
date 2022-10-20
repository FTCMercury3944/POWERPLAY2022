package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.CompletableFuture;

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

    static final double FEET_PER_METER = 3.28084;

    // Width/height (w = h) of tag in meters
    private static final double TAG_SIZE = 0.166;

    // Lens intrinsics for Logitech C920 webcam
    // (fx, fy) is focal length in pixels
    // (cx, cy) is principal point in pixels
    private final double FX = 1428.470;
    private final double FY = 1428.470;
    private final double CX = 630.909;
    private final double CY = 358.721;

    // Tag IDs 1, 2, and 3 from the 36H11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        // Setup webcam
        int cameraID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraID);

        // Start pipeline
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

            if(currentDetections.size() != 0)
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
                    telemetry.addLine("Tag is in sight!\n\n");
                }
            }

            tagToTelemetry(tagOfInterest);
            telemetry.update();

            sleep(20);
        }

        // Send parking zone to telemetry
        if (tagOfInterest == null)
        {
            telemetry.addLine("No tag snapshot available.");
        }
        else
        {
            telemetry.addLine(String.format(Locale.ENGLISH, "Parking zone %d!", tagOfInterest.id));
            telemetry.update();
        }

        // TODO: Remove (debug)
        while (opModeIsActive())
        {
            sleep(20);
        }
    }

    /**
     *  Print out detected tag, translation, and rotation information
     *
     *  @param detection Detected tag
     */
    void tagToTelemetry(AprilTagDetection detection)
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

package org.firstinspires.ftc.teamcode;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/**
 *  AprilTag detection pipeline, for use in Autonomous mode.
 */

class DetectionPipeline extends OpenCvPipeline
{
    private long nativeAprilTagPtr;
    private final Mat GRAY = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private Mat cameraMatrix;

    private final Scalar BLUE = new Scalar(7, 197, 235, 255);
    private final Scalar RED = new Scalar(255, 0, 0, 255);
    private final Scalar GREEN = new Scalar(0, 255, 0, 255);
    private final Scalar WHITE = new Scalar(255, 255, 255, 255);

    // (FX, FY) is focal length in pixels
    // (CX, CY) is principal point in pixels
    private final double FX;
    private final double FY;
    private final double CX;
    private final double CY;

    // Side length of square tag in meters
    private final double TAG_SIZE;

    private boolean needToSetDecimation;
    private final Object DECIMATION_SYNC = new Object();

    public DetectionPipeline(double tagSize, double fx, double fy, double cx, double cy)
    {
        this.TAG_SIZE = tagSize;
        this.FX = fx;
        this.FY = fy;
        this.CX = cx;
        this.CY = cy;

        // Construct camera matrix
        constructMatrix();

        // Allocate a native context object
        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    /**
     *  End routine.
     */
    @Override
    protected void finalize()
    {
        if (nativeAprilTagPtr != 0.0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
            nativeAprilTagPtr = 0;
        }
    }

    /**
     *  Finds AprilTag in the provided frame of video capture.
     *
     *  @param input The provided frame of video
     */
    @Override
    public Mat processFrame(Mat input)
    {
        // Convert to grayscale
        Imgproc.cvtColor(input, GRAY, Imgproc.COLOR_RGBA2GRAY);

        // Set decimation parameter
        synchronized (DECIMATION_SYNC)
        {
            if (needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeAprilTagPtr, 0.0f);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, GRAY, TAG_SIZE, FX, FY, CX, CY);

        // Draw 6DOF markers on the image with OpenCV
        for (AprilTagDetection detection : detections)
        {
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix);
            drawAxisMarker(input, TAG_SIZE / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, TAG_SIZE, TAG_SIZE, TAG_SIZE, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        return input;
    }

    /**
     *  Gets the list of detections per frame.
     *
     *  @return ArrayList of detections
     */
    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    /**
     *  Constructs the camera matrix for use by OpenCV.
     */
    private void constructMatrix()
    {
        // Matrix:
        //        ┌------------┐
        //        | fx  0   cx |
        //        | 0   fy  cy |
        //        | 0   0   1  |
        //        └------------┘

        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        // Top
        cameraMatrix.put(0, 0, FX);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, CX);

        // Middle
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, FY);
        cameraMatrix.put(1, 2, CY);

        // Bottom
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    /**
     *  Draw a 3D axis to mark a detection.
     *
     *  @param buf The RGB buffer on which to draw the marker
     *  @param length The length of each of the marker 'poles'
     *  @param rvec The rotation vector of the detection
     *  @param tvec The translation vector of the detection
     *  @param cameraMatrix The camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane
        // The origin of the space is assumed to be in the center of the detection
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(length, 0, 0),
                new Point3(0, length, 0),
                new Point3(0, 0, -length)
        );

        // Project points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw marker
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], RED, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], BLUE, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, WHITE, -1);
    }

    /**
     *  Draw a 3D cube to mark a detection.
     *
     *  @param buf The RGB buffer on which to draw the marker
     *  @param length The length of each of the marker 'poles'
     *  @param tagWidth Width of the AprilTag
     *  @param tagHeight Height of the AprilTag
     *  @param thickness Thickness of the cube
     *  @param rvec The rotation vector of the detection
     *  @param tvec The translation vector of the detection
     *  @param cameraMatrix The camera matrix used when finding the detection
     */
    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane
        // The origin of the space is assumed to be in the center of the detection
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length)
        );

        // Project points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw pillars
        for (int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], BLUE, thickness);
        }

        // Draw top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], GREEN, thickness);
    }

    /**
     *  Extracts 6DOF pose from a trapezoid.
     *
     *  @param points The points which form the trapezoid
     *  @param cameraMatrix The camera intrinsics matrix
     *  @return The 6DOF pose of the camera relative to the tag
     */
    private Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an "ideal projection"
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-TAG_SIZE / 2, TAG_SIZE / 2, 0);
        arrayPoints3d[1] = new Point3(TAG_SIZE / 2, TAG_SIZE / 2, 0);
        arrayPoints3d[2] = new Point3(TAG_SIZE / 2, -TAG_SIZE / 2, 0);
        arrayPoints3d[3] = new Point3(-TAG_SIZE / 2, -TAG_SIZE / 2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /**
     *  A simple container to hold both rotation and translation vectors, which together form a 6DOF pose
     */
    private static class Pose
    {
        public final Mat rvec;
        public final Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }
    }
}
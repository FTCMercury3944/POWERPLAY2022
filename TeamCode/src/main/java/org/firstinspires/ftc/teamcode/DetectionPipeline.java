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
 *  AprilTag detection pipeline, for use in Autonomous
 */

class DetectionPipeline extends OpenCvPipeline
{
    private long nativeAprilTagPtr;
    private final Mat gray = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private Mat cameraMatrix;

    private final Scalar blue = new Scalar(7, 197, 235, 255);
    private final Scalar red = new Scalar(255, 0, 0, 255);
    private final Scalar green = new Scalar(0, 255, 0, 255);
    private final Scalar white = new Scalar(255, 255, 255, 255);

    // (fx, fy) is focal length in pixels
    // (cx, cy) is principal point in pixels
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;

    // Side length of square tag in meters
    private final double tagSize;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public DetectionPipeline(double tagSize, double fx, double fy, double cx, double cy)
    {
        this.tagSize = tagSize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        // Construct camera matrix
        constructMatrix();

        // Allocate a native context object
        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

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

    @Override
    public Mat processFrame(Mat input)
    {
        // Convert to grayscale
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

        // Set decimation parameter
        synchronized (decimationSync)
        {
            if (needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeAprilTagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, gray, tagSize, fx, fy, cx, cy);

        // Draw 6DOF markers on the image with OpenCV
        for (AprilTagDetection detection : detections)
        {
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSize);
            drawAxisMarker(input, tagSize/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagSize, tagSize, tagSize, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        return input;
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    private void constructMatrix()
    {
        // Construct the camera matrix.
        //        ┌------------┐
        //        | fx  0   cx |
        //        | 0   fy  cy |
        //        | 0   0   1  |
        //        └------------┘

        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        // Top
        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        // Middle
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        // Bottom
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    /**
     *  Draw a 3D axis marker on a detection
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
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

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
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw pillars
        for (int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Draw top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    /**
     *  Extracts 6DOF pose from a trapezoid
     *
     *  @param points The points which form the trapezoid
     *  @param cameraMatrix The camera intrinsics matrix
     *  @param tagSize The original side length of the tag
     *  @return The 6DOF pose of the camera relative to the tag
     */
    private Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagSize)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an "ideal projection"
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagSize/2, tagSize/2, 0);
        arrayPoints3d[1] = new Point3(tagSize/2, tagSize/2, 0);
        arrayPoints3d[2] = new Point3(tagSize/2, -tagSize/2, 0);
        arrayPoints3d[3] = new Point3(-tagSize/2, -tagSize/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /**
     *  A simple container to hold both rotation and translation vectors, which together form a 6DOF pose.
     */
    private class Pose
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
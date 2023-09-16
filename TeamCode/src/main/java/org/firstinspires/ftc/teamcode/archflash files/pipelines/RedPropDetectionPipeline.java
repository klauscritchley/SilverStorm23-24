import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.video.*;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RedPropDetectionPipeline extends OpenCvPipeline {
    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private Mat detectedObject = new Mat();

    private Scalar lowerRed = new Scalar(80, 100, 100); // Lower bound for blue color in HSV
    private Scalar upperRed = new Scalar(120, 255, 255); // Upper bound for blue color in HSV

    private Point objectCenter = new Point(-1, -1); // Initialize with an invalid point

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_BGR2HSV);

        // Create a mask to detect blue within the specified range
        Core.inRange(hsvMat, lowerRed, upperRed, mask);

        // Find contours in the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter and find the largest blue object
        double maxArea = -1;
        int maxAreaIdx = -1;
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea) {
                maxArea = area;
                maxAreaIdx = i;
            }
        }

        // If a blue object is found, calculate its center
        if (maxAreaIdx >= 0) {
            Moments moments = Imgproc.moments(contours.get(maxAreaIdx));
            double m00 = moments.get_m00();
            double m10 = moments.get_m10();
            double m01 = moments.get_m01();

            if (m00 != 0) {
                int centerX = (int) (m10 / m00);
                int centerY = (int) (m01 / m00);
                objectCenter.x = centerX;
                objectCenter.y = centerY;
            } else {
                objectCenter.x = -1;
                objectCenter.y = -1;
            }

            // Draw a circle at the object's center
            Imgproc.circle(input, objectCenter, 20, new Scalar(0, 255, 0), -1);
        } else {
            objectCenter.x = -1;
            objectCenter.y = -1;
        }

        return input;
    }

    // Get the detected object's center
    public Point getObjectCenter() {
        return objectCenter;
    }
}
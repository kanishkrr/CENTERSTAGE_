package VisionTest;

import org.apache.commons.math3.optim.InitialGuess;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class GPT extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input image to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            // Define the blue color range
            Scalar lowerBound = new Scalar(94, 107, 89);
            Scalar upperBound = new Scalar(136, 255, 255);

            // Threshold the image to detect blue color
            Mat mask = new Mat();
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // Find contours in the mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Release Mats to avoid memory leaks
            hsv.release();
            mask.release();
            Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            // Process contours to find the blue cube
            /*for (MatOfPoint contour : contours) {

                double area = Imgproc.contourArea(contour);
                if (area > 200 && area < 300) { // Adjust the area threshold based on your specific scenario
                    // Draw a bounding rectangle around the detected object
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);

                    // You can add additional logic here based on the detected cube
                }
            }*/

            return input;
        }
    }

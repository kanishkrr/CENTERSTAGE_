package VisionTest;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.*;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Color extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Mat output = new Mat();
        Scalar bottom = new Scalar(94, 107, 89);
        Scalar top = new Scalar(136, 255, 255);
        Core.inRange(input, bottom, top, output);
        return output;
        // Convert the input image to grayscale
        /*Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply GaussianBlur to reduce noise and improve edge detection
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);

        // Apply Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 50, 150);

        // Release the intermediate gray Mat to avoid memory leaks
        gray.release();

        // Return the edges Mat for further processing or display
        return edges;*/
    }
}

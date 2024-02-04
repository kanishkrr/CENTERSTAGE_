package VisionTest;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.*;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect2d;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
public class OpenCVTest extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        int c = 0;
        PropPositions propPosition = PropPositions.UNFOUND;
        int bottomAreaofBlock = 0, topAreaofBlock  = 0;
        // Convert the input image to grayscale
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply GaussianBlur to reduce noise and improve edge detection
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);

        // Apply Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 50, 150);

        Imgproc.equalizeHist(gray, gray);
        MatOfPoint2f srcPoints = new MatOfPoint2f(new Point(0, 0), new Point(input.cols(), 0), new Point(input.cols(), input.rows()), new Point(0, input.rows()));
        MatOfPoint2f dstPoints = new MatOfPoint2f(new Point(0, 0), new Point(gray.cols(), 0), new Point(gray.cols() * 0.8, gray.rows()), new Point(gray.cols() * 0.2, gray.rows()));
        Mat perspectiveTransform = Imgproc.getPerspectiveTransform(srcPoints, dstPoints);
        Imgproc.warpPerspective(input, gray, perspectiveTransform, input.size());
        // Release the intermediate gray Mat to avoid memory leaks

        Mat temp = Imgcodecs.imread("blueCube.jpg"); //Replace with "redCube.jpg" when needed
        Imgproc.matchTemplate(gray, temp, gray, Imgproc.TM_CCOEFF_NORMED);

        List<MatOfPoint> contourMapLeft = new ArrayList<>();
        List<MatOfPoint> contourMapCenter = new ArrayList<>();
        Mat left = gray.submat(0, 320, -320, 0);
        Mat center = gray.submat(0, -320, 0, 320);
        Imgproc.findContours(left, contourMapLeft, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(center, contourMapCenter, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        /*List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(gray, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);*/
        //gray.release();

        for(MatOfPoint i: contourMapLeft){
            if(Imgproc.contourArea(i) >= bottomAreaofBlock && Imgproc.contourArea(i) <= topAreaofBlock){
                propPosition = PropPositions.LEFT;
                Imgproc.drawContours(gray, contourMapLeft, c, new Scalar(0, 255, 0), 2);
            }
            c++;
        }
        c = 0;
        for(MatOfPoint i: contourMapCenter){
            if(Imgproc.contourArea(i) >= bottomAreaofBlock && Imgproc.contourArea(i) <= topAreaofBlock){
                propPosition = PropPositions.MIDDLE;
                Imgproc.drawContours(gray, contourMapLeft, c, new Scalar(0, 255, 0), 2);
            }
            c++;
        }
        // Return the edges Mat for further processing or display
        return gray;
        //return edges;
    }
    public enum PropPositions{
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND;
    }
}

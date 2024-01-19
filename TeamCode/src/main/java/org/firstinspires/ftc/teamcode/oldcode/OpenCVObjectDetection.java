package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
public class OpenCVObjectDetection {
    public class MyRobot extends LinearOpMode {

        private OpenCvCamera camera;

        @Override
        public void runOpMode() {
            // Initialize hardware and other components

            // Set up the OpenCV camera
            initOpenCVCamera();

            waitForStart();

            while (opModeIsActive()) {
                // Your main robot loop
            }
        }

        private void initOpenCVCamera() {
            // Specify your camera configuration
            // For example, using a webcam:
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

            // Set the image processing pipeline
            camera.setPipeline(new ObjectDetectionPipeline());

            // Start streaming with desired resolution and rotation
            camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        private class ObjectDetectionPipeline extends OpenCvPipeline {

            @Override
            public Mat processFrame(Mat input) {
                // Your object detection logic
                // For example, draw a rectangle on the image
                Imgproc.rectangle(input, new Point(50, 50), new Point(100, 100), new Scalar(0, 255, 0), 2);

                return input;
            }
        }
    }
}

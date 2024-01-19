package org.firstinspires.ftc.teamcode.oldcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "CameraCoords")
public class CameraCoordinates extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231018_181921.tflite";
    private static final String[] LABELS = {
            "team object",
    };
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    DcMotor leftF, leftB, rightF, rightB, Arm;
    Servo angleServo1, angleServo2, rightServo, leftServo;
    public ArrayList<Point2> addCoordinates(List<Recognition> currentRecognitions){
        ArrayList<Point2> s = new ArrayList<>();
        s.add(new Point2(currentRecognitions.get(0).getLeft(), currentRecognitions.get(0).getTop()));
        s.add(new Point2(currentRecognitions.get(0).getRight(), currentRecognitions.get(0).getBottom()));
        return s;
    }
    public void printBoundingCoordinates(ArrayList<Point2> a){
            telemetry.addData("Left (x1): ", a.get(0).getX());
            telemetry.addData("Top (y1): ", a.get(0).getX());
            telemetry.addData("Right (x2): ", a.get(0).getX());
            telemetry.addData("Bottom (y2): ", a.get(0).getX());
            telemetry.update();
    }
    public void runOpMode() {
        leftF = hardwareMap.dcMotor.get("Left_Front_Motor");
        leftB = hardwareMap.dcMotor.get("Left_Back_Motor");
        rightF = hardwareMap.dcMotor.get("Right_Front_Motor");
        rightB = hardwareMap.dcMotor.get("Right_Back_Motor");
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        angleServo1 = hardwareMap.get(Servo.class, "servo1");
        angleServo2 = hardwareMap.get(Servo.class, "servo5");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
        leftServo.setPosition(-1);
        rightServo.setPosition(0.75);
        Arm.setPower(0.23);
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        telemetry.update();
        waitForStart();
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("Recs", currentRecognitions);
        telemetry.update();
        int teamProp;
        if(currentRecognitions.size() != 0){
            float x1 = currentRecognitions.get(0).getLeft();
            float y1 = currentRecognitions.get(0).getTop();
            float x2 = currentRecognitions.get(0).getRight();
            float y2 = currentRecognitions.get(0).getBottom();
            printBoundingCoordinates(addCoordinates(currentRecognitions));

            //Code to determine in which one of the three lines the team prop is in
            //int teamProp; if teamProp == 0, left; if teamProp == 1, center; if teamProp == 2, right
            //Need to modify the numbers through trial and error.
            if(y1 < 0000){
                if(x1 < 0000){
                    teamProp = 0;
                }
                else if(x1 > 0000){
                    teamProp = 2;
                }
            }
            else{
                teamProp = 1;
            }
        }
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.65f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private List<Recognition> telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        return currentRecognitions;
    }

}
class Point2{
    private double x, y;
    public Point2(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }
    public double[] getXY(){
        double[] point = new double[2];
        point[0] = x;
        point[1] = y;
        return point;
    }
}

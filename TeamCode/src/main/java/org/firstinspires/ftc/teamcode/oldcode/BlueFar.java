package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "BlueAutoFar")
public class BlueFar extends LinearOpMode {

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

    DcMotor leftF;
    DcMotor leftB;
    DcMotor rightF;
    DcMotor rightB;
    DcMotor Arm;
    Servo angleServo1;
    Servo angleServo2;
    Servo rightServo;
    Servo leftServo;

    private static final double servoOpen = 0.3;
    public static final double powerMovementConstant = 0.35; //power value when robot is set to move forward or backward
    public static final double powerStrafeConstant = 0.5; //power value when robot is set to strafe left or right
    public static double powerRotateConstant = 0.3; //power value when robot is set to rotate
    public static double revstoInchesSB = (double) 1000/ (double) 23;
    public static double revstoInchesStrafe = (double) 1000/ (double) 20;
    public static double revstoDegreesRotate = (double) 1100/ (double) 90;
    public void hold() {
        while (leftB.isBusy()) {
            idle();
        }
        stopMovement(100);
    }
    public void angleServoDown() {
        angleServo1.setPosition(-0.99);
        angleServo2.setPosition(1);
        sleep(2500);
    }

    public void angleServoUp() {
        angleServo1.setPosition(1.00);
        angleServo2.setPosition(-0.99);
        sleep(1500);
    }
    public void angleServoMiddle(){
        angleServo1.setPosition(-0.52);
        angleServo2.setPosition(0.52);
        sleep(1500);
    }
    public void releaseFirstPixel() {
        rightServo.setPosition(servoOpen);
        sleep(700);
    }

    public void releaseSecondPixel() {
        leftServo.setPosition(servoOpen);
        sleep(700);
    }

    public void stopMovement(int milliseconds) {
        leftF.setPower(0); //stops robot for a set amount of time (milliseconds)
        leftB.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        sleep(milliseconds);
    }

    public void resetMotors() {
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        leftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveForward(double inches) {
        int revs = (int) Math.round(inches*revstoInchesSB);
        resetMotors();

        leftF.setTargetPosition(-revs);
        leftB.setTargetPosition(-revs);
        rightB.setTargetPosition(revs);
        rightF.setTargetPosition(revs);

        runToPosition();

        leftF.setPower(-powerMovementConstant); //moves robot forward based on input (inches)
        leftB.setPower(-powerMovementConstant);
        rightB.setPower(powerMovementConstant);
        rightF.setPower(powerMovementConstant);
        hold();
    }

    public void moveBackward(double inches) {
        int revs = (int) Math.round(inches*revstoInchesSB);
        resetMotors();

        leftF.setTargetPosition(revs);
        leftB.setTargetPosition(revs);
        rightB.setTargetPosition(-revs);
        rightF.setTargetPosition(-revs);

        runToPosition();

        leftF.setPower(powerMovementConstant); //moves robot backwards based on input (inches)
        leftB.setPower(powerMovementConstant);
        rightB.setPower(-powerMovementConstant);
        rightF.setPower(-powerMovementConstant);

        hold();
    }
    public void strafeRight(double inches) {
        int revs = (int) Math.round(inches*revstoInchesStrafe);
        resetMotors();
        leftF.setTargetPosition(-revs);
        leftB.setTargetPosition(revs);
        rightB.setTargetPosition(revs);
        rightF.setTargetPosition(-revs);

        runToPosition();

        leftF.setPower(-powerStrafeConstant); //moves robot right based on input (inches)
        leftB.setPower(powerStrafeConstant);
        rightB.setPower(powerStrafeConstant);
        rightF.setPower(-powerStrafeConstant);

        hold();
    }

    public void strafeLeft(double inches) {
        int revs = (int) Math.round(inches*revstoInchesStrafe);
        resetMotors();

        leftF.setTargetPosition(revs);
        leftB.setTargetPosition(-revs);
        rightB.setTargetPosition(-revs);
        rightF.setTargetPosition(revs);

        runToPosition();

        leftF.setPower(powerStrafeConstant); //moves robot left based on input (inches)
        leftB.setPower(-powerStrafeConstant);
        rightB.setPower(-powerStrafeConstant);
        rightF.setPower(powerStrafeConstant);

        hold();
    }

    public void rotate(double degrees) {
        int revs = (int) Math.round(degrees*revstoDegreesRotate);
        resetMotors();
        leftF.setTargetPosition(revs);
        leftB.setTargetPosition(revs);
        rightB.setTargetPosition(revs);
        rightF.setTargetPosition(revs);

        runToPosition();

        if (revs < 0) {
            powerRotateConstant*=-1;
        }

        leftF.setPower(powerRotateConstant); //rotates robot based on input (degrees) | positive degrees move robot counter-clockwise while negative degrees will move robot clockwise
        leftB.setPower(powerRotateConstant);
        rightB.setPower(powerRotateConstant);
        rightF.setPower(powerRotateConstant);

        if (revs < 0) {
            powerRotateConstant*=-1;
        }


        hold();
    }
    @Override
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
        Arm.setPower(0.18);
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        telemetry.update();
        waitForStart();

        stopMovement(1000);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("Recs", currentRecognitions);
        telemetry.update();

        if (currentRecognitions.size() != 0) {
            moveForward(10);
            Arm.setPower(0.23);
            angleServoDown();
            moveForward(13);
            releaseFirstPixel();
            sleep(100);
            moveBackward(2);
            strafeRight(2);
            rotate(-90);
            strafeRight(21);
            moveBackward(80);
            strafeLeft(24);
            rotate(180);
            Arm.setPower(0.3);
            sleep(500);
            Arm.setPower(0.23);
            strafeLeft(2);
            angleServoUp();
            angleServoMiddle();
            moveForward(8);
            strafeRight(2);
            releaseSecondPixel();
            moveBackward(6);
            angleServoDown();

        }
        else {
            moveForward(2);
            strafeRight(10);
            stopMovement(2000);
            currentRecognitions = tfod.getRecognitions();
            telemetry.addData("Recs", currentRecognitions);
            telemetry.update();
            if (currentRecognitions.size() != 0) {
                Arm.setPower(0.23);
                angleServoDown();
                moveForward(18);
                releaseFirstPixel();
                stopMovement(200);
                moveBackward(9);
                rotate(90);
                strafeLeft(8);
                rotate(-5);
                moveForward(89);
                strafeRight(30);
                Arm.setPower(0.25);
                angleServoUp();
                angleServoMiddle();
                moveForward(8);
                releaseSecondPixel();
                moveBackward(6);
                angleServoDown();
            }
            else {
                moveForward(4);
                rotate(90);
                Arm.setPower(0.23);
                angleServoDown();
                strafeRight(22);
                moveForward(13);
                moveBackward(3);
                releaseFirstPixel();
                moveBackward(6);
                strafeLeft(24);
                rotate(-5);
                moveForward(86);
                strafeRight(16);
                Arm.setPower(0.27);
                angleServoUp();
                angleServoMiddle();
                moveForward(8);
                releaseSecondPixel();
                moveBackward(6);
                angleServoDown();
            }
        }   }

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
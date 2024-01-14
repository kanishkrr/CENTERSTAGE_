package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;


@Autonomous
public class PropPipelineTuner extends OpMode {
    private VisionPortal visionPortal;
    private PropPipeline cam;

    @Override
    public void init() {
        //tuned for red, need to tune it for blue
        Scalar lower = new Scalar(150, 100, 100); //lower threshold
        Scalar upper = new Scalar(180, 255, 255); //upper threshold
        double minArea = 100; //area to detect obj

        cam = new PropPipeline(
                lower,
                upper,
                () -> minArea,
                () -> 200, // left div. line
                () -> 600 // right div. line
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(cam)
                .build();
    }
    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", cam.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + cam.getLargestContourX() + ", y: " + cam.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", cam.getLargestContourArea());
    }


    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        PropPipeline.PropPositions recordedPropPosition = cam.getRecordedPropPosition();


        if (recordedPropPosition == PropPipeline.PropPositions.UNFOUND) {
            recordedPropPosition = PropPipeline.PropPositions.RIGHT;
        }

        switch (recordedPropPosition) {
            case LEFT:
                // code if prop is left
                break;
            case MIDDLE:
                // code if prop is center
                break;
            case RIGHT:
                // code if prop is right
                break;
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        cam.close();
        visionPortal.close();
    }
}
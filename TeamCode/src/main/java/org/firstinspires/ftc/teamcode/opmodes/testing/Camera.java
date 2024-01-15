package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.common.vision.STATE;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;


public class Camera {
    private VisionPortal visionPortal;
    private PropPipeline cam;
    private STATE sid;
    private STATE col;

    public Camera(HardwareMap hardwareMap, STATE state, STATE state1) {
        Scalar lower = new Scalar(0, 0, 0);
        Scalar upper = new Scalar(0, 0, 0);
        double minArea = 100; //area to detect obj
        sid = state1;
        col = state;

        switch(state) {
            case RED:
                lower = new Scalar(150, 100, 100);
                upper = new Scalar(180, 255, 255);
                break;
            case BLUE:
                lower = new Scalar(160, 100, 100); //need to tune this for blue
                upper = new Scalar(180, 255, 255); //need to tune this for blue
                break;
        }

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

    public PropPipeline.PropPositions getRecog() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        PropPipeline.PropPositions recordedPropPosition = cam.getRecordedPropPosition();


        if (recordedPropPosition == PropPipeline.PropPositions.UNFOUND) {
            recordedPropPosition = PropPipeline.PropPositions.RIGHT;
        }

        if ((sid == STATE.FAR && col == STATE.BLUE) || (sid == STATE.CLOSE && col == STATE.RED)) {

            switch (recordedPropPosition) {
                case RIGHT:
                    recordedPropPosition = PropPipeline.PropPositions.LEFT;
                    break;
                case LEFT:
                    recordedPropPosition = PropPipeline.PropPositions.MIDDLE;
                    break;
                case MIDDLE:
                    recordedPropPosition = PropPipeline.PropPositions.RIGHT;
                    break;
            }

        }

        return recordedPropPosition;
    }

    public void stop() {
        cam.close();
        visionPortal.close();
    }

}

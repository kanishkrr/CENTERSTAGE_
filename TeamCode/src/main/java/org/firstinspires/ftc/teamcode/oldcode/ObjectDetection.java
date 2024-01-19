package org.firstinspires.ftc.teamcode.oldcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class ObjectDetection {
    private TfodProcessor tfod;
    private static String Tfod_Model_Asset;
    private static String[] labels;
    private VisionPortal visionPortal;
    private List<Recognition> currentRecognitions;
    private static final boolean USE_WEBCAM = true;
    private HardwareMap hardwareMap;
    public ObjectDetection(String model, String label, VisionPortal v, TfodProcessor tf){
        labels = new String[]{label};
        Tfod_Model_Asset = model;
        visionPortal = v;
        tfod = tf;
    }
    public void getRecognitions(){
        currentRecognitions = tfod.getRecognitions();
        sleep(300);
        currentRecognitions = tfod.getRecognitions();
        sleep(300);
        telemetry.addData("Recs", currentRecognitions);
        telemetry.update();
    }
    public void initializeTfod(){
            tfod = new TfodProcessor.Builder()
                    .setModelAssetName(Tfod_Model_Asset)
                    .setModelLabels(labels)
                    .build();
            VisionPortal.Builder builder = new VisionPortal.Builder();
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }
            builder.addProcessor(tfod);
            visionPortal = builder.build();
            tfod.setMinResultConfidence(0.65f);
    }
    private List<Recognition> getTfodTelemetry() {
        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
        return currentRecognitions;
    }
}

package org.firstinspires.ftc.teamcode.common.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "VisionPipeLineTest")
public class SampleOpMode extends LinearOpMode {
    private VisionPipelineRedClose visionPipelineRedClose;
    private VisionPipelineRedFar visionPipelineRedFar;
    private VisionPipelineBlueClose visionPipelineBlueClose;
    private VisionPipelineBlueFar visionPipelineBlueFar;

    private VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        visionPipelineBlueFar = new VisionPipelineBlueFar();
        visionPipelineBlueClose = new VisionPipelineBlueClose();
        visionPipelineRedClose = new VisionPipelineRedClose();
        visionPipelineRedFar = new VisionPipelineRedFar();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionPipelineBlueFar);

        if(visionPipelineBlueFar.returnLocation() == VisionPipelineBlueFar.TeamPropLocation.Left){
            //code
        }
        else if(visionPipelineBlueFar.returnLocation() == VisionPipelineBlueFar.TeamPropLocation.Middle){
            //code
        }
        else if(visionPipelineBlueFar.returnLocation() == VisionPipelineBlueFar.TeamPropLocation.Right){
            //code
        }
        else{
            //code
        }

    }
}

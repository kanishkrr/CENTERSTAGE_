package org.firstinspires.ftc.teamcode.opmodes.tfauto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.common.commandbase.Side;

import java.util.List;

@Autonomous
public class RCTwoPlusZeroPL extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231027_204348.tflite";
    private static final String[] LABELS = {
            "team object",
    };

    Side side;
    Pose2d purple, yellow, start;
    Vector2d parkLeft;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    Arm arm;
    Intake intake;
    int leftAdd;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.setP(0.23);
        intake = new Intake(hardwareMap);
        intake.initServos();

        initTfod();

        parkLeft = new Vector2d(50.2, -9.6);
        start = new Pose2d(14, -61, Math.toRadians(90));
        drive.setPoseEstimate(start);

        waitForStart();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();

        if(currentRecognitions.size() != 0){
            float x1 = currentRecognitions.get(0).getLeft();
            float y1 = currentRecognitions.get(0).getTop();

            if (x1 < 150) {
                side = Side.CENT;
            } else if (x1 > 150) {
                side = Side.RIGHT;
            } else {
                side = Side.LEFT;
            }
        }

        switch (side) {
            case LEFT:
                yellow = new Pose2d(43.9, -25.8, Math.toRadians(0));
                purple = new Pose2d(12.5, -26.5, Math.toRadians(180));
                leftAdd = 12;
                break;
            case CENT:
                yellow = new Pose2d(43.4, -29.4, Math.toRadians(0));
                purple = new Pose2d(26, 17.3, Math.toRadians(180));
                leftAdd = 26;
                break;
            case RIGHT:
                yellow = new Pose2d(43.4, -34, Math.toRadians(0));
                purple = new Pose2d(37, -24.5, Math.toRadians(180));
                leftAdd = 16;
                break;
        }


        TrajectorySequence traj = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker(() -> {
                    intake.angleServoMiddle();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(yellow)
                .addDisplacementMarker(() -> {
                    intake.releaseSecondPixel();
                })
                .waitSeconds(0.5)
                .back(4)
                .turn(Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.angleServoDown();
                })
                .waitSeconds(1)
                .lineToLinearHeading(purple)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    intake.releaseFirstPixel();
                })
                .waitSeconds(0.5)
                .back(3)
                .turn(Math.toRadians(-5))
                .addDisplacementMarker(() -> {
                    intake.initServos();
                    intake.angleServoUp();
                })
                .back(6)
                .waitSeconds(0.5)
                .strafeRight(14+leftAdd)
                .waitSeconds(0.5)
                .splineToConstantHeading(parkLeft, Math.toRadians(180))
                .back(5.5)
                .build();



        drive.followTrajectorySequence(traj);

    }
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
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

    private List<Recognition> telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

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
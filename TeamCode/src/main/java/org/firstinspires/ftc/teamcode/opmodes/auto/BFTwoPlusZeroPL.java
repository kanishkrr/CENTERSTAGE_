package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.common.rr.util.Side;

import java.util.List;

@Autonomous
public class BFTwoPlusZeroPL extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231018_181921.tflite";
    private static final String[] LABELS = {
            "team object",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    Side side;
    Pose2d purple, yellow, start, parkLeft;
    Arm arm;
    Intake intake;
    double leftDist;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        intake.initServos();

        initTfod();

        parkLeft = new Pose2d(47.2, 59.6, Math.toRadians(180));
        start = new Pose2d(-38, 61, Math.toRadians(-90));

        waitForStart();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();

        if(currentRecognitions.size() != 0){
            float x1 = currentRecognitions.get(0).getLeft();
            float y1 = currentRecognitions.get(0).getTop();

            if (x1 < 1000) {
                side = Side.CENTER;
            } else if (x1 > 1000) {
                side = Side.RIGHT;
            } else {
                side = Side.LEFT;
            }
        }

        switch (side) {
            case LEFT:
                yellow = new Pose2d(46.8, 37, Math.toRadians(0));
                purple = new Pose2d(-34.3, 31.7, Math.toRadians(0));
                leftDist = 26.5;
                break;
            case CENTER:
                yellow = new Pose2d(46.8, 33, Math.toRadians(0));
                purple = new Pose2d(-44.3, 26, Math.toRadians(0));
                leftDist = 32;
                break;
            case RIGHT:
                yellow = new Pose2d(46.8, 27, Math.toRadians(0));
                purple = new Pose2d(-56.3, 31.7, Math.toRadians(0));
                leftDist = 26.5;
                break;
        }

        TrajectorySequence traj = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(purple)
                .addTemporalMarker(0.1, () -> {
                    arm.setHeight(100, 0.3);
                    intake.angleServoDown();
                })
                .addDisplacementMarker(() -> {
                    intake.releaseFirstPixel();
                })
                .waitSeconds(9)
                .strafeLeft(leftDist)
                .forward(64)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.setHeight(70, 0.);
                    intake.angleServoUp();
                    intake.angleServoMiddle();
                })
                .lineToLinearHeading(yellow)
                .addDisplacementMarker(() -> {
                    intake.releaseSecondPixel();
                })
                .waitSeconds(0.5)
                .back(14)
                .strafeLeft(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    arm.setHeight(100, 0.34);
                    intake.angleServoDown();
                })
                .lineToSplineHeading(parkLeft)
                .back(7)
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
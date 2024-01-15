package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;

import java.util.List;
@Config
@Autonomous
public class RedRoutePathing extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231027_204348.tflite";
    private static final String[] LABELS = {
            "team object",
    };

    Pose2d purple, yellow, start;
    Vector2d parkLeft, parkRight;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    Arm arm;
    Intake intake;
    public static double leftPurpleX, leftPurpleY, centerPurpleX, centerPurpleY, rightPurpleX, rightPurpleY, purpleHeading = 0.0;
    public static double leftYellowX, leftYellowY, centerYellowX, centerYellowY, rightYellowX, rightYellowY, yellowHeading = 0.0;
    public static double startX, startY, startHeading = 0.0;
    public static int side = 1;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        arm = new Arm(hardwareMap);
        arm.setP(0.23);

        intake = new Intake(hardwareMap);
        intake.initServos();

        initTfod();

        parkLeft = new Vector2d(50.2, -9.6);
        parkRight = new Vector2d(50.2, -59.6);
        start = new Pose2d(startX, startY, Math.toRadians(startHeading));
        drive.setPoseEstimate(start);

        waitForStart();
        sleep(2000);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();
        telemetryTfod();

        //recognition code here

        switch (side) {
            case 1:
                yellow = new Pose2d(leftYellowX, leftYellowY, Math.toRadians(yellowHeading));
                purple = new Pose2d(leftPurpleX, leftPurpleY, Math.toRadians(purpleHeading));
                break;
            case 2:
                yellow = new Pose2d(centerYellowX, centerYellowY, Math.toRadians(yellowHeading));
                purple = new Pose2d(centerPurpleX, centerPurpleY, Math.toRadians(purpleHeading));
                break;
            case 3:
                yellow = new Pose2d(rightYellowX, rightYellowY, Math.toRadians(yellowHeading));
                purple = new Pose2d(rightPurpleX, rightPurpleY, Math.toRadians(purpleHeading));
                break;
        }




        TrajectorySequence traj = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker(() -> {
                    intake.angleServoMiddle();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(yellow)
                .addDisplacementMarker(() -> {
                    intake.releaseSecondPixel();
                })
                .waitSeconds(0.2)
                .back(4)
                .turn(Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.angleServoDown();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(purple)
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    intake.releaseFirstPixel();
                })
                .waitSeconds(0.2)
                .back(3)
                .turn(Math.toRadians(5))
                .addDisplacementMarker(() -> {
                    intake.initServos();
                    intake.angleServoUp();
                })
                .back(6)
                .waitSeconds(0.2)
                .strafeRight(14)
                .waitSeconds(0.2)
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
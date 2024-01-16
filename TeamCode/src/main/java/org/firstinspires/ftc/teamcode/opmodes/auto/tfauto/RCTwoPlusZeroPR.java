package org.firstinspires.ftc.teamcode.opmodes.auto.tfauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.common.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.common.centerstage.Side;

import java.util.List;

@Autonomous
public class RCTwoPlusZeroPR extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231027_204348.tflite";
    private static final String[] LABELS = {
            "team object",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    Side side;
    Pose2d purple, yellow;
    Vector2d parkRight;
    Arm arm;
    Intake intake;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        intake.initServos();

        initTfod();

        parkRight = new Vector2d(50.2, -59.6);

        waitForStart();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();

        if(currentRecognitions.size() != 0){
            float x1 = currentRecognitions.get(0).getLeft();
            float y1 = currentRecognitions.get(0).getTop();

            if (x1 < 1000) {
                side = Side.LEFT;
            } else if (x1 > 1000) {
                side = Side.CENT;
            } else {
                side = Side.RIGHT;
            }
        }

        switch (side) {
            case LEFT:
                yellow = new Pose2d(46.8, -31, Math.toRadians(0));
                purple = new Pose2d(10.5, -28, Math.toRadians(180));
                break;
            case CENT:
                yellow = new Pose2d(46.8, -37, Math.toRadians(0));
                purple = new Pose2d(22.5, -21.5, Math.toRadians(180));
                break;
            case RIGHT:
                yellow = new Pose2d(46.8, -43, Math.toRadians(0));
                purple = new Pose2d(10.5, 28, Math.toRadians(180));
                break;
        }

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                .lineToSplineHeading(yellow)
                .addTemporalMarker(1, () -> {
                    arm.setHeight(65, 0.27);
                    intake.angleServoMiddle();
                })
                .addTemporalMarker(2.2, () -> {
                    arm.setHeight(arm.getPos(),0.21);
                })
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(purple)
                .addTemporalMarker(2.5, () -> {
                    arm.setHeight(115, 0.32);
                    intake.angleServoDown();
                })
                .addTemporalMarker(4, () -> {
                    arm.setHeight(95, 0.3);
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(parkRight)
                .build();

        drive.followTrajectory(traj);

        intake.releaseSecondPixel();

        drive.followTrajectory(traj1);

        intake.releaseFirstPixel();

        drive.followTrajectory(traj2);

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
package org.firstinspires.ftc.teamcode.opmodes.auto.opencvauto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class BFPR extends LinearOpMode {
    private VisionPortal visionPortal;
    private PropPipeline cam;
    Pose2d start;
    Vector2d parkRight;
    Arm arm;
    Intake intake;
    TrajectorySequence traj;
    SampleMecanumDrive drive;
    PropPipeline.PropPositions recordedPropPosition;
    TrajectorySequence moveRight;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.setP(0.23);
        intake = new Intake(hardwareMap);
        intake.initServos();
        parkRight = new Vector2d(55, 8);
        start =  new Pose2d(-38, 61, Math.toRadians(-90));
        drive.setPoseEstimate(start);
        //tuned for red, need to tune it for blue
        Scalar lower = new Scalar(80, 180, 130);
        Scalar upper = new Scalar(135, 255, 255);
        double minArea = 7000; //area to detect obj

        cam = new PropPipeline(
                lower,
                upper,
                () -> minArea,
                () -> 250, // left div. line
                () -> 650 // right div. line
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(cam)
                .build();


        moveRight = drive.trajectorySequenceBuilder(start)
                .strafeTo(new Vector2d(-42, 59))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(moveRight);

        recordedPropPosition = cam.getRecordedPropPosition();

        if (recordedPropPosition == PropPipeline.PropPositions.UNFOUND) {
            recordedPropPosition = PropPipeline.PropPositions.RIGHT;
        }

        if (recordedPropPosition == PropPipeline.PropPositions.RIGHT) {
            recordedPropPosition = PropPipeline.PropPositions.LEFT;
        } else if (recordedPropPosition == PropPipeline.PropPositions.LEFT) {
            recordedPropPosition = PropPipeline.PropPositions.MIDDLE;
        } else {
            recordedPropPosition = PropPipeline.PropPositions.RIGHT;
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        switch (recordedPropPosition) {
            case LEFT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(-35.3, 34.3, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).waitSeconds(7).back(8).splineToConstantHeading(new Vector2d(-21.9, 8.5), Math.toRadians(0))
                        .forward(26).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoUp();
                            intake.angleServoMiddle();
                        }).splineToConstantHeading(new Vector2d(44.9, 36.2), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).strafeTo(new Vector2d(44, 8)).turn(Math.toRadians(180))
                        .strafeTo(parkRight).build();
                break;
            case MIDDLE:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                            intake.angleServoDown();
                        }).splineToConstantHeading(new Vector2d(-32.7, 33.3), Math.toRadians(-90))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).back(2.5).turn(Math.toRadians(90)).waitSeconds(10).forward(50)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoUp();
                            intake.angleServoMiddle();
                        }).splineToConstantHeading(new Vector2d(44.9, 29.4), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).strafeTo(new Vector2d(44, 8)).turn(Math.toRadians(180))
                        .strafeTo(parkRight).build();
                break;
            case RIGHT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                            intake.angleServoDown();
                        }).splineToConstantHeading(new Vector2d(-43.7, 37.3), Math.toRadians(-90))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).strafeLeft(8)
                        .lineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(0)))
                        .splineToLinearHeading(new Pose2d(-8.6, 9.5), Math.toRadians(0))
                        .waitSeconds(8)
                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                            intake.angleServoUp();
                            intake.angleServoMiddle();
                        }).splineTo(new Vector2d(45.1, 25.8), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).strafeTo(new Vector2d(44, 8)).turn(Math.toRadians(180)).strafeTo(parkRight).build();
                break;
        }

        drive.followTrajectorySequence(traj);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
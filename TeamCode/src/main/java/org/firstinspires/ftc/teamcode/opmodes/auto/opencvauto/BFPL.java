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
public class BFPL extends LinearOpMode {
    private VisionPortal visionPortal;
    private PropPipeline cam;
    Pose2d start;
    Vector2d parkLeft;
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
        parkLeft = new Vector2d(57, 59.6);
        start =  new Pose2d(-38, 61, Math.toRadians(-90));
        drive.setPoseEstimate(start);
        //tuned for red, need to tune it for blue
        Scalar lower = new Scalar(80, 180, 130);
        Scalar upper = new Scalar(135, 255, 255);
        double minArea = 8200; //area to detect obj

        cam = new PropPipeline(
                lower,
                upper,
                () -> minArea,
                () -> 350, // left div. line
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
                        }).forward(8).lineToLinearHeading(new Pose2d(-38, 28.3, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).waitSeconds(7).back(8).strafeRight(16)
                        .splineToConstantHeading(new Vector2d(-21.9, 8.5), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                            intake.angleServoMiddle();
                        }).forward(36).splineToConstantHeading(new Vector2d(48, 41.8), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).back(3).strafeTo(new Vector2d(42, 58)).turn(Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(parkLeft, Math.toRadians(180))).build();
                break;
            case MIDDLE:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(-50, 24, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).waitSeconds(8).strafeLeft(3).back(6).strafeRight(14).splineToConstantHeading(new Vector2d(-21.9, 8.5), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                            intake.angleServoMiddle();
                        }).forward(36).splineToConstantHeading(new Vector2d(48, 37), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).waitSeconds(0.2).UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                            intake.angleServoUp();
                        }).back(3).strafeTo(new Vector2d(45, 58)).turn(Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(parkLeft, Math.toRadians(180))).build();

                break;
            case RIGHT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                            intake.angleServoDown();
                        }).forward(8).lineToLinearHeading(new Pose2d(-59, 28.3, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).waitSeconds(7).strafeRight(16)
                        .splineToConstantHeading(new Vector2d(-21.9, 8.5), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                            intake.angleServoMiddle();
                        }).forward(36).splineToConstantHeading(new Vector2d(48, 32.5), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).back(3).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoUp();
                        }).strafeTo(new Vector2d(42, 58)).turn(Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(parkLeft, Math.toRadians(180))).build();
                break;
        }

        drive.followTrajectorySequence(traj);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
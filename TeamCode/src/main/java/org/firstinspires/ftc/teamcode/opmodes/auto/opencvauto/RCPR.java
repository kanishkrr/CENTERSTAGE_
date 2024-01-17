package org.firstinspires.ftc.teamcode.opmodes.auto.opencvauto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
public class RCPR extends LinearOpMode {
    private VisionPortal visionPortal;
    private PropPipeline cam;
    Pose2d start;
    Vector2d parkRight, rightAlign;
    Arm arm;
    Intake intake;
    TrajectorySequence traj, moveRight;
    SampleMecanumDrive drive;
    PropPipeline.PropPositions recordedPropPosition;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.setP(0.23);
        intake = new Intake(hardwareMap);
        intake.initServos();
        parkRight = new Vector2d(55, -59.6);
        rightAlign = new Vector2d(42, -59.6);
        start = new Pose2d(14, -61, Math.toRadians(90));
        drive.setPoseEstimate(start);

        Scalar lower = new Scalar(150, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);

        double minArea = 9000; //area to detect obj

        cam = new PropPipeline(
                lower,
                upper,
                () -> minArea,
                () -> 200, // left div. line
                () -> 650 // right div. line
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(cam)
                .build();

        moveRight = drive.trajectorySequenceBuilder(start)
                .strafeTo(new Vector2d(18, -59))
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
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(9.7, -33, Math.toRadians(180)))
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).back(16).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoUp();
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(44.9, -30, Math.toRadians(0)))
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            intake.angleServoUp();
                        }).strafeTo(rightAlign).lineToLinearHeading(new Pose2d(parkRight, Math.toRadians(180)))
                        .build();

                break;
            case MIDDLE:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).splineToConstantHeading(new Vector2d(13, -34), Math.toRadians(90))
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).strafeRight(16).UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.angleServoUp();
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(44.9, -29.4, Math.toRadians(0)))
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).strafeTo(rightAlign).lineToLinearHeading(new Pose2d(parkRight, Math.toRadians(180))).build();

                break;
            case RIGHT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).splineToConstantHeading(new Vector2d(20, -37), Math.toRadians(90))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).strafeRight(16).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoUp();
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(44.9, -36, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).strafeTo(rightAlign).lineToLinearHeading(new Pose2d(parkRight, Math.toRadians(180))).build();

                break;
        }

        drive.followTrajectorySequence(traj);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
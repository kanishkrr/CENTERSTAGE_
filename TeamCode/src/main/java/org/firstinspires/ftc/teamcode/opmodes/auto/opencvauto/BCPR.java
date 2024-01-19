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
public class BCPR extends LinearOpMode {
    private VisionPortal visionPortal;
    private PropPipeline cam;
    Pose2d start;
    Vector2d parkRight, rightAlign;
    Arm arm;
    Intake intake;
    TrajectorySequence traj;
    SampleMecanumDrive drive;
    PropPipeline.PropPositions recordedPropPosition;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.setP(0.23);
        intake = new Intake(hardwareMap);
        intake.initServos();
        parkRight = new Vector2d(55, 8);
        rightAlign = new Vector2d(37, 8);
        start = new Pose2d(14, 61, Math.toRadians(-90));
        drive.setPoseEstimate(start);
        //tuned for red, need to tune it for blue
        Scalar lower = new Scalar(80, 180, 130);
        Scalar upper = new Scalar(135, 255, 255);
        double minArea = 7000; //area to detect obj

        cam = new PropPipeline(
                lower,
                upper,
                () -> minArea,
                () -> 200, // left div. line
                () -> 550 // right div. line
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(cam)
                .build();


        while (!opModeIsActive()) {

            recordedPropPosition = cam.getRecordedPropPosition();


            if (recordedPropPosition == PropPipeline.PropPositions.UNFOUND) {
                recordedPropPosition = PropPipeline.PropPositions.RIGHT;
            }
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        switch (recordedPropPosition) {
            case LEFT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(44.9, 36.2, Math.toRadians(0)))
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).back(5).turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(35.7, 24.5, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).back(4).turn(Math.toRadians(-5)).splineToConstantHeading(rightAlign, Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoUp();
                        }).waitSeconds(0.2).splineToConstantHeading(parkRight, Math.toRadians(180)).build();
                break;
            case MIDDLE:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(44.9, 31.4, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).back(5).turn(Math.toRadians(180)).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(26, 19.7, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).back(3).turn(Math.toRadians(5)).splineToConstantHeading(rightAlign, Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.initServos();
                            intake.angleServoUp();
                        }).waitSeconds(0.2).splineToConstantHeading(parkRight, Math.toRadians(180)).build();
                break;
            case RIGHT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(45.1, 25, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            intake.releaseSecondPixel();
                        }).back(5).turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                            intake.initServos();
                        }).lineToLinearHeading(new Pose2d(13.6, 25.5, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).back(4).back(8).splineToConstantHeading(rightAlign, Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            intake.initServos();
                            intake.angleServoUp();
                        }).waitSeconds(0.2).splineToConstantHeading(parkRight, Math.toRadians(180)).build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
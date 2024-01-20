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
public class RFPL extends LinearOpMode {
    private VisionPortal visionPortal;
    private PropPipeline cam;
    Pose2d start;
    Vector2d parkLeft, leftAlign;
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
        parkLeft = new Vector2d(55, -8);
        leftAlign = new Vector2d(39, -8);
        start = new Pose2d(-38, -61, Math.toRadians(90));
        drive.setPoseEstimate(start);

        Scalar lower = new Scalar(0, 180, 110);
        Scalar upper = new Scalar(180, 255, 255);

        double minArea = 6000; //area to detect obj

        cam = new PropPipeline(
                lower,
                upper,
                () -> minArea,
                () -> 250, // left div. line
                () -> 700 // right div. line
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
                            intake.angleServoDown();
                        }).waitSeconds(1).lineToLinearHeading(new Pose2d(-53, -14, Math.toRadians(311)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).back(2).strafeLeft(4).lineToLinearHeading(new Pose2d(-36, -9, Math.toRadians(0)))
                        .strafeLeft(4)
                        .strafeRight(4)
                        .forward(26)
                        .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                            intake.angleServoUp();
                        }).UNSTABLE_addTemporalMarkerOffset(2.6, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(49, -37)).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).back(4).strafeTo(leftAlign).turn(Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(parkLeft, Math.toRadians(180)))
                        .build();
                break;
            case MIDDLE:
                traj = drive.trajectorySequenceBuilder(start)
                        .strafeLeft(5)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).waitSeconds(1).lineToLinearHeading(new Pose2d(-51, -20, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).waitSeconds(1).back(6).strafeLeft(4).lineToLinearHeading(new Pose2d(-45, -7, Math.toRadians(367)))
                        .forward(42)
                        .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                            intake.angleServoUp();
                        }).UNSTABLE_addTemporalMarkerOffset(2.6, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(49, -38)).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            intake.angleServoUp();
                        }).back(4).strafeTo(leftAlign).turn(Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(parkLeft, Math.toRadians(180)))
                        .build();
                break;
            case RIGHT:
                traj = drive.trajectorySequenceBuilder(start)
                        .build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
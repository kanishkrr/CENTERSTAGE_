package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.subsystems.Claw;
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionMechanism;

@Autonomous
public class BlueClose extends LinearOpMode {
    Pose2d start;
    Vector2d parkLeft, leftAlign;
    ExtensionMechanism extension;
    Claw claw;
    TrajectorySequence left, center, right;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        parkLeft = new Vector2d(55, 58);
        leftAlign = new Vector2d(37, 58);
        start = new Pose2d(14, 61, Math.toRadians(-90));
        drive.setPoseEstimate(start);

        extension = new ExtensionMechanism(hardwareMap);
        claw = new Claw(hardwareMap);

        claw.changeAngleState(Claw.Mode.REST);
        claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.BOTH);
        claw.update(extension.getArmCurrent());

        extension.updateState(ExtensionMechanism.Mode.CUSTOM);
        extension.setArmTarget(120);

        left = drive.trajectorySequenceBuilder(start)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extension.setArmTarget(265);
                    claw.changeAngleState(Claw.Mode.SCORING);
                }).UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    extension.setSlideTarget(-1000);
                }).lineToLinearHeading(new Pose2d(41, 38, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    extension.setSlideTarget(0);
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    extension.setArmTarget(800);
                }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    extension.setArmTarget(1180);
                    claw.changeAngleState(Claw.Mode.STRAIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(1.85, () -> {
                    extension.setArmTarget(1370);
                }).waitSeconds(0.3).splineToConstantHeading(new Vector2d(43, 16), Math.toRadians(0)).waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setClawState(Claw.Mode.WIDE, Claw.Mode.LEFT);
                }).forward(1).build();


        while (opModeInInit()) {
            extension.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(left);


        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            extension.update();
            claw.update(extension.getArmCurrent());
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
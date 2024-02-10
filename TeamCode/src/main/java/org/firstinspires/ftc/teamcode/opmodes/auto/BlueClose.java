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
        extension.updateState(ExtensionMechanism.Mode.CUSTOM);

        claw.update(extension.getArmCurrent());

        left = drive.trajectorySequenceBuilder(start)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extension.setArmTarget(265);
                    claw.changeAngleState(Claw.Mode.SCORING);
                }).UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    extension.setSlideTarget(-1000);
                }).lineToLinearHeading(new Pose2d(41, 34.8, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(1).UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    extension.updateState(ExtensionMechanism.Mode.HOLD);
                    claw.changeAngleState(Claw.Mode.REST);
                }).waitSeconds(1).turn(Math.toRadians(-135)).UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    extension.updateState(ExtensionMechanism.Mode.FLAT);
                    claw.changeAngleState(Claw.Mode.FLAT);
                }).UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    claw.setClawState(Claw.Mode.WIDE, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(5, () -> {
                    extension.updateState(ExtensionMechanism.Mode.HOLD);
                    claw.changeAngleState(Claw.Mode.REST);
                }).build();


        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(left);

        while (opModeInInit()) {
            extension.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            extension.update();
            claw.update(extension.getArmCurrent());

            telemetry.addData("poseHeading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
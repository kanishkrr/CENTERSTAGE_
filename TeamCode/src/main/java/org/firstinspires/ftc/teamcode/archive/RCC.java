package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Claw;
import org.firstinspires.ftc.teamcode.opmodes.teleop.RobotH;
import org.firstinspires.ftc.teamcode.archive.rr.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.archive.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.archive.rr.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous
public class RCC extends LinearOpMode {


    RobotH robot;

    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robot = new RobotH(hardwareMap);

        Globals.IS_AUTO = true;
        Globals.IS_BLUE = false;
        Globals.IS_CLOSE = true;

        robot.init();



        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringArchive.YELLOW_PIXEL_POSITIONS[5])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.5).lineToLinearHeading(new Pose2d(42.4, -37.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.extension.setArmTarget(1160);
                }).waitSeconds(0.7).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.changeAngleState(Claw.Mode.LINED);
                }).strafeLeft(12).setReversed(true).splineToConstantHeading(new Vector2d(30, -6), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.extension.setArmTarget(1360);
                    robot.claw.changeAngleState(Claw.Mode.STRAIGHT);
                }).setReversed(true).splineToConstantHeading(new Vector2d(-24, -6), Math.toRadians(0))
                .setReversed(true).splineToConstantHeading(new Vector2d(-38, -8.5), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.extension.setArmTarget(1470);
                    robot.whitePixelExtendCommand();
                    robot.claw.changeAngleState(Claw.Mode.STRAIGHT);
                    robot.claw.setClawState(Claw.Mode.WIDE, Claw.Mode.BOTH);
                }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.BOTH);
                }).UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    robot.slideRetractCommand();
                    robot.extension.setArmTarget(1360);
                }).waitSeconds(2.6).splineToConstantHeading(new Vector2d(30, -4), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.whitePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.whitePixelExtendCommand();
                }).lineToLinearHeading(new Pose2d(42.8, -36, Math.toRadians(0))).waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.WIDE, Claw.Mode.BOTH);
                }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.whitePixelRetractCommand();
                }).waitSeconds(2).back(3).build();

        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringArchive.YELLOW_PIXEL_POSITIONS[4])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.5).lineToLinearHeading(new Pose2d(37, -25.3, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).build();

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringArchive.YELLOW_PIXEL_POSITIONS[3])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.8).lineToLinearHeading(new Pose2d(21.5, -39.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).build();




        while (opModeInInit()) {
            robot.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(right);

        while (!isStopRequested() && opModeIsActive()) {
            robot.update();
        }

        PoseStorage.currentPose = robot.drive.getPoseEstimate();
    }
}
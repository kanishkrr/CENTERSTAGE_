package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Claw;
import org.firstinspires.ftc.teamcode.opmodes.teleop.RobotH;
import org.firstinspires.ftc.teamcode.archive.rr.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous
public class BCP extends LinearOpMode {


    RobotH robot;

    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robot = new RobotH(hardwareMap);

        Globals.IS_AUTO = true;
        Globals.IS_BLUE = true;
        Globals.IS_CLOSE = true;

        robot.init();



        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringArchive.YELLOW_PIXEL_POSITIONS[0])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(2.5).lineToLinearHeading(new Pose2d(43, 27.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.resetCommand();
                }).lineToLinearHeading(new Pose2d(48, 59.5, Math.toRadians(179))).back(6).build();

        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringArchive.YELLOW_PIXEL_POSITIONS[1])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.5).lineToLinearHeading(new Pose2d(37, 16.3, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.resetCommand();
                }).lineToLinearHeading(new Pose2d(48, 59.5, Math.toRadians(179))).back(6).build();

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringArchive.YELLOW_PIXEL_POSITIONS[2])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.8).lineToLinearHeading(new Pose2d(21.5, 27.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.resetCommand();
                }).lineToLinearHeading(new Pose2d(48, 59.5, Math.toRadians(179))).back(6).build();




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
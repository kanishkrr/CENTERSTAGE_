package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.common.centerstage.Globals;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.hardware.Claw;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous
public class BlueCloseCycle extends LinearOpMode {


    RobotHardware robot;

    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }




        robot = new RobotHardware(hardwareMap);

        Globals.IS_AUTO = true;
        Globals.IS_BLUE = true;
        Globals.IS_CLOSE = true;

        robot.init();


        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(new Pose2d(40.2, 39, Math.toRadians(0)))
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
                }).lineToLinearHeading(new Pose2d(40.2, 33, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.5).lineToLinearHeading(new Pose2d(37, 16.3, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.5).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.changeAngleState(Claw.Mode.LINED);
                }).setReversed(true).splineToConstantHeading(new Vector2d(30, 5), Math.toRadians(0))
                .setReversed(true).splineToConstantHeading(new Vector2d(14, 5), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.cycleExtendCommand();
                    robot.claw.setClawState(Claw.Mode.WIDE, Claw.Mode.RIGHT);
                }).splineToConstantHeading(new Vector2d(-24, 4), Math.toRadians(0))
                .setReversed(true).splineToConstantHeading(new Vector2d(-36, 3.6), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.RIGHT);
                }).splineToConstantHeading(new Vector2d(30, 5), Math.toRadians(0)).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.7, () -> {
                    robot.yellowPixelExtendCommand();
                }).build();

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(new Pose2d(40.7, 27, Math.toRadians(0)))
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

        robot.drive.followTrajectorySequenceAsync(center);

        while (!isStopRequested() && opModeIsActive()) {
            robot.update();
        }

        PoseStorage.currentPose = robot.drive.getPoseEstimate();
    }
}

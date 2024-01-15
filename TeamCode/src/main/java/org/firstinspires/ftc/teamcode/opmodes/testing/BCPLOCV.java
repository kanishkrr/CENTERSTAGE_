package org.firstinspires.ftc.teamcode.opmodes.testing;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.rr.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.common.vision.STATE;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;

@Autonomous
public class BCPLOCV extends LinearOpMode {
    PropPipeline.PropPositions pos;
    Pose2d start;
    Vector2d parkLeft, leftAlign;
    Arm arm;
    Intake intake;
    TrajectorySequence traj;
    Camera cam;
    @Override
    public void runOpMode() {
        //setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        cam = new Camera(hardwareMap, STATE.BLUE, STATE.CLOSE);
        arm = new Arm(hardwareMap);
        arm.setP(0.23);
        intake = new Intake(hardwareMap);
        intake.initServos();
        parkLeft = new Vector2d(55, 59.6);
        leftAlign = new Vector2d(37, 59.6);
        start = new Pose2d(14, 61, Math.toRadians(-90));
        drive.setPoseEstimate(start);

        while (!opModeIsActive()) {
            pos = cam.getRecog();

            if (pos == PropPipeline.PropPositions.LEFT || pos == PropPipeline.PropPositions.MIDDLE) {
                break;
            }
        }

        switch(pos) {
            case LEFT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(44.9, 36.2, Math.toRadians(0)))
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).back(3).turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(35.7, 24.5, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).turn(Math.toRadians(5)).splineToConstantHeading(leftAlign, Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoUp();
                        }).splineToConstantHeading(parkLeft, Math.toRadians(180)).build();
                break;
            case MIDDLE:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(44.9, 29.4, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseSecondPixel();
                        }).back(3).turn(Math.toRadians(180)).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(26, 19.3, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).turn(Math.toRadians(5)).splineToConstantHeading(leftAlign, Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.initServos();
                            intake.angleServoUp();
                        }).splineToConstantHeading(parkLeft, Math.toRadians(180)).build();
                break;
            case RIGHT:
                traj = drive.trajectorySequenceBuilder(start)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.angleServoMiddle();
                        }).lineToLinearHeading(new Pose2d(45.1, 25.8, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            intake.releaseSecondPixel();
                        }).back(3).turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            intake.angleServoDown();
                        }).lineToLinearHeading(new Pose2d(12.5, 26.5, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            intake.releaseFirstPixel();
                        }).turn(Math.toRadians(5)).splineToConstantHeading(leftAlign, Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            intake.initServos();
                            intake.angleServoUp();
                        }).splineToConstantHeading(parkLeft, Math.toRadians(180)).build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(traj);

    }

}
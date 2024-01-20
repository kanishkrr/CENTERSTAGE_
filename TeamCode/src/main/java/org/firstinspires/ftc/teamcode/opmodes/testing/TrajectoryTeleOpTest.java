package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
@Config
@TeleOp(group = "TeleOpRR")
public class TrajectoryTeleOpTest extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Pose2d targetPose, pickUpPose;
    Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        targetPose = new Pose2d(44.9, 36.2, Math.toRadians(180));
        pickUpPose = new Pose2d(-52,-52, Math.toRadians(225));

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y*0.75,
                                    -gamepad1.left_stick_x*0.75,
                                    -gamepad1.right_stick_x*0.75
                            )
                    );
                    if (gamepad1.right_bumper) {
                        Trajectory toBoard = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(targetPose)
                                .build();

                        drive.followTrajectoryAsync(toBoard);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    if (gamepad1.left_bumper) {
                        Trajectory toHuman = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(targetPose)
                                .build();

                        drive.followTrajectoryAsync(toHuman);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}
package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;

@TeleOp(group = "advancedrrteleop")
public class TeleOpRRTest extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;

    Vector2d targetAVector = new Vector2d(45, 45);
    double targetAHeading = Math.toRadians(90);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(55, 59.6, Math.toRadians(180)));

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
                    if (gamepad1.a) {
                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);
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
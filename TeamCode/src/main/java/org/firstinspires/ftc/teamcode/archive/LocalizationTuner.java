package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.archive.rr.drive.SampleMecanumDrive;
@Config
@TeleOp(group = "Localization")
public class LocalizationTuner extends LinearOpMode {

    public static double startX = 0;
    public static double startY = 0;
    public static double startHeading = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d start = new Pose2d(startX, startY, Math.toRadians(startHeading));
        drive.setPoseEstimate(start);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*0.75,
                            -gamepad1.left_stick_x*0.75,
                            -gamepad1.right_stick_x*0.75
                    )
            );
        }
    }
}
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.common.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.subsystems.Claw;
import org.firstinspires.ftc.teamcode.common.subsystems.DroneRelease;
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionSystem;

@TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends LinearOpMode {

    ExtensionSystem extension;
    Claw claw;
    Actuator actuator;
    DroneRelease drone;
    SampleMecanumDrive drive;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Pose2d backBoardPose, pickUpPose, poseEstimate;
    Mode currentMode = Mode.DRIVER_CONTROL;
    double driveMultiplier = 0.75;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize all subsystems
        drive = new SampleMecanumDrive(hardwareMap);
        extension = new ExtensionSystem(hardwareMap);
        claw = new Claw(hardwareMap);
        actuator = new Actuator(hardwareMap);
        drone = new DroneRelease(hardwareMap);

        //create set poses on the field that the drive might have to head towards
        backBoardPose = new Pose2d(44.9, 34, Math.toRadians(180));
        pickUpPose = new Pose2d(-52,-52, Math.toRadians(225));

        //final drive setup
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            poseEstimate = drive.getPoseEstimate();

            switch (currentMode) {
                case DRIVER_CONTROL:

                    //drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

                    /*

                    if (gamepad1.b && gamepad1.dpad_left) {
                        Trajectory toBoard = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(backBoardPose)
                                .build();

                        drive.followTrajectoryAsync(toBoard);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                    if (gamepad1.x) {
                        Trajectory toHuman = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(pickUpPose)
                                .build();

                        drive.followTrajectoryAsync(toHuman);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;

                     */

                case AUTOMATIC_CONTROL:
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}
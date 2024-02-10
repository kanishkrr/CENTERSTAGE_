package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.common.subsystems.Claw;
import org.firstinspires.ftc.teamcode.common.subsystems.DroneRelease;
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionMechanism;

@TeleOp
public class TestingTele extends LinearOpMode {

    //creating all subsystems
    ExtensionMechanism extension;
    Claw claw;
    Actuator actuator;
    DroneRelease drone;
    SampleMecanumDrive drive;

    //initialize time
    ElapsedTime runtime = new ElapsedTime();
    double driveMultiplier = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize all subsystems
        drive = new SampleMecanumDrive(hardwareMap);
        extension = new ExtensionMechanism(hardwareMap);
        claw = new Claw(hardwareMap);
        actuator = new Actuator(hardwareMap);
        drone = new DroneRelease(hardwareMap);

        //sets drive mode + localizes
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        //init drone
        drone.init();

        //initialize arm start pos
        extension.updateState(ExtensionMechanism.Mode.HOLD);

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            extension.update();
            claw.update(extension.getArmCurrent());

            //gamepad2 functions
            if (gamepad2.x) {
                extension.updateState(ExtensionMechanism.Mode.FLAT);
                claw.changeAngleState(Claw.Mode.FLAT);
                claw.setClawState(Claw.Mode.WIDE, Claw.Mode.BOTH);
            }

            if (gamepad2.y) {
                claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.BOTH);
                extension.updateState(ExtensionMechanism.Mode.HOLD);
                claw.changeAngleState(Claw.Mode.REST);
            }

            if (gamepad2.a) {
                extension.updateState(ExtensionMechanism.Mode.SCORING);
                claw.changeAngleState(Claw.Mode.SCORING);
            }

            if (gamepad2.right_bumper) {
                claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
            }

            if (gamepad2.left_bumper) {
                claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
            }

            if (gamepad2.right_trigger > 0.5) {
                claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.RIGHT);
            }

            if (gamepad2.left_trigger > 0.5) {
                claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.LEFT);
            }

            if (gamepad2.left_stick_y > 0.15 || gamepad2.left_stick_y < -0.15 ) {
                extension.updateState(ExtensionMechanism.Mode.CUSTOM);

                double y = -gamepad2.left_stick_y;

                double target = extension.getArmTarget()+y*20;

                double currentPos = extension.getArmCurrent();

                if (Math.abs(target - currentPos) > 120) {
                    if (target > currentPos) {
                        target = currentPos + 50;
                    } else {
                        target = currentPos - 50;
                    }
                }

                extension.setArmTarget(target);
            }

            if (gamepad2.right_stick_y > 0.15 || gamepad2.right_stick_y < -0.15) {
                extension.updateState(ExtensionMechanism.Mode.CUSTOM);

                double y = gamepad2.right_stick_y;

                double target = extension.getSlideTarget() + y*35;

                if (target > 0) {
                    target = 0;
                } else if (target < -1990) {
                    target = -1990;
                }

                extension.setSlideTarget(target);
            }

            if (gamepad2.dpad_up) {
                actuator.setPower(0.8);
            } else if (gamepad2.dpad_down) {
                actuator.setPower(-0.8);
            } else {
                actuator.setPower(0);
            }

            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                drone.release();
            }

            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveMultiplier, -gamepad1.left_stick_x*driveMultiplier, -gamepad1.right_stick_x*driveMultiplier));

            driveMultiplier = 0.4;

            double leftTrig = gamepad1.left_trigger;
            double rightTrig = gamepad1.right_trigger;

            if (rightTrig > 0.1 && leftTrig > 0.1) {

                double avg = (leftTrig+rightTrig) / 2;

                driveMultiplier = (avg*0.6) + 0.4;
            }

            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                driveMultiplier = 0.25;
            }


            telemetry.addData("slide current:", extension.getSlideCurrent());
            telemetry.addData("arm real current:", extension.armEncoder.getCurrentPosition());
            telemetry.addData("arm real target:", extension.armTarget);
            telemetry.addData("arm current:", extension.getArmCurrent());
            telemetry.addData("gamepad right stick y", gamepad2.right_stick_y);
            telemetry.addData("gamepad left stick y", gamepad2.left_stick_y);
            telemetry.addData("robot x:", drive.getPoseEstimate().getX());
            telemetry.addData("robot y:", drive.getPoseEstimate().getY());
            telemetry.addData("robot heading:", drive.getPoseEstimate().getHeading());
            telemetry.update();

        }
    }
}
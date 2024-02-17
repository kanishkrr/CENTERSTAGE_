package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.centerstage.Globals;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.Actuator;
import org.firstinspires.ftc.teamcode.common.hardware.Claw;
import org.firstinspires.ftc.teamcode.common.hardware.DroneRelease;
import org.firstinspires.ftc.teamcode.common.hardware.ExtensionMechanism;
import org.firstinspires.ftc.teamcode.opmodes.testing.RobotHardwareTest;

@TeleOp
public class DuoTest extends LinearOpMode {

    RobotHardwareTest robot;
    double driveMultiplier;

    @Override
    public void runOpMode() {

        Globals.IS_AUTO = false;

        robot = new RobotHardwareTest(hardwareMap);

        robot.init();

        while (opModeInInit()) {
            robot.update();
        }

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            //gamepad2 functions
            if (gamepad2.x) {
                robot.pixelPickupCommand();
            }

            if (gamepad2.y) {
                robot.holdCommand();
            }

            if (gamepad2.a) {
                robot.scoreCommand();
            }

            robot.clawOpenCommand(gamepad2.left_bumper, gamepad2.right_bumper);

            robot.clawCloseCommand(gamepad2.left_trigger, gamepad2.right_trigger);



            if (gamepad2.left_stick_y > 0.15 || gamepad2.left_stick_y < -0.15 ) {
                robot.gamepadCommand();

                double y = -gamepad2.left_stick_y;
                double target = robot.extension.armTarget+y*35;
                double currentPos = robot.extension.armCurrent;

                if (Math.abs(target - currentPos) > 120) {
                    if (target > currentPos) {
                        target = currentPos + 50;
                    } else {
                        target = currentPos - 50;
                    }
                }

                robot.extension.setArmTarget(target);
            }

            if (gamepad2.right_stick_y > 0.15 || gamepad2.right_stick_y < -0.15) {
                robot.gamepadCommand();

                double y = gamepad2.right_stick_y;
                double target = robot.extension.slideTarget + y*35;
                if (target > 0) {
                    target = 0;
                } else if (target < -1990) {
                    target = -1990;
                }

                robot.extension.setSlideTarget(target);
            }

            if (gamepad2.dpad_up) {
                robot.hangPower(0.8);
            } else if (gamepad2.dpad_down) {
                robot.hangPower(-0.8);
            } else {
                robot.hangPower(0);
            }

            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                robot.releaseDrone();
            }

            //drive commands
            robot.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveMultiplier, -gamepad1.left_stick_x*driveMultiplier, -gamepad1.right_stick_x*driveMultiplier));
            double LT = gamepad1.left_trigger;
            if (LT > 0.05) {
                driveMultiplier = (LT * 0.75) + 0.25;
            } else {
                driveMultiplier = 0.25;
            }

            telemetry.addData("slide current:", robot.extension.slideCurrent);
            telemetry.addData("slide target:", robot.extension.slideTarget);
            telemetry.addData("arm target:", robot.extension.armTarget);
            telemetry.addData("arm current:", robot.extension.armCurrent);
            telemetry.addData("gamepad right stick y", gamepad2.right_stick_y);
            telemetry.addData("gamepad left stick y", gamepad2.left_stick_y);
            telemetry.addData("robot x:", robot.drive.getPoseEstimate().getX());
            telemetry.addData("robot y:", robot.drive.getPoseEstimate().getY());
            telemetry.addData("robot heading:", robot.drive.getPoseEstimate().getHeading());
            telemetry.update();

        }
    }
}
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
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionMechanism;
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionSystem;

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

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize all subsystems
        drive = new SampleMecanumDrive(hardwareMap);
        extension = new ExtensionMechanism(hardwareMap);
        claw = new Claw(hardwareMap);
        actuator = new Actuator(hardwareMap);

        /*
        drone = new DroneRelease(hardwareMap); //still need to create functions for this class

         */

        //sets drive mode + localizes
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        //initialize arm start pos
        extension.setArmTarget(140);

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            extension.update();
            claw.update(extension.getArmCurrent());

            //gamepad2 functions
            if (gamepad2.x) {
                extension.setSlideTarget(-780);
                extension.setArmTarget(95);
                claw.updateState(Claw.Mode.FLAT);
            }

            if (gamepad2.y) {
                if (extension.getSlideCurrent() > -300) {
                    extension.setArmTarget(140);
                }
                if (extension.getArmCurrent() < 150) {
                    extension.setSlideTarget(0);
                }
                claw.updateState(Claw.Mode.REST);
            }

            if (gamepad2.b) {
                extension.setSlideTarget(0);
            }

            /*

            if mode is flat
            don't extend slide until arm is in right position

            if mode is reset
            dont reset arm until slide is reset

            if mode is custom
            dont extend slide until arm is in right position

             */
            if (gamepad2.a) {
                extension.setSlideTarget(-1200);
                extension.setArmTarget(330);
                claw.updateState(Claw.Mode.SCORING);
            }

            if (gamepad2.left_stick_y > 0.15 || gamepad2.left_stick_y < -0.15 ) {

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


            telemetry.addData("slide current:", extension.getSlideCurrent());
            telemetry.addData("arm real current:", extension.armEncoder.getCurrentPosition());
            telemetry.addData("arm real target:", extension.armTarget);
            telemetry.addData("arm current:", extension.getArmCurrent());
            telemetry.addData("gamepad right stick y", gamepad2.right_stick_y);
            telemetry.addData("gamepad left stick y", gamepad2.left_stick_y);
            telemetry.update();

        }
    }
}
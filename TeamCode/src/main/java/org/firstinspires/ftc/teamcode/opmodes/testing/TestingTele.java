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
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionSystem;

@TeleOp
public class TestingTele extends LinearOpMode {

    //creating all subsystems
    ExtensionSystem extension;
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
        extension = new ExtensionSystem(hardwareMap);
        claw = new Claw(hardwareMap);
        actuator = new Actuator(hardwareMap);

        /*
        drone = new DroneRelease(hardwareMap); //still need to create functions for this class

         */

        //sets drive mode + localizes
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        //initialize arm start pos
        extension.setArmTarget(100);

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            //gets the current time since start
            double currentTime = runtime.seconds();

            //records all arm+slide positions for telemetry
            int currentArmPos = extension.getArmCurrent();
            double targetArmPos = extension.getArmTarget();
            int currentSlidePos = extension.getSlideCurrent();
            double targetSlidePos = extension.getSlideTarget();

            //updates telemtry w/ accurate data
            telemetry.addData("current arm position: ", currentArmPos);
            telemetry.addData("current arm target position:", targetArmPos);
            telemetry.addData("current slide position: ", currentSlidePos);
            telemetry.addData("current slide target position:", targetSlidePos);
            telemetry.update();

            //gamepad2 functions
            if (gamepad2.x) {
                extension.setSlideTarget(-780);
                extension.setArmTarget(85);
                claw.alignWithGround();
            }

            if (gamepad2.y) {
                extension.setSlideTarget(0);
                extension.setArmTarget(120);
                claw.resetPosition();
            }

            if (gamepad2.a) {
                extension.setSlideTarget(-1200);
                extension.setArmTarget(270);
            }

            if (gamepad2.left_stick_y > 0.1) {
                extension.setArmTarget(extension.getArmCurrent()+gamepad2.left_stick_y);
            } else if (gamepad2.left_stick_y < -0.1) {
                extension.setArmTarget(extension.getArmCurrent()-gamepad2.left_stick_y);

            }

            //updates extension w/ new target positions for slide + arm
            extension.update();

        }
    }
}
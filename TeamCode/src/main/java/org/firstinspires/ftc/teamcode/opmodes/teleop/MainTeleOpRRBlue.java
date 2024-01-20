package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.subsystems.Arm;

@TeleOp(name = "MainTeleOpRRBlue")
public class MainTeleOpRRBlue extends LinearOpMode {
    Arm arm;
    DcMotor Actuator, LB, LF, RB, RF;
    Servo rightServo, leftServo, angleServo1, angleServo2, planeServo;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Pose2d backBoardPose, pickUpPose, poseEstimate;
    Mode currentMode = Mode.DRIVER_CONTROL;
    double driveMultiplier = 0.75;

    public void telemetryUpdate() {
        telemetry.addData("mode", currentMode);
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm  = new Arm(hardwareMap);
        RB = hardwareMap.dcMotor.get("Left_Back_Motor");
        LB = hardwareMap.dcMotor.get("Right_Back_Motor");
        RF = hardwareMap.dcMotor.get("Left_Front_Motor");
        LF = hardwareMap.dcMotor.get("Right_Front_Motor");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
        angleServo1 = hardwareMap.get(Servo.class, "servo1");
        angleServo2 = hardwareMap.get(Servo.class, "servo5");
        planeServo = hardwareMap.get(Servo.class, "servo4");
        Actuator = hardwareMap.dcMotor.get("Actuator_Motor");

        backBoardPose = new Pose2d(44.9, 34, Math.toRadians(180));
        pickUpPose = new Pose2d(-52,-52, Math.toRadians(225));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            poseEstimate = drive.getPoseEstimate();

            telemetryUpdate();

            if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                arm.setP(0.2);
            }
            if (gamepad2.right_trigger == 1) {
                arm.setP(0.55);
            }
            if (gamepad2.right_stick_y>0.8) {
                arm.setP(-1.00);
            }
            if (gamepad2.right_stick_y<-0.8) {
                arm.setP(1.00);
            }

            if (gamepad2.left_trigger == 1) {
                arm.setP(-0.35);
            }
            if (gamepad2.y) {
                arm.setP(-0.13);
            }

            if (gamepad2.a) {
                leftServo.setPosition(0.3);
                rightServo.setPosition(0.3);
            }

            if (gamepad2.b) {
                leftServo.setPosition(-1);
                rightServo.setPosition(0.75);
            }

            if (gamepad2.dpad_right) {
                rightServo.setPosition(0.3);
            }

            if (gamepad2.dpad_left) {
                leftServo.setPosition(0.3);
            }

            if (gamepad2.left_bumper) {
                angleServo1.setPosition(0);
                angleServo2.setPosition(1);
            }

            if (gamepad2.right_bumper) {
                angleServo1.setPosition(0.73);
                angleServo2.setPosition(0.27);
                //servoAngle =0.00;
            }

            if(gamepad2.x) {
                angleServo1.setPosition(0.3);
                angleServo2.setPosition(0.7);
            }

            if (gamepad2.y && gamepad2.dpad_down && gamepad2.left_trigger == 1) {
                planeServo.setPosition(0.4);
            }

            if (gamepad2.left_stick_y>0.1 || gamepad2.left_stick_y<-0.1) {
                Actuator.setPower(-gamepad2.left_stick_y);
            }
            else {
                Actuator.setPower(0);
            }


            switch (currentMode) {
                case DRIVER_CONTROL:
                    if (gamepad1.left_stick_x > 0 && gamepad1.right_stick_x > 0){
                        LB.setPower(gamepad1.right_stick_x*driveMultiplier);
                        LF.setPower(-gamepad1.right_stick_x*driveMultiplier);
                        RB.setPower(gamepad1.left_stick_x*driveMultiplier);
                        RF.setPower(-gamepad1.left_stick_x*driveMultiplier);
                    }

                    //right
                    if (gamepad1.left_stick_x < 0 && gamepad1.right_stick_x < 0){
                        LB.setPower(gamepad1.right_stick_x*driveMultiplier);
                        LF.setPower(-gamepad1.right_stick_x*driveMultiplier);
                        RB.setPower(gamepad1.left_stick_x*driveMultiplier);
                        RF.setPower(-gamepad1.left_stick_x*driveMultiplier);
                    }

                    //Changing motor powers
                    if (gamepad1.a) {
                        driveMultiplier = 0.75;
                    }

                    if (gamepad1.b) {
                        driveMultiplier = 1.00;
                    }

                    if (gamepad1.y) {
                        driveMultiplier = 0.5;
                    }

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

                case AUTOMATIC_CONTROL:
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}
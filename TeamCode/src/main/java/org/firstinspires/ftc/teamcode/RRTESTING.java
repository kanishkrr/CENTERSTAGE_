package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;

@Autonomous
public class RRTESTING extends LinearOpMode {
    DcMotor Arm;
    Servo angleServo1, angleServo2, rightServo, leftServo;
    private static final double servoOpen = 0.3;
    public void angleServoMiddle(){
        angleServo1.setPosition(-0.52);
        angleServo2.setPosition(0.52);
        Arm.setPower(0.23);
    }

    public void moveArmTo(int revs, double power) {
        Arm.setTargetPosition(revs);

        int error = Arm.getTargetPosition()-Arm.getCurrentPosition();

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (error < 0) {
            Arm.setPower(-power);
        } else {
            Arm.setPower(power);
        }

        while (Arm.isBusy()) {
            System.out.println(Arm.getCurrentPosition());
        }

        Arm.setPower(0.0);
    }

    public void initServos() {
        leftServo.setPosition(-1);
        rightServo.setPosition(0.75);
    }

    public void releaseFirstPixel() {
        rightServo.setPosition(servoOpen);
    }

    public void releaseSecondPixel() {
        leftServo.setPosition(servoOpen);
    }

    public void angleServoDown() {
        angleServo1.setPosition(-0.99);
        angleServo2.setPosition(1);
    }
    public void angleServoUp() {
        angleServo1.setPosition(1.00);
        angleServo2.setPosition(-0.99);
    }

    public void initMotors() {
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode() {

        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        angleServo1 = hardwareMap.get(Servo.class, "servo1");
        angleServo2 = hardwareMap.get(Servo.class, "servo5");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");

        initServos();
        initMotors();

        //left Pose2d yellow = new Pose2d(49, 40, Math.toRadians(0)) - Pose2d purple = new Pose2d(33, 28, Math.toRadians(180));

        Pose2d yellow = new Pose2d(49, 40, Math.toRadians(0));
        Pose2d purple = new Pose2d(33, 28, Math.toRadians(180));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(yellow)
                .addTemporalMarker(1, () -> {
                    moveArmTo(65, 0.27);
                    angleServoMiddle();
                })
                .addTemporalMarker(2.2, () -> {
                    Arm.setPower(0.21);
                })
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(purple)
                .addTemporalMarker(2.5, () -> {
                    moveArmTo(115, 0.32);
                    angleServoDown();
                })
                .addTemporalMarker(4, () -> {
                    moveArmTo(95, 0.3);
                })
                .addTemporalMarker(4.9, () -> {
                    releaseFirstPixel();
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(() -> {
                    moveArmTo(100, 0.33);
                })
                .splineTo(new Vector2d(4, 4), Math.toRadians(-90))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj);

        releaseSecondPixel();

        drive.followTrajectory(traj1);

        drive.followTrajectory(traj2);

    }
}
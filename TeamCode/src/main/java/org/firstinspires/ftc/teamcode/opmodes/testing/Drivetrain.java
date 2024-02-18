package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.rr.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {

    DcMotorEx leftFront, leftRear, rightFront, rightRear;
    StandardTrackingWheelLocalizer localizer;

    Pose current, target;

    public Drivetrain(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");
        leftRear = hardwareMap.get(DcMotorEx.class, "Left_Back_Motor");
        rightRear = hardwareMap.get(DcMotorEx.class, "Right_Back_Motor");
        rightFront = hardwareMap.get(DcMotorEx.class, "Right_Front_Motor");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }

    public void setPoseEstimate(Pose pose) {
        localizer.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), pose.getHeading()));
    }

    public void setPowers(double p1, double p2, double p3, double p4) {
        leftFront.setPower(p1);
        leftRear.setPower(p2);
        rightFront.setPower(p3);
        rightRear.setPower(p4);
    }
}

package org.firstinspires.ftc.teamcode.common.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Vector2D;

public class Drivetrain extends SubsystemBase {

    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightFront;
    DcMotorEx rightRear;

    public Localizer localizer;

    public Drivetrain(HardwareMap hardwareMap) {

        this.leftFront = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "Left_Back_Motor");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "Right_Back_Motor");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "Right_Front_Motor");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.localizer = new Localizer(hardwareMap);

    }

    @Override
    public void periodic() {
        localizer.update();
    }

    public void set(Pose pose) {
        set(pose, 0);
    }

    public void set(double strafeSpeed, double forwardSpeed,
                    double turnSpeed, double gyroAngle) {

        Vector2D input = new Vector2D(strafeSpeed, forwardSpeed).rotate(-gyroAngle);

        strafeSpeed = Range.clip(input.x, -1, 1);
        forwardSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);


        double frontLeft = forwardSpeed + strafeSpeed + turnSpeed;
        double frontRight = forwardSpeed - strafeSpeed - turnSpeed;
        double backLeft = (forwardSpeed - strafeSpeed + turnSpeed);
        double backRight = (forwardSpeed + strafeSpeed - turnSpeed);

        setPowers(frontLeft, backLeft, frontRight, backRight);

    }

    public void set(Pose pose, double angle) {
        set(pose.x, pose.y, pose.heading, angle);
    }

    public void setPowers(double lf, double lr, double rf, double rr) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }
}

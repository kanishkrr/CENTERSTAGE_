package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ExtensionSystem {
    private PIDController armController, slideController;

    double f; //Figure out this value
    public static double armTarget, slideTarget = 0;
    public final double ticks_in_degree = 537;
    private DcMotor arm, slide, armEncoder;
    public  ExtensionSystem(HardwareMap hMap){
        //create pid controllers, one for arm and one for viper slide
        armController = new PIDController(0.01311, 0, 0.0012199); //input these values pArm = 0.0164, iArm = 0.03, dArm = 0.0018
        slideController = new PIDController(0.0029, 0, 0.000165); //input these values pSlide = 0.00171, iSlide = 0, dSlide = 0.000165

        //initialize all motors
        arm = hMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hMap.get(DcMotorEx.class, "ViperSlide");
        armEncoder = hMap.get(DcMotorEx.class, "Left_Front_Motor");

        //set settings form motors + encoders
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        f = 0.56;
    }

    public void update() {
        //temp variable
        double tempTarget = 0;

        //calculations for arm
        int armPos = getArmCurrent();

        if (armTarget - armPos > 50) {
            tempTarget = armPos+50;
        } else if (armTarget - armPos < -50) {
            tempTarget = armPos-50;
        } else {
            tempTarget = armTarget;
        }

        double pid = armController.calculate(armPos, tempTarget);
        double ff = Math.cos(Math.toRadians(tempTarget / ticks_in_degree)) * f;
        double armPower = pid * ff;

        //calculations for slide
        int slidePos = getSlideCurrent();
        double slidePower = slideController.calculate(slidePos, slideTarget);

        //setting power
        arm.setPower(armPower);
        slide.setPower(slidePower);
    }

    public void setArmTarget(double aTarget) {

        //sets the target position for the arm and slide
        armTarget = aTarget;
    }

    public int getArmCurrent() {
        return Math.abs(armEncoder.getCurrentPosition());
    }

    public void setSlideTarget(double sTarget) {
        slideTarget = sTarget;
    }

    public int getSlideCurrent() {
        return arm.getCurrentPosition();
    }

    public double getArmTarget() {
        return armTarget;
    }

    public double getSlideTarget() {
        return slideTarget;
    }

}
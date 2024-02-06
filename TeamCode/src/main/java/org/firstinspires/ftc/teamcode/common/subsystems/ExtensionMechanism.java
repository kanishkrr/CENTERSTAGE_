package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ExtensionMechanism {
    private PIDController armController, slideController;
    public static double pArm = 0.0129, iArm = 0, dArm = 0.0012;
    public static double pSlide = 0.0029, iSlide = 0, dSlide = 0.000165;


    public static double f=0.56;
    public double armTarget, slideTarget = 0;
    public final double ticks_in_degree = 537;
    public DcMotor arm, slide, armEncoder;

    public double maxPower = 0.5;
    public double armPow = 0.0;

    public enum Mode {
        FLAT, HOLD, SCORING;
    }

    Mode currentArmState = Mode.HOLD;
    public ExtensionMechanism(HardwareMap hardwareMap){
        armController = new PIDController(pArm, iArm, dArm);
        slideController = new PIDController(pSlide, iSlide, dSlide);

        arm = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hardwareMap.get(DcMotorEx.class, "ViperSlide");
        armEncoder = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");

    }


    public void update() {

        switch(currentArmState) {
            case HOLD:
                setSlideTarget(0);
                if (getArmCurrent() < getArmTarget() || getSlideCurrent() > -600) {
                    setArmTarget(140);
                } else {
                    setArmTarget(getArmCurrent());
                }
                break;
            case FLAT:
                if ()
        }

        //calculations for arm
        int armPos = (int)getArmCurrent();
        double pid = armController.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
        double armPower = pid * ff;

        //calculations for slide
        int slidePos = arm.getCurrentPosition();
        double slidePower = slideController.calculate(slidePos, slideTarget);

        maxPower = Math.abs(((Math.abs(arm.getCurrentPosition()) / 4000) + 0.5));

        if (armPower > maxPower) {
            armPower = maxPower;
        } else if (armPower < -maxPower) {
            armPower = -maxPower;
        }

        arm.setPower(armPower);
        slide.setPower(slidePower);
    }


    public void setArmTarget(double target) {
        armTarget = target;
    }

    public void setSlideTarget(double target) {
        slideTarget = target;
    }

    public double getArmCurrent() {
        return Math.abs(armEncoder.getCurrentPosition());
    }

    public double getSlideCurrent() {
        return arm.getCurrentPosition();
    }

    public double getArmTarget() {
        return armTarget;
    }

    public double getSlideTarget() {
        return slideTarget;
    }

    public void setArmPower(double pow) {
        arm.setPower(pow);
    }

    public void updateState(Mode state) {
        currentArmState = state;
    }

}
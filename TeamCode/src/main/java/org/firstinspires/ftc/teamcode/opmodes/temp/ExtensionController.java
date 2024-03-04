package org.firstinspires.ftc.teamcode.opmodes.temp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


@Config
public class ExtensionController extends SubsystemBase {
    private PIDController armController, slideController;
    public double ARM_CURRENT, SLIDE_CURRENT;
    public static double MAX_ARM_POWER = 0.5;
    public static double MAX_SLIDE_POWER = 1.0;
    public static double ARM_TARGET, SLIDE_TARGET = 0;
    private DcMotor arm, slide, armEncoder;
    public double kG = 3.5;
    public double pastPower;

    public ExtensionController(HardwareMap hardwareMap){
        this.armController = new PIDController(5, 1, 3);
        this.slideController = new PIDController(0.0053, 0.0, 0);

        arm = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hardwareMap.get(DcMotorEx.class, "ViperSlide");
        armEncoder = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {

        ARM_CURRENT = Math.toRadians(90 * (Math.abs(armEncoder.getCurrentPosition())-100)/650);
        SLIDE_CURRENT = Math.abs(arm.getCurrentPosition());

        double pid = armController.calculate(ARM_CURRENT, ARM_TARGET);

        /*
            kG = power needed to sustain at 0 angle, max extension --> need to find this first
            theta = arm angle
            distcog = dist of center of mass from center of rotation
            distcogmax = dist of center of mass from center of rotation at max extension
         */

        double distcog = SLIDE_CURRENT + 600;
        double distcogmax = 2600;

        double f = kG * Math.abs(Math.cos(ARM_CURRENT)) * (distcog/distcogmax);
        double armPower = pid * f;

        double slidePower = slideController.calculate(SLIDE_CURRENT, SLIDE_TARGET);
        slidePower = Range.clip(slidePower, -MAX_SLIDE_POWER, MAX_SLIDE_POWER);

        armPower = Range.clip(armPower, -MAX_ARM_POWER, MAX_ARM_POWER);

        arm.setPower(armPower);
        slide.setPower(slidePower);

        pastPower = armPower;
    }

    public void setArmPID(PIDController controller) {
        armController = controller;
    }

    public void setSlidePID(PIDController controller) {
        slideController = controller;
    }

    public void setKG(double k) {
        kG = k;
    }

    public void setArmTargetPosition(double target) {
        ARM_TARGET = target;
    }

    public void setSlideTargetPosition(double target) {
        SLIDE_TARGET = target;
    }



}
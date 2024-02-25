package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class ExtensionSubsystem extends SubsystemBase {
    /*
    create PID controllers
     */
    private PIDController armController, slideController;
    /*
    all constants associated with running PID
     */
    public double f=0.56;
    public double armTarget, slideTarget = 0;
    public double armCurrent, slideCurrent;
    public final double ticks_in_degree = 537;
    public DcMotor arm, slide, armEncoder;
    public double maxPower = 0.5;


    public ExtensionSubsystem(HardwareMap hardwareMap){
        //sets pid for both arm and slide controllers
        armController = new PIDController(0.0117, 0, 0.00134);
        slideController = new PIDController(0.004, 0, 0.000165);

        //set all motors | name of the motor controls the component | encoders: arm --> slide, armEncoder --> arm
        arm = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hardwareMap.get(DcMotorEx.class, "ViperSlide");
        armEncoder = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    @Override
    public void periodic() {
        armCurrent = Math.abs(armEncoder.getCurrentPosition());
        slideCurrent = arm.getCurrentPosition();

        double pid = armController.calculate(armCurrent, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
        double armPower = pid * ff;

        double slidePower = slideController.calculate(slideCurrent, slideTarget);

        armPower = Range.clip(armPower, -maxPower, maxPower);

        //sets the power
        arm.setPower(armPower);
        slide.setPower(slidePower);
    }

    public void setArmTargetPosition(double target) {
        armTarget = target;
    }

    public void setSlideTargetPosition(double target) {
        slideTarget = target;
    }

    public void setArmPID(PIDController c) {
        armController = c;
    }

    public void setSlidePID(PIDController c) {
        slideController = c;
    }


}
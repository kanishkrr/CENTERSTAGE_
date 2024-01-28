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
@TeleOp
public class ExtensionSystem {
    private PIDController armController, slideController;

    double f; //Figure out this value
    public static int armTarget, slideTarget = 0;
    public final double ticks_in_degree = 537;
    public final double armMaxHeight = 0; //to be figured out
    private DcMotor arm, slide;
    public  ExtensionSystem(HardwareMap hMap){
        //create pid controllers, one for arm and one for viper slide
        armController = new PIDController(0, 0, 0); //input these values
        slideController = new PIDController(0, 0, 0); //input these values

        arm = hMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hMap.get(DcMotorEx.class, "Viper_Slide");

        f = 0;
    }

    public void update() {
        //calculations for arm
        int armPos = arm.getCurrentPosition();
        double pid = armController.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
        double armPower = pid * ff;

        //calculations for slide
        int slidePos = slide.getCurrentPosition();
        double slidePower = slideController.calculate(slidePos, slideTarget);

        //setting power
        arm.setPower(armPower);
        slide.setPower(slidePower);
    }

    public void setTarget(int aTarget, int sTarget) {

        //sets the target position for the arm and slide
        armTarget = aTarget;
        slideTarget = sTarget;
    }

    public void setToPixelPickup() {

    }

    public void setToResetPosition() {

    }

    public void setToBackBoard() {

    }

    public void changeArmHeight(double change) {
        armTarget += (int)(change * 20);
    }

}
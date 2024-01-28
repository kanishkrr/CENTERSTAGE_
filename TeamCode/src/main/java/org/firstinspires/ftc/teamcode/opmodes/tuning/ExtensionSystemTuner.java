package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ExtensionSystemTuner extends OpMode {
    private PIDController armController;
    private PIDController slideController;
    public static double pArm = 0, iArm = 0, dArm = 0; //Tune these values
    public static double pSlide = 0, iSlide = 0, dSlide = 0; //Tune these values

    public static double f=0; //Figure out this value
    public static int armTarget, slideTarget = 0;
    public final double ticks_in_degree = 537; //Constant to figure out
    private DcMotor arm, slide;
    @Override
    public void init(){
        //create pid controllers, one for arm and one for viper slide
        armController = new PIDController(pArm, iArm, dArm);
        slideController = new PIDController(pSlide, iSlide, dSlide);

        //set telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init motors
        arm = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hardwareMap.get(DcMotorEx.class, "Viper_Slide");
    }

    @Override
    public void loop() {
        //re-set pid values
        armController.setPID(pArm, iArm, dArm);
        slideController.setPID(pSlide, iSlide, dSlide);

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

        telemetry.addData("pos", armPos);
        telemetry.addData("target", armTarget);
        telemetry.update();
    }

}
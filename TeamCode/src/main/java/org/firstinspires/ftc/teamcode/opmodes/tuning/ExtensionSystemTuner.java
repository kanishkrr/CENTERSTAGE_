package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.subsystems.Claw;

@Config
@TeleOp
public class ExtensionSystemTuner extends OpMode {
    private PIDController armController, slideController;
    public static double pArm = 0.0129, iArm = 0, dArm = 0.0013; //Tune these values

    /*
    armController = new PIDController(0.0164, 0.03, 0.00112); //input these values pArm = 0.0164, iArm = 0.03, dArm = 0.0018
        slideController = new PIDController(0.00171, 0, 0.000165);
     */
    public static double pSlide = 0.0029, iSlide = 0, dSlide = 0.000165; //Tune these values

    public static double asPos = 0.3;

    public static double f=0.56; //Figure out this value
    public static int armTarget, slideTarget = 0;
    public final double ticks_in_degree = 537; //Constant to figure out
    private DcMotor arm, slide, armEncoder;
    Servo angleServo;
    @Override
    public void init(){
        //create pid controllers, one for arm and one for viper slide
        armController = new PIDController(pArm, iArm, dArm);
        slideController = new PIDController(pSlide, iSlide, dSlide);

        //set telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //init motors
        arm = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hardwareMap.get(DcMotorEx.class, "ViperSlide");
        armEncoder = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");

        angleServo = hardwareMap.get(Servo.class, "as");
        angleServo.setPosition(asPos);
    }

    @Override
    public void loop() {
        //re-set pid values
        armController.setPID(pArm, iArm, dArm);
        slideController.setPID(pSlide, iSlide, dSlide);

        //calculations for arm
        int armPos = armEncoder.getCurrentPosition();
        double pid = armController.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
        double armPower = pid * ff;

        //calculations for slide
        int slidePos = arm.getCurrentPosition();
        double slidePower = slideController.calculate(slidePos, slideTarget);

        //setting power
        arm.setPower(armPower);
        slide.setPower(slidePower);

        //200 --> 0.37
        //450 --> 0.22

        double newAsPos;

        if (armPos > 180 && armPos < 470) {
            newAsPos = (-0.0006*(armPos) + 0.49);
        } else {
            newAsPos = asPos;
        }

        angleServo.setPosition(newAsPos);

        telemetry.addData("armPos:", armPos);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("armPower:", armPower);

        telemetry.addData("slidePos:", slidePos);
        telemetry.addData("slideTarget", slideTarget);
        telemetry.addData("slidePower:", slidePower);

        telemetry.update();
    }

}
package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MainConfig")
public class DriveTrain extends OpMode {

    DcMotor RB;
    DcMotor LB;
    DcMotor RF;
    DcMotor LF;
    DcMotor Arm;
    DcMotor Actuator;

    //DistanceSensor distanceSensor;
    Servo angleServo1;
    Servo angleServo2;
    Servo rightServo;
    Servo leftServo;
    Servo planeServo;
    double servoAngle;


    @Override
    public void init() {

        RB = hardwareMap.dcMotor.get("Left_Back_Motor");
        LB = hardwareMap.dcMotor.get("Right_Back_Motor");
        RF = hardwareMap.dcMotor.get("Left_Front_Motor");
        LF = hardwareMap.dcMotor.get("Right_Front_Motor");
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        angleServo1 = hardwareMap.get(Servo.class, "servo1");
        angleServo2 = hardwareMap.get(Servo.class, "servo5");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
        planeServo = hardwareMap.get(Servo.class, "servo4");
        Actuator = hardwareMap.dcMotor.get("Actuator_Motor");

        //servoAngle = 0.5;

        planeServo.setPosition(0);
        Arm.setPower(0.2);


    }


    public void loop() {

        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();
        //RB.setPower(-gamepad1.left_stick_y);
        //LB.setPower(gamepad1.right_stick_y);
        //RF.setPower(gamepad1.left_stick_y);
        //LF.setPower(-gamepad1.right_stick_y);


        RB.setPower(gamepad1.left_stick_y*0.75);
        LB.setPower(-gamepad1.right_stick_y*0.75);
        RF.setPower(gamepad1.left_stick_y*0.75);
        LF.setPower(-gamepad1.right_stick_y*0.75);


        //left
        if (gamepad1.left_stick_x > 0 && gamepad1.right_stick_x > 0){
            LB.setPower(gamepad1.right_stick_x*0.75);
            LF.setPower(-gamepad1.right_stick_x*0.75);
            RB.setPower(gamepad1.left_stick_x*0.75);
            RF.setPower(-gamepad1.left_stick_x*0.75);
        }

        //right
        if (gamepad1.left_stick_x < 0 && gamepad1.right_stick_x < 0){
            LB.setPower(gamepad1.right_stick_x*0.75);
            LF.setPower(-gamepad1.right_stick_x*0.75);
            RB.setPower(gamepad1.left_stick_x*0.75);
            RF.setPower(-gamepad1.left_stick_x*0.75);
        }

        //dpad movements
        if (gamepad1.dpad_right){
            LB.setPower(0.25);
            LF.setPower(-0.25);
            RB.setPower(0.3);
            RF.setPower(-0.25);
        }
        if (gamepad1.dpad_left){
            LB.setPower(-0.25);
            LF.setPower(0.25);
            RB.setPower(-0.3);
            RF.setPower(0.25);
        }
        if (gamepad1.dpad_up) {
            RB.setPower(0.2);
            LB.setPower(-0.2);
            RF.setPower(0.2);
            LF.setPower(-0.2);
        }
        if (gamepad1.dpad_down) {
            RB.setPower(-0.2);
            LB.setPower(0.2);
            RF.setPower(-0.2);
            LF.setPower(0.2);
        }

        //rotations
        if (gamepad1.left_trigger>0) {
            RB.setPower(-0.4);
            LB.setPower(-0.4);
            RF.setPower(-0.4);
            LF.setPower(-0.4);
        }
        if (gamepad1.right_trigger>0) {
            RB.setPower(0.4);
            LB.setPower(0.4);
            RF.setPower(0.4);
            LF.setPower(0.4);
        }


        //arm motor controls
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            Arm.setPower(0.2);
        }
        if (gamepad2.right_trigger == 1) {
            Arm.setPower(0.55);
        }
        if (gamepad2.right_stick_y>0.8) {
            Arm.setPower(-1.00);
        }
        if (gamepad2.right_stick_y<-0.8) {
            Arm.setPower(1.00);
        }

        if (gamepad2.left_trigger == 1) {
            Arm.setPower(-0.35);
        }
        if (gamepad2.y) {
            Arm.setPower(-0.13);
        }

        //claw controls

        if (gamepad2.a) {
            leftServo.setPosition(0.3);
            rightServo.setPosition(0.3);
        }

        if (gamepad2.b) {
            leftServo.setPosition(-1);
            rightServo.setPosition(0.75);
        }

        if (gamepad2.dpad_right) {
            rightServo.setPosition(0.3);
        }

        if (gamepad2.dpad_left) {
            leftServo.setPosition(0.3);
        }

        if (gamepad2.left_bumper) {
            angleServo1.setPosition(-0.99);
            angleServo2.setPosition(1);
            //servoAngle = 0.1;
        }

        if (gamepad2.right_bumper) {
            angleServo1.setPosition(1.00);
            angleServo2.setPosition(-0.99);
            //servoAngle =0.00;
        }

        if(gamepad2.x) {
            angleServo1.setPosition(-0.44);
            angleServo2.setPosition(0.45);
        }

        if (gamepad2.x && gamepad2.dpad_down) {
            planeServo.setPosition(0.4);
        }

        if (gamepad2.left_stick_y>0.1 || gamepad2.left_stick_y<-0.1) {
            Actuator.setPower(-gamepad2.left_stick_y);
        }
        else {
            Actuator.setPower(0);
        }

        //angleServo.setPosition(servoAngle);




    }






}
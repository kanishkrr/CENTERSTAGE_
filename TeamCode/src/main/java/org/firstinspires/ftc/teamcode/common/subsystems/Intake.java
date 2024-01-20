package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    Servo angleServo1, angleServo2, rightServo, leftServo;
    private static final double servoOpen = 0.3;

    public Intake(HardwareMap hardwareMap) {
        angleServo1 = hardwareMap.get(Servo.class, "servo1");
        angleServo2 = hardwareMap.get(Servo.class, "servo5");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
    }

    public void angleServoDown() {
        angleServo1.setPosition(0);
        angleServo2.setPosition(1);
    }

    public void angleServoUp() {
        angleServo1.setPosition(0.73);
        angleServo2.setPosition(0.27);
    }
    public void angleServoMiddle(){
        angleServo1.setPosition(0.3);
        angleServo2.setPosition(0.7);
    }
    public void releaseFirstPixel() {
        rightServo.setPosition(servoOpen);
    }

    public void releaseSecondPixel() {
        leftServo.setPosition(servoOpen);
    }

    public void initServos() {
        leftServo.setPosition(-1);
        rightServo.setPosition(0.75);
    }

    public void setAngleServoPos(double leftServo, double rightServo) {
        angleServo1.setPosition(leftServo);
        angleServo2.setPosition(rightServo);
    }


}

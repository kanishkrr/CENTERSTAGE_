package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo angleServo, rightServo, leftServo;
    private static final double servoOpen = 0.3;

    public enum Mode {
        FLAT, SCORING, REST;
    }

    Mode current = Mode.REST;

    public Claw(HardwareMap hMap) {
        angleServo = hMap.get(Servo.class, "as");
        rightServo = hMap.get(Servo.class, "rs");
        leftServo = hMap.get(Servo.class, "ls");
    }

    public void openRightNarrow() { rightServo.setPosition(0.65); }

    public void openLeftNarrow() {
        leftServo.setPosition(0.38);
    }

    public void openRightWide() {
        rightServo.setPosition(0.65);
    }

    public void openLeftWide() {
        leftServo.setPosition(0.35);
    }

    public void alignWithBoard(int armPos) {
        double newAsPos = 0.3;

        if (armPos > 180 && armPos < 470) {
            newAsPos = (-0.0006*(armPos) + 0.49);
        }

        angleServo.setPosition(newAsPos);
    }

    public void alignWithGround() {
        angleServo.setPosition(0.16);
    }

    public void closeRight() {
        rightServo.setPosition(0.58);
    }

    public void closeLeft() {
        leftServo.setPosition(0.5);
    }

    public void resetPosition() {
        angleServo.setPosition(0.5);
    }

    public void closeBoth() {
        closeLeft();
        closeRight();
    }

    public void openBothNarrow() {
        openLeftNarrow();
        openRightNarrow();
    }

    public void openBothWide() {
        openLeftWide();
        openRightWide();
    }

    public void updateState(Mode mode) {
        current = mode;
    }

    public void update(double armPos) {
        if (current == Mode.FLAT) {
            alignWithGround();
        } else if (current == Mode.SCORING) {
            alignWithBoard((int) armPos);
        } else {
            resetPosition();
        }
    }


}

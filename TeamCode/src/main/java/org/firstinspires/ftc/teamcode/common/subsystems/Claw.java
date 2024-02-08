package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo angleServo, rightServo, leftServo;
    private static final double servoOpen = 0.3;

    /*
    all the different states that the servo can be in
     */
    public enum Mode {
        FLAT, SCORING, REST, LEFT, RIGHT, BOTH, WIDE, SHARP, CLOSE;
    }

    Mode current = Mode.REST;
    Mode side = Mode.BOTH;
    Mode clawState = Mode.CLOSE;

    public Claw(HardwareMap hMap) {
        /*
        init all the servos
         */
        angleServo = hMap.get(Servo.class, "as");
        rightServo = hMap.get(Servo.class, "rs");
        leftServo = hMap.get(Servo.class, "ls");
    }

    /*
    all the basic servo commands
     */
    public void openRightNarrow() { rightServo.setPosition(0.37); }

    public void openLeftNarrow() {
        leftServo.setPosition(0.63);
    }

    public void openRightWide() {
        rightServo.setPosition(0.7);
    }

    public void openLeftWide() {
        leftServo.setPosition(0.3);
    }

    /*
    auto aligns the angle servo with the board if it is in "scoring" state --> range of values is expected to change
     */
    public void alignWithBoard(int armPos) {
        double newAsPos = 0.3;

        if (armPos > 170 && armPos < 530) {
            newAsPos = (-0.0006*(armPos) + 0.49);
        }

        angleServo.setPosition(newAsPos);
    }

    public void alignWithGround() {
        angleServo.setPosition(0.135);
    }

    public void closeRight() {
        rightServo.setPosition(0.3);
    }

    public void closeLeft() {
        leftServo.setPosition(0.7);
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

    public void changeAngleState(Mode mode) {
        current = mode;
    }

    /*
    command used to control the side attachments
     */
    public void setClawState(Mode clawMode, Mode sideMode) {
        clawState = clawMode;
        side = sideMode;
    }

    /*
    update function -->
        checks the which mode the angle servo should be in and then either stays flat, aligns with the board, or moves to hold

        claw commands self explanatory
     */

    public void update(double armPos) {
        if (current == Mode.FLAT) {
            alignWithGround();
        } else if (current == Mode.SCORING) {
            alignWithBoard((int) armPos);
        } else {
            resetPosition();
        }

        switch (clawState) {
            case CLOSE:
                if (side == Mode.BOTH) {
                    closeBoth();
                } else if (side == Mode.LEFT) {
                    closeLeft();
                } else if (side == Mode.RIGHT) {
                    closeRight();
                }
                break;
            case SHARP:
                if (side == Mode.BOTH) {
                    openBothNarrow();
                } else if (side == Mode.LEFT) {
                    openLeftNarrow();
                } else if (side == Mode.RIGHT) {
                    openRightNarrow();
                }
                break;
            case WIDE:
                if (side == Mode.BOTH) {
                    openBothWide();
                } else if (side == Mode.LEFT) {
                    openLeftWide();
                } else if (side == Mode.RIGHT) {
                    openRightWide();
                }
                break;

        }
    }


}

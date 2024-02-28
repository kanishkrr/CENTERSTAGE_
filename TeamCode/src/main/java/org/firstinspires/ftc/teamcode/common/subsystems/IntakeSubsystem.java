package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class IntakeSubsystem extends SubsystemBase {
    Servo angleServo, rightServo, leftServo;
    public enum Mode {
        FLAT, SCORING, REST, LEFT, RIGHT, BOTH, WIDE, SHARP, CLOSE, STRAIGHT, LINED;
    }

    //modes for angle should be --> HOLD (after pickup), FLAT (during pickup), SCORING (to align with board), and STRAIGHT (to drop purple pixel)

    Mode current = Mode.REST;
    Mode side = Mode.BOTH;
    Mode clawState = Mode.CLOSE;

    public IntakeSubsystem(HardwareMap hMap) {
        angleServo = hMap.get(Servo.class, "as");
        rightServo = hMap.get(Servo.class, "rs");
        leftServo = hMap.get(Servo.class, "ls");
    }

    public void openRightNarrow() { rightServo.setPosition(0.5); }

    public void openLeftNarrow() {
        leftServo.setPosition(0.5);
    }

    public void openRightWide() {
        rightServo.setPosition(0.72);
    }

    public void openLeftWide() {
        leftServo.setPosition(0.28);
    }

    public void alignWithBoard(double armPos) {
        double newAsPos = 0.3;

        if (armPos > 170 && armPos < 530) {
            newAsPos = (-0.0006*(armPos) + 0.49) + 0.02;
        }

        angleServo.setPosition(newAsPos);
    }

    public void alignWithGround() {
        angleServo.setPosition(0.177);
    }

    public void alignWithGroundAuto() {
        angleServo.setPosition(0.17);
    }

    public void alignWithGroundLined() {angleServo.setPosition(0.18);}

    public void closeRight() {
        rightServo.setPosition(0.42);
    }

    public void closeLeft() {
        leftServo.setPosition(0.58);
    }

    public void resetPosition() {
        angleServo.setPosition(0.54);
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

    public void setClawState(Mode clawMode, Mode sideMode) {
        clawState = clawMode;
        side = sideMode;
    }

    public void update(double armPos) {
        if (current == Mode.FLAT) {
            alignWithGround();
        } else if (current == Mode.SCORING) {
            alignWithBoard((int) armPos);
        } else if (current == Mode.STRAIGHT) {
            alignWithGroundAuto();
        } else if (current == Mode.LINED) {
            alignWithGroundLined();
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

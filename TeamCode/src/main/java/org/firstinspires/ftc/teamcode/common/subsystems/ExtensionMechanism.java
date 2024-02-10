package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ExtensionMechanism {
    private PIDController armController, slideController;
    public static double pArm = 0.0117, iArm = 0, dArm = 0.00134; //still might change
    public static double pSlide = 0.004, iSlide = 0, dSlide = 0.000165; //still might change
    public static double f=0.56;
    public double armTarget, slideTarget = 0;
    public final double ticks_in_degree = 537;
    public DcMotor arm, slide, armEncoder;

    public double maxPower = 0.5;
    public double armPow = 0.0;

    //enum for the diff states that the arm will be in --> used to control in teleop mainly
    public enum Mode {
        FLAT, HOLD, SCORING, CUSTOM;
    }

    /*
    start arm in hold position
     */

    Mode currentArmState = Mode.HOLD;
    public ExtensionMechanism(HardwareMap hardwareMap){
        //sets pid for both arm and slide controllers
        armController = new PIDController(pArm, iArm, dArm);
        slideController = new PIDController(pSlide, iSlide, dSlide);

        //set all motors | name of the motor controls the component | encoders: arm --> slide, armEncoder --> arm
        arm = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hardwareMap.get(DcMotorEx.class, "ViperSlide");
        armEncoder = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void update() {
        maxPower = Math.abs(((Math.abs(arm.getCurrentPosition()) / 4000) + 0.5));

        switch(currentArmState) {
            case HOLD:
                //sets slide to automatically retract
                setSlideTarget(0);
                if (getArmCurrent() < getArmTarget() || getSlideCurrent() > -300) {
                    //only moves the arm if the slide is in proper position --> for much smoother movements or if arm is in lower position
                    setArmTarget(140);
                } else {
                    //keeps arm in place until slide is ready --> keeps movements smooth, also allows for more stable bot during auto
                    setArmTarget(getArmCurrent());
                }
                break;
            case FLAT:
                //if arm is in good spot then extends the slide to pickup pixel
                if (getArmCurrent() < 170) {
                    setSlideTarget(-1200);
                } else {
                    //if arm is not in good spot, sets slide to 0 so arm can move freely
                    setSlideTarget(0);
                }
                //only sets arm position if slide is tuck in --> to prevent the slide from slamming into ground when extended
                if (getSlideCurrent() > -300) {
                    setArmTarget(110);
                }
                break;
            case SCORING:
                /*
                checks if the arm is within range to deploy slide, if not sets slide to retract
                 */
                if (getArmCurrent() < 270 || getArmCurrent() > 350) {
                    setSlideTarget(0);
                } else {
                    setSlideTarget(-1200);
                }
                /*
                checks if the arm is ready to be changed --> only if slide is retracted
                 */
                if (getSlideCurrent() > -300) {
                    setArmTarget(330);
                }
                break;
            case CUSTOM:
                //basically sets the max arm power
                /*
                checks if the arm is far enough away from the target position and is not more than 90* degrees up
                 */
                if (getArmCurrent() < (getArmTarget()-60) && getArmCurrent() < 550) {
                    maxPower = 0.9; //to be tuned
                } else {
                    maxPower = 0.5;
                }
                break;
        }

        //calculations for arm | basically plugs in the error into PID and then uses f (power needed to stay up) to output a power
        int armPos = (int)getArmCurrent();
        double pid = armController.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * f;
        double armPower = pid * ff;

        //calculations for slide | basic PID
        int slidePos = arm.getCurrentPosition();
        double slidePower = slideController.calculate(slidePos, slideTarget);


        //filters the power | if power is too much --> tm inertia + dangerous
        if (armPower > maxPower) {
            armPower = maxPower;
        } else if (armPower < -maxPower) {
            armPower = -maxPower;
        }

        //sets the power
        arm.setPower(armPower);
        slide.setPower(slidePower);
    }

    /*
    basic arm commands to get info for diff functions
     */

    public void setArmTarget(double target) {
        armTarget = target;
    }

    public void setSlideTarget(double target) {
        slideTarget = target;
    }

    public double getArmCurrent() {
        return Math.abs(armEncoder.getCurrentPosition());
    }

    public double getSlideCurrent() {
        return arm.getCurrentPosition();
    }

    public double getArmTarget() {
        return armTarget;
    }

    public double getSlideTarget() {
        return slideTarget;
    }

    public void updateState(Mode state) {
        currentArmState = state;
    }

}
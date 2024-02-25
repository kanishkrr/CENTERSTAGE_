package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ExtensionTest {
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
    public double desiredMaxPower = 0.5;
    public final double ticks_in_degree = 537;
    public DcMotor arm, slide, armEncoder;
    public double maxPower = 0.5;

    //enum for the diff states that the arm will be in --> used to control in teleop mainly
    public enum Mode {
        FLAT, HOLD, SCORING, CUSTOM, AUTO;
    }

    Mode currentArmState;
    public ExtensionTest(HardwareMap hardwareMap){
        //sets pid for both arm and slide controllers
        armController = new PIDController(0.0117, 0.04, 0.00134);
        slideController = new PIDController(0.0042, 0, 0.000165);

        //set all motors | name of the motor controls the component | encoders: arm --> slide, armEncoder --> arm
        arm = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        slide = hardwareMap.get(DcMotorEx.class, "ViperSlide");
        armEncoder = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void update() {
        maxPower = Math.abs(((Math.abs(arm.getCurrentPosition()) / 3500) + 0.5));

        armCurrent = getArmCurrent(); //only calls once to minimize
        slideCurrent = getSlideCurrent();

        switch(currentArmState) {
            case HOLD:

                f = 0.56;

                armController.setPID(0.0117, 0.04, 0.00134);

                //sets slide to automatically retract
                setSlideTarget(0);
                if (armCurrent < getArmTarget() || slideCurrent > -100) {
                    //only moves the arm if the slide is in proper position --> for much smoother movements or if arm is in lower position
                    setArmTarget(160);
                } else {
                    //keeps arm in place until slide is ready --> keeps movements smooth, also allows for more stable bot during auto
                    setArmTarget(armCurrent);
                }
                break;
            case FLAT:

                f = 0.7;

                armController.setPID(0.0121, 0.04, 0.00134);

                //if arm is in good spot then extends the slide to pickup pixel
                if (armCurrent < 170) {
                    setSlideTarget(-1200);
                } else {
                    //if arm is not in good spot, sets slide to 0 so arm can move freely
                    setSlideTarget(0);
                }
                //only sets arm position if slide is tuck in --> to prevent the slide from slamming into ground when extended
                if (slideCurrent > -300) {
                    setArmTarget(110);
                }
                if (armCurrent < 100) {
                    setArmTarget(130);
                }
                break;
            case SCORING:
                /*
                checks if the arm is within range to deploy slide, if not sets slide to retract
                 */

                f = 0.7;

                armController.setPID(0.0117, 0.04, 0.00134);


                if (armCurrent < 270 || armCurrent > 350) {
                    setSlideTarget(0);
                } else {
                    setSlideTarget(-600);
                }
                /*
                checks if the arm is ready to be changed --> only if slide is retracted
                 */
                if (slideCurrent > -300) {
                    setArmTarget(330);
                }
                break;
            case CUSTOM:

                f = 0.74;

                armController.setPID(0.0124, 0.05, 0.00143);

                //basically sets the max arm power
                /*
                checks if the arm is far enough away from the target position and is not more than 90* degrees up
                 */
                if (armCurrent < (getArmTarget()-60) && armCurrent < 550) {
                    maxPower = 0.9; //to be tuned
                } else {
                    maxPower = 0.5;
                }
                break;
            case AUTO:
                maxPower = desiredMaxPower;
                break;
        }

        //calculations for arm || basically plugs in the error into PID and then uses f (power needed to stay up) to output a power
        int armPos = (int) armCurrent;
        double pid = armController.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(180 * armTarget / 1400 - 90)) * f;
        double fff = (-(slideTarget-1) / 1000.0) * ff;
        double armPower = pid * fff;

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

    public void setDesiredMaxPower(double dmp) {
        desiredMaxPower = dmp;
    }

}
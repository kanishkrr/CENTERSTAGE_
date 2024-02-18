package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.lock.qual.Holding;
import org.firstinspires.ftc.teamcode.common.centerstage.Globals;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.hardware.Actuator;
import org.firstinspires.ftc.teamcode.common.hardware.DroneRelease;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.hardware.Claw;
import org.opencv.objdetect.HOGDescriptor;

public class RobotHardwareTest {

    public SampleMecanumDrive drive;
    public ExtensionTesting extension;
    public Claw claw;
    public Actuator actuator;
    public DroneRelease drone;



    public RobotHardwareTest(HardwareMap hardwareMap) {

        this.drive = new SampleMecanumDrive(hardwareMap);
        this.extension = new ExtensionTesting(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.actuator = new Actuator(hardwareMap);
        this.drone = new DroneRelease(hardwareMap);

    }

    public void init() {

        double startX = Globals.IS_CLOSE == true ? 14 : -38;
        double startY = Globals.IS_BLUE == true ? 61 : -61;
        double heading = Globals.IS_BLUE == true ? -90 : 90;

        if (Globals.IS_AUTO) {
            drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(heading)));

            extension.updateState(ExtensionTesting.Mode.AUTO);
            extension.setArmTarget(120);
        } else {
            drive.setPoseEstimate(PoseStorage.currentPose);

            extension.updateState(ExtensionTesting.Mode.HOLD);
        }

        claw.changeAngleState(Claw.Mode.REST);
        claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.BOTH);
        claw.update(extension.getArmCurrent());

        drone.init();

    }

    public void update() {

        drive.update();
        extension.update();
        claw.update(extension.armCurrent);

    }

    public void yellowPixelArmCommand() {
        extension.setArmTarget(245);
        claw.changeAngleState(Claw.Mode.SCORING);
    }

    public void yellowPixelExtendCommand() {
        extension.setSlideTarget(-1000);
    }

    public void slideRetractCommand() {
        extension.setSlideTarget(0);
    }

    public void purplePixelArmCommand() {
        extension.setArmTarget(1350);
        claw.changeAngleState(Claw.Mode.STRAIGHT);
    }

    public void cycleExtendCommand() {
        extension.setSlideTarget(-1400);
    }

    public void whitePixelArmCommand() {
        extension.setArmTarget(330);
        claw.changeAngleState(Claw.Mode.SCORING);
    }

    public void whitePixelExtendCommand() {
        extension.setSlideTarget(-1200);
    }

    public void resetCommand() {
        extension.setArmTarget(140);
        extension.setSlideTarget(0);
        claw.changeAngleState(Claw.Mode.REST);
    }

    public void limitArm(double p) {
        extension.setDesiredMaxPower(p);
    }

    //teleop commands

    public void pixelPickupCommand() {
        extension.updateState(ExtensionTesting.Mode.FLAT);
        claw.changeAngleState(Claw.Mode.FLAT);
        claw.setClawState(Claw.Mode.WIDE, Claw.Mode.BOTH);
    }

    public void holdCommand() {
        claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.BOTH);
        extension.updateState(ExtensionTesting.Mode.HOLD);
        claw.changeAngleState(Claw.Mode.REST);
    }

    public void scoreCommand() {
        extension.updateState(ExtensionTesting.Mode.SCORING);
        claw.changeAngleState(Claw.Mode.SCORING);
    }

    public void gamepadCommand() {
        extension.updateState(ExtensionTesting.Mode.CUSTOM);
    }

    public void hangPower(double p) {
        actuator.setPower(p);
    }

    public void releaseDrone() {
        drone.release();
    }

    public void clawOpenCommand(boolean left, boolean right) {
        if (right && left) {
            claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
            claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
        } else if (right) {
            claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
        } else if (left) {
            claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
        }
    }

    public void clawCloseCommand(double left, double right) {
        if (right > 0.08 && left > 0.08) {
            claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.RIGHT);
            claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.LEFT);
        } else if (right > 0.08) {
            claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.RIGHT);
        } else if (left > 0.08) {
            claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.LEFT);
        }
    }


}
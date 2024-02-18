package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.lock.qual.Holding;
import org.firstinspires.ftc.teamcode.common.centerstage.Globals;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.hardware.Claw;
import org.firstinspires.ftc.teamcode.common.hardware.ExtensionMechanism;
import org.opencv.objdetect.HOGDescriptor;

public class RobotHardware {

    public SampleMecanumDrive drive;
    public ExtensionMechanism extension;
    public Claw claw;
    public Actuator actuator;
    public DroneRelease drone;


    public RobotHardware(HardwareMap hardwareMap) {

        this.drive = new SampleMecanumDrive(hardwareMap);
        this.extension = new ExtensionMechanism(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.actuator = new Actuator(hardwareMap);
        this.drone = new DroneRelease(hardwareMap);

    }

    public void init() {

        double startX = Globals.IS_CLOSE == true ? 14 : -38;
        double startY = Globals.IS_BLUE == true ? 61 : -61;
        double heading = Globals.IS_BLUE == true ? -90 : 90;

        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(heading)));

        claw.changeAngleState(Claw.Mode.REST);
        claw.setClawState(Claw.Mode.CLOSE, Claw.Mode.BOTH);
        claw.update(extension.getArmCurrent());

        if (Globals.IS_AUTO) {
            extension.updateState(ExtensionMechanism.Mode.AUTO);
            extension.setArmTarget(120);
        } else {
            extension.updateState(ExtensionMechanism.Mode.HOLD);
        }

        drone.init();

    }

    public void update() {

        drive.update();
        extension.update();
        claw.update(extension.armCurrent);

    }

    public void yellowPixelArmCommand() {
        extension.setArmTarget(220);
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
        claw.changeAngleState(Claw.Mode.LINED);
    }

    public void cycleExtendCommand() {
        extension.setSlideTarget(-1400);
    }

    public void whitePixelArmCommand() {
        extension.setArmTarget(290);
        claw.changeAngleState(Claw.Mode.SCORING);
    }

    public void whitePixelExtendCommand() {
        extension.setSlideTarget(-1300);
    }

    public void whitePixelRetractCommand() {
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


}

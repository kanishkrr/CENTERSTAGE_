package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotHardware {
    private static RobotHardware instance = null;

    public Drivetrain drivetrain;
    public ExtensionSubsystem extension;
    public IntakeSubsystem claw;
    public HangSubsystem hang;
    public DroneSubsystem drone;

    private HardwareMap hardwareMap;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private double voltage = 12.5;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.drivetrain = new Drivetrain(hardwareMap);
        this.extension = new ExtensionSubsystem(hardwareMap);
        this.claw = new IntakeSubsystem(hardwareMap);

        if (Globals.IS_AUTO) {

            this.hang = new HangSubsystem(hardwareMap);
            this.drone = new DroneSubsystem(hardwareMap);



        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

    }

    public void startCamera() {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                //set camera settings here
                .build();

        /*
        visionPortal = new VisionPortal.Builder()
                .setCamera()

         */
    }

    public double getVoltage() {
        return voltage;
    }

    public void update() {
        claw.update(extension.armCurrent);
    }

    public void kill() {
        instance = null;
    }

}

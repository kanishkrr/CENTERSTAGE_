package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

@Config
@Autonomous(name = "pid for arm test")
public class PIDTuner extends LinearOpMode {

    ExtensionSubsystem ext;
    IntakeSubsystem claw;

    public static double armP = 0;
    public static double armI = 0;
    public static double armD = 0;
    public static double slideP = 0;
    public static double slideI = 0;
    public static double slideD = 0;
    public static double kg = 0;
    public static double armTarget = 0;
    public static double slideTarget = 0;
    public static double value = 0;

    @Override
    public void runOpMode() {

        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        ext = new ExtensionSubsystem(hardwareMap);

        claw = new IntakeSubsystem(hardwareMap);
        claw.changeAngleState(IntakeSubsystem.Mode.SCORING);


        waitForStart();
        if (isStopRequested()) return;


        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
            claw.update(ext.armCurrent);

            ext.setArmPID(new PIDController(armP, armI, armD));
            ext.setSlidePID(new PIDController(slideP, slideI, slideD));

            ext.setArmTargetPosition(armTarget);
            ext.setSlideTargetPosition(slideTarget);

            telemetry.addData("arm current", ext.armCurrent);
            telemetry.addData("slide current", ext.slideCurrent);
            telemetry.update();
        }

    }
}

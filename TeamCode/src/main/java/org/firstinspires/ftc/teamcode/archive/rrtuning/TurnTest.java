package org.firstinspires.ftc.teamcode.archive.rrtuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.teleop.ExtensionMechanism;
import org.firstinspires.ftc.teamcode.archive.rr.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ExtensionMechanism ext = new ExtensionMechanism(hardwareMap);

        ext.updateState(ExtensionMechanism.Mode.HOLD);

        while (opModeInInit()) {
            ext.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}

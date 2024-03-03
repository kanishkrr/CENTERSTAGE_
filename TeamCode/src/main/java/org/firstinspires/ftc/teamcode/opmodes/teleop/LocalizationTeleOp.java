package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.DroneCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.PickupPixelCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.PixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.ScoreCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.SlideRetractCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

@TeleOp(name = "Localization")
public class LocalizationTeleOp extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;

        robot.init(hardwareMap);

        robot.extension.setArmTargetPosition(180);
        robot.extension.setSlideTargetPosition(0);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.update();

        robot.drivetrain.set(new Pose(gamepad1.left_stick_x*(gamepad1.right_trigger*0.75 + 0.25), -gamepad1.left_stick_y*(gamepad1.right_trigger*0.75 + 0.25), gamepad1.right_stick_x*(gamepad1.right_trigger*0.5 + 0.4)), 0);

        telemetry.addData("robot x:", robot.drivetrain.localizer.getPose().x);
        telemetry.addData("robot y:", robot.drivetrain.localizer.getPose().y);
        telemetry.addData("robot heading:", robot.drivetrain.localizer.getPose().heading);
        telemetry.update();

    }
}

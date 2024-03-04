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

@TeleOp(name = "Duo")
public class MainTeleOp extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);

        robot.extension.setArmTargetPosition(180);
        robot.extension.setSlideTargetPosition(0);

        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);
        robot.claw.update(0);

        robot.drone.init();

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new PickupPixelCommand());

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new PixelRetractCommand());

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ScoreCommand());

        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SlideRetractCommand());

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.LEFT));

        gamepadEx.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new DroneCommand());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.update();

        robot.drivetrain.set(new Pose(gamepad1.left_stick_x*(gamepad1.right_trigger*0.75 + 0.25), -gamepad1.left_stick_y*(gamepad1.right_trigger*0.75 + 0.25), gamepad1.right_stick_x*(gamepad1.right_trigger*0.5 + 0.4)), 0);

        robot.hang.setPower(0);
        if (gamepad2.dpad_down) robot.hang.setPower(-0.8);
        else if (gamepad2.dpad_up) robot.hang.setPower(0.8);

        robot.extension.setArmTargetPosition(robot.extension.armTarget - gamepad2.left_stick_y*15);
        robot.extension.setSlideTargetPosition(robot.extension.slideTarget + gamepad2.right_stick_y*30);
    }
}

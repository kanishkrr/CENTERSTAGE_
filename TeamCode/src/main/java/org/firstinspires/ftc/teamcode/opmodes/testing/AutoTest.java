package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteBackExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteBackRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

import java.util.List;

@Config
@Autonomous(name = "auto route test")
public class AutoTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() {


        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.init(hardwareMap);

        robot.claw.setClawState(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH);
        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);

        robot.claw.update(0);

        CommandScheduler.getInstance().schedule(new ArmCommand(160));


        while (opModeInInit()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            robot.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        //new Pose(24, 30, Math.toRadians(-90))

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new PositionCommand(new Pose(25.7, 21.5, Math.toRadians(-90)))
                            .alongWith(new YellowPixelExtendCommand()),
                    new WaitCommand(50),
                    new YellowPixelRetractCommand(),
                    new WaitCommand(50),
                    new PositionCommand(new Pose(27.6, 25, Math.toRadians(-90))) //20
                            .alongWith(new PurplePixelExtendCommand()),
                    new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.LEFT),
                    new WaitCommand(800),
                    new ClawCommand(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH),
                    new PositionCommand(new Pose(7,2.5,Math.toRadians(-90))),
                    new PositionCommand(new Pose(-55,0.5,Math.toRadians(-90))),
                    new PositionCommand(new Pose(-57,20,Math.toRadians(-90))),
                    new WhiteBackExtendCommand(),
                    new WhiteBackRetractCommand(),
                    new PositionCommand(new Pose(-55,-1,Math.toRadians(-90))),
                    new PositionCommand(new Pose(-55,-1,Math.toRadians(-90)))
            )
        );


        while (!isStopRequested() && opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            robot.update();
            CommandScheduler.getInstance().run();
        }

        robot.kill();

    }
}

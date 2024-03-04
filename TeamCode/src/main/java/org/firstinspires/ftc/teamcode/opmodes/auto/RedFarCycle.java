package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.lock.qual.LockPossiblyHeld;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhitePixelPlaceCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurpleDropRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurplePixelDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.YellowPixelFarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand.InitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand.StartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ExtensionCommand;
import org.firstinspires.ftc.teamcode.common.drive.Constants;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.centerstage.Location;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionBlueFar;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRedFar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Config
@Autonomous(name = "red far cycle")
public class RedFarCycle extends LinearOpMode {

    OpModeVisionRedFar pipeline;
    OpenCvWebcam camera;

    private final RobotHardware robot = RobotHardware.getInstance();

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() {

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewID);
        pipeline = new OpModeVisionRedFar();
        camera.setPipeline(pipeline);

        camera.setMillisecondsPermissionTimeout(Integer.MAX_VALUE); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("No Camera Could Not Start");
                runOpMode();
            }
        });

        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);


        Pose purplePose = new Pose();
        Pose yellowPose = new Pose();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.FAR;
        Globals.PATH = Location.STAGEDOOR;
        Globals.PARK = Location.RIGHT;

        Constants.MAX_LINEAR_SPEED = 0.5;
        Constants.MAX_ROTATIONAL_SPEED = 0.4;

        robot.init(hardwareMap);

        robot.claw.setClawState(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH);
        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);

        robot.claw.update(0);

        telemetry.addData("voltage:", robot.getVoltage());

        CommandScheduler.getInstance().schedule(new InitCommand());


        while (opModeInInit()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            OpModeVisionRedFar.TeamPropLocation initobjLocation = pipeline.location;

            if (initobjLocation == OpModeVisionRedFar.TeamPropLocation.Middle) {
                telemetry.addData("block detected in center", initobjLocation);
                telemetry.update();
            } else if (initobjLocation == OpModeVisionRedFar.TeamPropLocation.Right) {
                telemetry.addData("block detected in right", initobjLocation);
                telemetry.update();
            } else if (initobjLocation == OpModeVisionRedFar.TeamPropLocation.Left) {
                telemetry.addData("block detected in left", initobjLocation);
                telemetry.update();
            } else {
                telemetry.addData("no block detected", initobjLocation);
                telemetry.update();
            }

            CommandScheduler.getInstance().run();
            robot.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        OpModeVisionRedFar.TeamPropLocation location = pipeline.location;
        int numberOfLeft = 0;
        int numberOfRight = 0;
        int numberOfCenter = 0;
        for (int i=0; i<100; i++) {
            location = pipeline.location;
            if (location == OpModeVisionRedFar.TeamPropLocation.Middle) {
                numberOfCenter += 1;
                telemetry.addData("block detected in center", location);
                telemetry.update();
            }
            else if (location == OpModeVisionRedFar.TeamPropLocation.Right) {
                numberOfRight += 1;
                telemetry.addData("block detected in right", location);
                telemetry.update();
            }
            else if (location == OpModeVisionRedFar.TeamPropLocation.Left) {
                numberOfLeft += 1;
                telemetry.addData("block detected in left", location);
                telemetry.update();
            }
            else {
                telemetry.addData("no block detected", location);
                telemetry.update();
            }}
        if (numberOfLeft>numberOfCenter && numberOfLeft>numberOfRight) {
            //insert code for left auto here
            telemetry.addData("following left trajectory", location);
            telemetry.update();
        }
        else if (numberOfRight>numberOfLeft && numberOfRight>numberOfCenter) {
            //insert code for right auto here
            telemetry.addData("following right trajectory", location);
            telemetry.update();
        }
        else if (numberOfCenter>numberOfLeft && numberOfCenter>numberOfRight) {
            //insert code for center auto here
            telemetry.addData("following center trajectory", location);
            telemetry.update();
        }
        else {
            telemetry.addData("no trajectory being followed", location);
            telemetry.update();
        }

        switch(location) {
            case Left:
                purplePose = new Pose(-3.5, 8.5, Math.toRadians(12.7));
                yellowPose = new Pose(77, 24.5, Math.toRadians(-90));
                break;
            case Middle:
                purplePose = new Pose(-1.5, 8.6, Math.toRadians(-13));
                yellowPose = new Pose(76, 18, Math.toRadians(-90));
                break;
            case Right:
                purplePose = new Pose(-1, 1, Math.toRadians(-32));
                yellowPose = new Pose(76, 12, Math.toRadians(-92));
                break;
        }

        final OpModeVisionRedFar.TeamPropLocation l = location;



        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new StartCommand(),

                        new PositionCommand(purplePose)
                                .alongWith(new PurplePixelDropCommand()),

                        new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),

                        new WaitCommand(200),

                        new PurpleDropRetractCommand(),

                        new ConditionalCommand(
                                new PositionCommand(new Pose(-5, 25.5, Math.toRadians(90))),
                                new SequentialCommandGroup(
                                        new ConditionalCommand(
                                                new PositionCommand(new Pose(3, 49.5, Math.toRadians(0))),
                                                new PositionCommand(new Pose(-10, 49.5, Math.toRadians(0))),
                                                () -> {
                                                    return l != OpModeVisionRedFar.TeamPropLocation.Middle;
                                                }
                                        ),
                                        new PositionCommand(new Pose(-5, 49.5, Math.toRadians(90)))
                                ), //stagedoor
                                () -> {
                                    return Globals.PATH == Location.SIDE;
                                }
                        ),

                        new WhiteFrontExtendCommand(),

                        new WhiteFrontRetractCommand(),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new PositionCommand(new Pose(-4, 2, Math.toRadians(90))),
                                        new PositionCommand(new Pose(77, -2.5, Math.toRadians(85)))
                                ),
                                new SequentialCommandGroup(
                                        new PositionCommand(new Pose(0, 50, Math.toRadians(90))),
                                        new PositionCommand(new Pose(60, 50, Math.toRadians(90)))
                                ),
                                () -> {
                                    return Globals.PATH == Location.SIDE;
                                }
                        ),

                        new WaitCommand(50),

                        new ConditionalCommand(
                                new PositionCommand(yellowPose),
                                new ConditionalCommand(
                                        new PositionCommand(new Pose(yellowPose.x, yellowPose.y + 2, yellowPose.heading)),
                                        new PositionCommand(new Pose(yellowPose.x-2, yellowPose.y + 1, yellowPose.heading-11)),
                                        () -> {
                                            return l == OpModeVisionRedFar.TeamPropLocation.Left;
                                        }
                                ),
                                () -> {
                                    return Globals.PATH == Location.SIDE;
                                }
                        ),

                        new PositionCommand(yellowPose),

                        new YellowPixelFarCommand(),

                        new WaitCommand(50),

                        new YellowPixelRetractCommand(),

                        new WhitePixelPlaceCommand(),

                        new ExtensionCommand(0),

                        new PurplePixelRetractCommand(),

                        new WaitCommand(400)
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

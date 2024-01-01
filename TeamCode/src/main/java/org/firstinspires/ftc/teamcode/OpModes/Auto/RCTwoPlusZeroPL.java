package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RoadRunner.Drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.RoadRunner.Util.Side;

import java.util.List;

@Autonomous
public class RCTwoPlusZeroPL extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231027_204348.tflite";
    private static final String[] LABELS = {
            "team object",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    Side side;
    Pose2d purple, yellow;
    Vector2d parkLeft;
    DcMotor Arm;
    Intake intake;

    public void moveArmTo(int revs, double power) {
        Arm.setTargetPosition(revs);

        int error = Arm.getTargetPosition()-Arm.getCurrentPosition();

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (error < 0) {
            Arm.setPower(-power);
        } else {
            Arm.setPower(power);
        }

        while (Arm.isBusy()) {
            System.out.println(Arm.getCurrentPosition());
        }

        Arm.setPower(0.0);
    }



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        intake = new Intake(hardwareMap);
        intake.initServos();
        initTfod();

        parkLeft = new Vector2d(50.2, -9.6);

        waitForStart();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();

        if(currentRecognitions.size() != 0){
            float x1 = currentRecognitions.get(0).getLeft();
            float y1 = currentRecognitions.get(0).getTop();

            if (x1 < 1000) {
                side = Side.LEFT;
            } else if (x1 > 1000) {
                side = Side.CENTER;
            } else {
                side = Side.RIGHT;
            }
        }

        switch (side) {
            case LEFT:
                yellow = new Pose2d(49, -31, Math.toRadians(0));
                purple = new Pose2d(10.5, -28, Math.toRadians(180));
                break;
            case CENTER:
                yellow = new Pose2d(49, -37, Math.toRadians(0));
                purple = new Pose2d(22.5, -21.5, Math.toRadians(180));
                break;
            case RIGHT:
                yellow = new Pose2d(49, -43, Math.toRadians(0));
                purple = new Pose2d(10.5, 28, Math.toRadians(180));
                break;
        }

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                .lineToSplineHeading(yellow)
                .addTemporalMarker(1, () -> {
                    moveArmTo(65, 0.27);
                    intake.angleServoMiddle();
                })
                .addTemporalMarker(2.2, () -> {
                    Arm.setPower(0.21);
                })
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(purple)
                .addTemporalMarker(2.5, () -> {
                    moveArmTo(115, 0.32);
                    intake.angleServoDown();
                })
                .addTemporalMarker(4, () -> {
                    moveArmTo(95, 0.3);
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(parkLeft)
                .build();

        drive.followTrajectory(traj);

        intake.releaseSecondPixel();

        drive.followTrajectory(traj1);

        intake.releaseFirstPixel();

        drive.followTrajectory(traj2);

    }
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.65f);
    }

    private List<Recognition> telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        return currentRecognitions;
    }

}
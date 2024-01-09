package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class Test {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d yellow = new Pose2d(49, 33, Math.toRadians(0));
        Pose2d purple = new Pose2d(-44.3, 26, Math.toRadians(0));
        Pose2d parkRight = new Pose2d(52.2, 10, Math.toRadians(180));
        Pose2d parkLeft = new Pose2d(47, 59.6, Math.toRadians(180));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(150), Math.toRadians(150), 17.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 61, Math.toRadians(-90)))
                                .lineToLinearHeading(purple)
                                .waitSeconds(9)
                                .strafeLeft(32)
                                .forward(66)
                                .lineToLinearHeading(yellow)
                                .waitSeconds(0.5)
                                .strafeRight(18)
                                .lineToSplineHeading(parkRight)
                                .back(7)
                                .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
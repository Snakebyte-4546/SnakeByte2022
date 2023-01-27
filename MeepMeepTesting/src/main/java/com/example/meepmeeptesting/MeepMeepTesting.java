package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity path = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50.46434527240892, Math.toRadians(304.35726315789475), Math.toRadians(304.35726315789475), 9.5)
                .setDimensions(12.87, 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(-35, -15, Math.toRadians(90)))
                                    .addDisplacementMarker(() -> {
                                        //do shit
                                    })
                                    .lineToLinearHeading(new Pose2d(-23.5, -10, Math.toRadians(90)))
                                    .addDisplacementMarker(() -> {
                                        //more shit
                                    })
                                    .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                                    .addDisplacementMarker(() ->{
                                        //even more shit
                                    })
                                    .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(-28, -6, Math.toRadians(235)))
                                    .addDisplacementMarker(() -> {
                                        //who would have guessed, more shit!
                                    })
                                    .build();
                        }
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path)
                .start();
    }
}
package com.example.meepmeeptesting;

import java.util.Vector;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        Pose2d scorePose= new Pose2d(-29, -5, Math.toRadians(220));
        Pose2d pickupPose = new Pose2d(-60, -12, Math.toRadians(180));


        RoadRunnerBotEntity path = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel     maxAccel     maxAngVel            maxAngAccel          track width
                .setConstraints(35, 20, Math.toRadians(100), Math.toRadians(250), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(startPose)


                                    // preloadToGoal
                                    .strafeTo(new Vector2d(-35, -10.2))
                                    .turn(Math.toRadians(130), Math.toRadians(80), Math.toRadians(150))
                                    .strafeTo(new Vector2d(-28.5, -4.5))

                                    // score1
                                    .strafeTo(new Vector2d(-36, -12))
                                    .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                                    .strafeTo(new Vector2d(-58, -12))

                                    //.strafeTo(new Vector2d(-36, -12))
                                    //.turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                                    .build();
                        }
                );


        RoadRunnerBotEntity drift = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel     maxAccel     maxAngVel            maxAngAccel          track width
                .setConstraints(45, 45, Math.toRadians(305), Math.toRadians(300), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))

                                    //drop off first cone

                                    .forward(5)
                                    .splineToSplineHeading(new Pose2d(-29, -15, Math.toRadians(220)), Math.toRadians(62))

                                    .build();
                        }
                );


        RoadRunnerBotEntity movementTest = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))

                                    //drop off first cone
                                    .strafeTo(new Vector2d(-35, -12.5))
                                    .strafeTo(new Vector2d(-12, -12.5))
                                    .lineToSplineHeading(new Pose2d(-12, -40, Math.toRadians(270)))
                                    .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(180)), Math.toRadians(180))
                                    .splineToSplineHeading(new Pose2d(-35, -50, Math.toRadians(90)), Math.toRadians(90))
                                    .strafeTo(new Vector2d(-35, -64))
                                    .build();
                        }
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path) //  path | drift | movementTest
                .start();
    }
}

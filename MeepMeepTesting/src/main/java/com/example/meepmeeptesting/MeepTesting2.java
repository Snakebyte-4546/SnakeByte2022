package com.example.meepmeeptesting;

import java.util.Collections;
import java.util.Vector;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.RoadRunnerBotEntityBuilder;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.sequencesegment.TurnSegment;

public class MeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);


        double  east = Math.toRadians(0);
        double  northEast = Math.toRadians(45);
        double  north = Math.toRadians(90);
        double  northWest = Math.toRadians(125);
        double  west = Math.toRadians(180);
        double  southWest = Math.toRadians(225);
        double  south = Math.toRadians(270);
        double  southEast = Math.toRadians(295);

        Pose2d start_pose      = new Pose2d(-35, -64, north);

        Pose2d scoreHigh_pose  = new Pose2d(-29, -5, southWest);
        Vector2d scoreHigh_vector  = new Vector2d(-29, -5);

        Pose2d pickup_pose     = new Pose2d(-60, -12, west);
        Vector2d pickup_vector = new Vector2d(-60, -12);

        Pose2d neutral_pose      = new Pose2d(-35, -12,west);
        Vector2d neutral_vector = new Vector2d(-35, -12);

        /*RoadRunnerBotEntity positionTester = new DefaultBotBuilder(meepMeep)
                .setConstraints(35, 20, Math.toRadians(100), Math.toRadians(250), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(startPose)

                                    //first
                                    .lineTo(nutralVector)
                                    .lineToSplineHeading(poseMaker(nutralVector, Math.toRadians(359)))

                                    .forward(.1)

                                    .build();
                        }
                );


        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel     maxAccel     maxAngVel            maxAngAccel          track width
                .setConstraints(35, 20, Math.toRadians(100), Math.toRadians(250), 9.5)
                .setDimensions(12.87    , 12)
                .followRoadRunnerBotEntity(drive -> {
                            return drive.trajectorySequenceBuilder(startPose)


                                    // preloadToNutral
                                    .strafeTo(nutralVector)
                                    .lineToSplineHeading(poseMaker(nutralVector, northWest))

                                    // score1
                                    .lineToSplineHeading(scorePose)
                                    .strafeTo(new Vector2d(-58, -12))

                                    //.strafeTo(new Vector2d(-36, -12))
                                    //.turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                                    .build();
                        }
                );


        RoadRunnerBotEntity motionCalibration = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followRoadRunnerBotEntity(drive -> {
                            return drive.trajectorySequenceBuilder(new Pose2d(-35, -64, Math.toRadians(90)))

                                    .strafeTo(new Vector2d(-35, -12.5))
                                    //     angle to turn to  |   turn velocity   |  turn acceleration
                                    .turn(0            , Math.toRadians(80), Math.toRadians(150))
                                    .turn(Math.toRadians(90))
                                    .turn(Math.toRadians(270))
                                    .strafeTo(new Vector2d(-12, -12.5))
                                    .lineToSplineHeading(new Pose2d(-12, -40, Math.toRadians(270)))
                                    .splineToSplineHeading(new Pose2d(-24, -60, Math.toRadians(180)), Math.toRadians(180))
                                    .splineToSplineHeading(new Pose2d(-35, -50, Math.toRadians(90)), Math.toRadians(90))
                                    .splineToConstantHeading(new Vector2d(-48, -34), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-60, -50, Math.toRadians(270)), Math.toRadians(270))
                                    .strafeTo(new Vector2d(-60, -54))

                                    .lineToLinearHeading(new Pose2d(-35, -64, Math.toRadians(90)))
                                    .build();
                        }
                );*/



        // Trajectory setup
        RoadRunnerBotEntity startToNeutral = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(start_pose)
                                    .addDisplacementMarker(5, () -> {
                                        //prime();
                                        })
                                    .lineTo(neutral_vector)

                                    // from neutral to highGoal
                                    .turn(Math.toRadians(135))
                                    .lineToLinearHeading(scoreHigh_pose)
                                    // score
                                    .addDisplacementMarker(() -> {
                                        //score();
                                        })
                                    .waitSeconds(2)

                                    // from highGoal to neutral
                                    .lineTo(neutral_vector)
                                    .turn(Math.toRadians(135))

                                    // from neutral to pickup
                                    .addSpatialMarker(new Vector2d(-35 -12), () -> {
                                        //restAtConeLevel(numCones);
                                        })
                                    .lineTo(pickup_vector)
                                    .addDisplacementMarker(() -> {
                                        //intake();
                                    })
                                    .waitSeconds(2)
                                    .lineTo(neutral_vector)
                                    .build();
                        }
                );

        RoadRunnerBotEntity highToNeutral = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                    return drive.trajectorySequenceBuilder(scoreHigh_pose)
                            .lineTo(neutral_vector)
                            .turn(Math.toRadians(135))
                            .build();
                        }
                );

        RoadRunnerBotEntity scoreHigh = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                            return drive.trajectorySequenceBuilder(pickup_pose)
                                    .lineTo(neutral_vector)

                                    //to High and score
                                    .turn(Math.toRadians(45))
                                    .lineToLinearHeading(scoreHigh_pose)
                                    .addDisplacementMarker(() -> {
                                        //score();
                                    })
                                    .waitSeconds(4)

                                    //back to neutral
                                    .lineTo(neutral_vector)
                                    .turn(Math.toRadians(-45))

                                    //from neutral to pickup
                                    .lineTo(pickup_vector)
                                    .build();
                        }
                );

        RoadRunnerBotEntity pickupToNeutral = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                    return drive.trajectorySequenceBuilder(pickup_pose)
                .lineTo(neutral_vector)
                .build();
                        }
                );

        RoadRunnerBotEntity park1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                    return drive.trajectorySequenceBuilder(neutral_pose)
                .lineToLinearHeading(poseMaker(neutral_vector, west))
                .lineTo(new Vector2d(-58, -12))
                .build();
                        }
                );

        RoadRunnerBotEntity park2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                    return drive.trajectorySequenceBuilder(neutral_pose)
                .lineToLinearHeading(poseMaker(neutral_vector, west))
                .build();
                        }
                );

        RoadRunnerBotEntity park3 = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                    return drive.trajectorySequenceBuilder(neutral_pose)
                .lineToLinearHeading(poseMaker(neutral_vector, west))
                .lineTo(new Vector2d(-12, -12))
                .build();
                        }
                );

        RoadRunnerBotEntity neutralToStart =new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 50.5, Math.toRadians(305), Math.toRadians(305), 9.5)
                .setDimensions(12.87    , 12)
                .followTrajectorySequence(drive -> {
                    return drive.trajectorySequenceBuilder(neutral_pose)
                            .turn(Math.toRadians(-90))
                            .lineTo(new Vector2d(-35, -64))
                .build();
                        }
                );





        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(startToNeutral)
                .addEntity(scoreHigh)
                .start();
    }
                    private static Pose2d poseMaker(Vector2d cord, double head) {
                        return new Pose2d(cord.getX() + .1, cord.getY(), head);
                    }


}

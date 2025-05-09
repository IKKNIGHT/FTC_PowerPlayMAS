package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep) // #3
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 8.75)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-33, -65, Math.toRadians(180)))
                                .lineTo(new Vector2d(-14, -65)) //backup from starting position
                                .strafeTo(new Vector2d(-14, -16.5)) //go to middle of field
                                .lineTo(new Vector2d(-16.75, -16.25)) //adjust a bit forward to score preload
                                //5 seconds
                                .waitSeconds(2.5)
                        .lineTo(new Vector2d(-65, -14.5)) //cone stack pos
                        //2.5 seconds
                        .waitSeconds(1)
                                .lineTo(new Vector2d(-16.75, -16.5)) //goto score pos
                                .waitSeconds(5)
                                .lineTo(new Vector2d(-14, -14.5))
//                        .addTemporalMarker(0, () -> robot.liftArm(poleHeight, 1)) //raise arm
//                        .addTemporalMarker(4, () -> robot.shoulder.setPosition(0.95)) //stretch arm out
//                        .addTemporalMarker(5.5, () -> robot.grabber.setPosition(1.0)) //release cone
//                        .addTemporalMarker(6.5, () -> robot.shoulder.setPosition(0.6)) //bring arm back
//                        .addTemporalMarker(7, () -> robot.liftArm(coneTop, 1)) //drop arm to top of cone stack
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
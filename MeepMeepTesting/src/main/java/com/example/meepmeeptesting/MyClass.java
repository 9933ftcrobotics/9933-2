package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, 61.25, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(5, 35))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(5, 42))
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(32, 42))
                .strafeToConstantHeading(new Vector2d(35, 42))
                .strafeToConstantHeading(new Vector2d(38, 27))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(40, 36)) // Get first yellow
                .turnTo(Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(51, 51))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(35, 42)) //Score
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(48, 26))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(50, 36))
                .turnTo(Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(51, 51))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(45, 38))
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(58, 24)) // Go to third
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(40, 27))
                .turnTo(Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(51, 51))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, 62, Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(-5, 36), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-8, 42), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-35, 42), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-35, 25), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-60, 50), Math.toRadians(90))
                //.strafeToSplineHeading(new Vector2d(-47, 42), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-47, 47), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-5, 45), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-5, 36), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-8, 42), Math.toRadians(-90))
                //.strafeToSplineHeading(new Vector2d(-47, 42), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-47, 47), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-5, 45), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-5, 36), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-8, 42), Math.toRadians(-90))
                /*.strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50))
                .strafeTo(new Vector2d(50, 50))
                .strafeToSplineHeading(new Vector2d(35, 37), Math.toRadians(0))
                .strafeTo(new Vector2d(35, 25))
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50))
                .strafeTo(new Vector2d(50, 50))
                .strafeToSplineHeading(new Vector2d(45, 25), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50))
                .strafeTo(new Vector2d(50, 50))
                .strafeToSplineHeading(new Vector2d(56, 25), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50))
                .strafeTo(new Vector2d(50, 50))
                .strafeToSplineHeading(new Vector2d(40, 40), Math.toRadians(-90))*/
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
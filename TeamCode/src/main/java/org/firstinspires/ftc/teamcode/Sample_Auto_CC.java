package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Autonomous(name = "Sample_Auto_CC", group = "Autonomous")
public class Sample_Auto_CC extends LinearOpMode {


    @Override
    public void runOpMode() {
        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
        );
        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        Pose2d initialPose = new Pose2d(-15, 62, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        /*TrajectoryActionBuilder One = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-15, 50));
        //.turnTo(Math.toRadians(45))
        //.strafeTo(new Vector2d(45, 45));
        TrajectoryActionBuilder FirstScore = drive.actionBuilder(secondPos)
                .strafeTo(new Vector2d(-5, 45));
        TrajectoryActionBuilder Two = drive.actionBuilder(thirdPos)
                .strafeTo(new Vector2d(-36, 35))
                .turnTo(Math.toRadians(175));
        TrajectoryActionBuilder TwoFinish = drive.actionBuilder(fourthPos)
                .strafeTo(new Vector2d(-34, 23));
        TrajectoryActionBuilder ScoreOne = drive.actionBuilder(afterPick)
                .strafeTo(new Vector2d(45, 45))
                .turnTo(Math.toRadians(50));
        TrajectoryActionBuilder ScoreTwo = drive.actionBuilder(afterScoreOne)
                .strafeTo(new Vector2d(51, 51));
        TrajectoryActionBuilder Four = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(38, 28))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(48, 26));
        TrajectoryActionBuilder Sleep = drive.actionBuilder(initialPose)
                .waitSeconds(0.75);
        TrajectoryActionBuilder SleepLong = drive.actionBuilder(initialPose)
                .waitSeconds(2);
        TrajectoryActionBuilder SleepReallyLong = drive.actionBuilder(initialPose)
                .waitSeconds(30);
        TrajectoryActionBuilder VeryShort = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        TrajectoryActionBuilder Five = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(48, 26))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(57, 26));
        TrajectoryActionBuilder Final = drive.actionBuilder(finishPos)
                //.strafeTo(new Vector2d(-5, 45))
                .strafeTo(new Vector2d(-53, 58))
                        .turnTo(Math.toRadians(-90));


        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();


        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        //Rotate Wrist
                        new ParallelAction(
                                arm.upSpecimen(),
                                arm.outSpecimen()
                        ),
                        //Wait,
                        new ParallelAction(
                                RunFirst,
                                arm.upSpecimen(),
                                arm.outSpecimen()
                        ),
                        //Wait,
                        new ParallelAction(
                                scoreFirst,
                                arm.upSpecimenScore(),
                                arm.outSpecimenScore()
                        ),
                        new ParallelAction(
                                RunSecond,
                                arm.upRest(),
                                arm.outRest()
                        ),
                        new InstantAction(claw::SetWristCenter),
                        new ParallelAction(
                               RunSecondFinish,
                                arm.upRest(),
                                arm.outRest()
                        ),
                        new ParallelAction(
                                new InstantAction(claw::grabberPick),
                                arm.upPick(),
                                arm.outPick()
                        ),
                        Wait,
                        new InstantAction(claw::grabberStop),
                        new ParallelAction(
                                Finish,
                                arm.upRest(),
                                arm.outRest()
                        ),
                        new ParallelAction(
                                WaitReallyLong,
                                arm.outZero(),
                                arm.upZero()
                        )

                )
        );

         */
    }
}
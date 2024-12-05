package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RedLeft", group = "Autonomous")
@Disabled
public class RedLeft extends LinearOpMode {


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

        Pose2d initialPose = new Pose2d(15, 62, Math.toRadians(-90));
        Pose2d secondPos = new Pose2d(45, 45, Math.toRadians(-90));
        Pose2d thirdPos = new Pose2d(52, 52, Math.toRadians(50));
        Pose2d fourthPos = new Pose2d(35, 35, Math.toRadians(0));
        Pose2d afterPick = new Pose2d(36, 22, Math.toRadians(0));
        Pose2d afterScoreOne = new Pose2d(45, 45, Math.toRadians(50));
        Pose2d finishPos = new Pose2d(51, 51, Math.toRadians(50)); //TODO: Will need to change is update Trajectory
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder One = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(45, 45));
        //.turnTo(Math.toRadians(45))
        //.strafeTo(new Vector2d(45, 45));
        TrajectoryActionBuilder FirstScore = drive.actionBuilder(secondPos)
                .turnTo(Math.toRadians(50))
                .strafeTo(new Vector2d(51, 51));
        TrajectoryActionBuilder Two = drive.actionBuilder(thirdPos)
                .strafeTo(new Vector2d(36, 35))
                .turnTo(Math.toRadians(0));
        TrajectoryActionBuilder TwoFinish = drive.actionBuilder(fourthPos)
                .strafeTo(new Vector2d(36, 22));
        TrajectoryActionBuilder ScoreOne = drive.actionBuilder(afterPick)
                .strafeTo(new Vector2d(45, 45))
                .turnTo(Math.toRadians(50));
        TrajectoryActionBuilder ScoreTwo = drive.actionBuilder(afterScoreOne)
                .strafeTo(new Vector2d(50, 50));
        TrajectoryActionBuilder Four = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(38, 28))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(48, 26));
        TrajectoryActionBuilder Sleep = drive.actionBuilder(initialPose)
                .waitSeconds(0.75);
        TrajectoryActionBuilder SleepLong = drive.actionBuilder(initialPose)
                .waitSeconds(2);
        TrajectoryActionBuilder SleepReallyLong = drive.actionBuilder(initialPose)
                .waitSeconds(10);
        TrajectoryActionBuilder VeryShort = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);
        TrajectoryActionBuilder Five = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(48, 26))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(57, 26));
        TrajectoryActionBuilder Final = drive.actionBuilder(finishPos)
                .strafeTo(new Vector2d(40, 40))
                .turnTo(Math.toRadians(-95)); ///90?


        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();

        Action RunFirst = One.build();
        Action scoreFirst = FirstScore.build();
        Action RunSecond = Two.build();
        Action RunSecondFinish = TwoFinish.build();
        Action RunScoreOne = ScoreOne.build();
        Action RunScoreTwo = ScoreTwo.build();
        Action RunFourth = Four.build();
        Action Wait = Sleep.build();
        Action WaitLong = SleepLong.build();
        Action WaitReallyLong = SleepReallyLong.build();
        Action Finish = Final.build();
        Action Short = VeryShort.build();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        //Rotate Wrist
                        new ParallelAction(
                                ///arm.upRest(),
                                arm.upTest(),
                                arm.outRest()
                        ),
                        new InstantAction(claw::SetWristCenter),
                        RunFirst,
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outHigh(),
                                scoreFirst
                        ),
                        //Wait,
                        /*new ParallelAction(
                            arm.outHigh()
                        ),*/
                        //WaitLong
                        new InstantAction(claw::grabberPlace),
                        Wait,
                        //RunSecond,
                        new ParallelAction(
                                arm.outRest()
                                //new InstantAction(arm::stopUpDown)
                        ),
                        new ParallelAction(
                                new InstantAction(arm::stopUpDown)
                        ),
                        new ParallelAction(
                                //arm.outRest(),
                                RunSecond,
                                new InstantAction(claw::grabberStop)
                                //new InstantAction(arm::stopUpDown)
                        ),
                        new ParallelAction(
                                arm.outRest(),
                                //new InstantAction(arm::stopUpDown),
                                RunSecondFinish,
                                //new InstantAction(claw::grabberStop),
                                new InstantAction(claw::grabberPick)
                        ),
                        //new InstantAction(claw::grabberPick),
                        new ParallelAction(
                                arm.outPick(),
                                arm.upPick()
                        ),
                        new ParallelAction(
                                new InstantAction(claw::grabberStop),
                                RunScoreOne,
                                arm.upHigh()
                        ),
                        new ParallelAction(
                                RunScoreTwo,
                                arm.outHigh(),
                                arm.upHigh()
                        ),
                        new ParallelAction(
                                new InstantAction(claw::grabberPlace),
                                arm.outHigh(),
                                arm.upHigh()
                        ),
                        //new InstantAction(claw::grabberPlace),
                        Wait,
                        new ParallelAction(
                                Finish,
                                new InstantAction(claw::grabberStop),
                                arm.outZero()
                        ),
                        new ParallelAction(
                                arm.upZero(),
                                arm.outZero(),
                                WaitReallyLong
                        )
                        //WaitLong
                        //arm.upRest()
                        /*new ParallelAction(
                                arm.upPick(),
                                arm.outPick(),
                                new InstantAction(claw::grabberPick)
                        ),
                        Wait,
                        new ParallelAction(
                                arm.upRest(),
                                arm.upRest()
                        )
                        /*new ParallelAction(
                                arm.upHigh(),
                                arm.upHigh(),
                                RunScoreTwo
                        ),
                        new InstantAction(claw::grabberPlace),
                        Wait,
                        //Finish,
                        new ParallelAction(
                                arm.outRest(),
                                Finish
                        ),
                        arm.upRest(),
                        Wait
                        /*Wait,
                        new ParallelAction(
                                arm.outRest()
                        ),
                        Wait,
                        new ParallelAction(
                                arm.upRest()
                        ),
                        RunSecond
                        /*arm.upSpecimenScore(),
                        //Wait,
                        arm.outSpecimenScore(),
                        //Score
                        claw.placeGrabber(),
                        RunSecond
                        //Wrist Center
                        /*claw.centerWrist(),
                        arm.upPick(),
                        //Wait,
                        arm.outPick(),
                        //Pick Up
                        claw.pickGrabber(),
                        WaitLong,
                        RunScore,
                        arm.upHigh(),
                        Wait,
                        arm.outHigh(),
                        WaitLong,
                        //Score
                        claw.placeGrabber(),
                        arm.outRest(),
                        Wait,
                        arm.upRest(),
                        RunFourth,
                        arm.upPick(),
                        //Wait,
                        arm.outPick(),
                        WaitLong,
                        RunScore,
                        arm.upHigh(),
                        Wait,
                        arm.outHigh(),
                        WaitLong,
                        //Score
                        claw.placeGrabber(),
                        arm.outRest(),
                        Wait,
                        arm.upRest(),
                        claw.stopGrabber(),
                        Finish*/
                )
        );
    }
}
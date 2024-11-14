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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BlueLeft", group = "Autonomous")
public class BlueLeft extends LinearOpMode {


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
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder One = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(45, 45))
                .turnTo(Math.toRadians(45))
                .strafeTo(new Vector2d(45, 45));
        TrajectoryActionBuilder FirstScore = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(50, 50));
        TrajectoryActionBuilder Two = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(35, 35))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(38, 26));
        TrajectoryActionBuilder ScoreOne = drive.actionBuilder(initialPose)
                .turnTo(Math.toRadians(45))
                .strafeTo(new Vector2d(45, 45));
        TrajectoryActionBuilder ScoreTwo = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(50, 50));
        TrajectoryActionBuilder Four = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(38, 28))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(48, 26));
        TrajectoryActionBuilder Sleep = drive.actionBuilder(initialPose)
                .waitSeconds(4);
        TrajectoryActionBuilder SleepLong = drive.actionBuilder(initialPose)
                .waitSeconds(1.25);
        TrajectoryActionBuilder Five = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(48, 26))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(57, 26));
        TrajectoryActionBuilder Final = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(40, 40))
                .turnTo(Math.toRadians(-90));


        // actions that need to happen on init; for instance, a claw tightening.
        claw.SetWristRight();

        Action RunFirst = One.build();
        Action scoreFirst = FirstScore.build();
        Action RunSecond = Two.build();
        Action RunScoreOne = ScoreOne.build();
        Action RunScoreTwo = ScoreTwo.build();
        Action RunFourth = Four.build();
        Action Wait = Sleep.build();
        Action WaitLong = SleepLong.build();
        Action Finish = Final.build();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        //Rotate Wrist
                        new InstantAction(claw::SetWristRight),
                        new ParallelAction(
                                //claw.rightWrist(),
                                //new InstantAction(() -> arm.setArm(DriveConstants.armSpecimenClip)),
                                //new InstantAction(() -> arm.setOutArm(DriveConstants.armOutSpecimenClip)),
                                arm.upRest(),
                                arm.outRest()
                            /*new SequentialAction(
                                RunFirst
                            ),
                            arm.upHigh(),
                            new SequentialAction(
                                Wait
                            ),
                            arm.outHigh(),
                            new SequentialAction(
                                RunSecond
                            ),
                            new InstantAction(claw::grabberPlace),
                                    new SequentialAction(
                                        Wait
                                    ),
                                    arm.outRest(),
                                new SequentialAction(
                                        Wait
                                ),
                                    arm.upRest(),
                            new SequentialAction(
                                RunFourth
                            ),
                                arm.upPick(),
                                arm.outPick(),
                                new InstantAction(claw::grabberPick),
                                new SequentialAction(
                                        Wait
                                ),
                                new InstantAction(claw::grabberStop),
                            new SequentialAction(
                                RunScoreOne
                            )*/
                        ),
                        new InstantAction(claw::SetWristCenter),
                        RunFirst,
                        new ParallelAction(
                            arm.upHigh()
                        ),
                        Wait,
                        new ParallelAction(
                            arm.outHigh()
                        ),
                        scoreFirst,
                        new InstantAction(claw::grabberPlace),
                        Wait,
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
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BlueLeft", group = "Autonomous")
public class BlueLeft extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(15, 62, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder One = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(4, 38));
        TrajectoryActionBuilder Two = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(30, 40))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(38, 28));
        TrajectoryActionBuilder Three = drive.actionBuilder(initialPose)
                .turnTo(Math.toRadians(45))
                .strafeTo(new Vector2d(50, 50))
                .waitSeconds(3);


        // actions that need to happen on init; for instance, a claw tightening.


        Action RunFirst = One.build();
        Action RunSecond = Two.build();
        Action RunThird = Three.build();
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        RunFirst,
                        RunSecond,
                        RunThird
                )
        );
    }
}
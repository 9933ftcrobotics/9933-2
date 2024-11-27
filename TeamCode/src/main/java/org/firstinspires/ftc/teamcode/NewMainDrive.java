package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp
public class NewMainDrive extends LinearOpMode {
    boolean wristCenter = true;


    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    private static ElapsedTime timmer = new ElapsedTime();
    private static ElapsedTime rumble = new ElapsedTime();
    private static ElapsedTime huskyTime = new ElapsedTime();
    private static ElapsedTime reset = new ElapsedTime();
    boolean warning = false;
    boolean endGame = false;

    static final boolean FIELD_CENTRIC = true;

    boolean sampleScoring = true; //true = sample false = specimin

    boolean YIsPressed = false;

    boolean upIsPressed = false;

    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    boolean retractArm = false;
    boolean retractUpArm = false;

    boolean started = false;

    int farPickPos = 1;
    boolean rightStickPressed = false;

    double leftY, leftX, rightX;

    @Override
    public void runOpMode() throws InterruptedException {

        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER

        DriveSubsystem drive = new DriveSubsystem(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312),
                new RevIMU(hardwareMap),
                hardwareMap.get(HuskyLens.class, "huskyLens")
        );

        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
        );

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );
        CameraSubsystem camera = new CameraSubsystem(

        );

        drive.setReadType(); //Set Husky Cam to color mode
        arm.resetOutArm();
        rumble.reset();


        // the extended gamepad object
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);


        waitForStart();
        //camera.initAprilTag();

        while (!isStopRequested()) {
            if (!driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                rightStickPressed = false;
            }

            rightX = driver1.getRightX();
            leftX = driver1.getLeftX();
            leftY = driver1.getLeftY();


            arm.armCurrent();
            telemetry.addData("Current", DriveConstants.armCurrent);
            telemetry.addData("Target", ArmSubsystem.target);
            CommandScheduler.getInstance().run();
            updateTelemetry(drive.getDriveTelemetry());

            if (driver1.getButton(GamepadKeys.Button.Y) && YIsPressed == false || driver2.getButton(GamepadKeys.Button.Y) && YIsPressed == false) {
                telemetry.addLine("Y is pressed");
                if (sampleScoring == true) {
                    sampleScoring = false;
                    telemetry.addLine("sampleScoring is true. Changing to false");
                } else if (sampleScoring == false) {
                    sampleScoring = true;
                    telemetry.addLine("sampleScoring is false. Changing to true");
                }
                YIsPressed = true;
            }

            if (!driver1.getButton(GamepadKeys.Button.Y) && !driver2.getButton(GamepadKeys.Button.Y) ) {
                YIsPressed = false;
            }




            if (driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) {
                arm.powerOutArm(-driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                retractArm = true;
            } else if (driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) {
                arm.powerOutArm(-driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                retractArm = true;
            } else {
                retractArm = false;
            }


            if (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                arm.resetOutArm();
            }


                claw.SetWristCenter();

                if (!retractArm && !retractUpArm) {
                    if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN) || driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                        drive.drive(leftX * 0.75, leftY * 0.75, rightX * 0.75, true);
                        arm.setArm(DriveConstants.armSamplePickFar);
                        arm.setOutArm(DriveConstants.armOutSamplePickFar);
                        claw.grabberPick();
                    } /*else if (driver1.getButton(GamepadKeys.Button.DPAD_LEFT) || driver2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        drive.drive(leftX * 0.75, leftY * 0.75, rightX * 0.75, true);

                        arm.setArm(DriveConstants.armSamplePickFar);
                        arm.setOutArm(DriveConstants.armOutSamplePickFar);
                        //}
                        claw.grabberPick();
                        //}
                    }*/ else if (driver1.getButton(GamepadKeys.Button.DPAD_UP) || driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                        drive.drive(leftX * 0.5, leftY * 0.5, rightX * 0.5, true);
                        /*if (ArmSubsystem.upCurrent < DriveConstants.armSampleScoreHigh - 150) {
                            arm.setArm(DriveConstants.armSampleScoreHigh);
                            arm.setOutArm(50);
                        } else {
                            arm.setArm(DriveConstants.armSampleScoreHigh);
                            arm.setOutArm(DriveConstants.armOutSampleScoreHigh);
                        }*/
                        arm.setArm(DriveConstants.armSampleScoreHigh);
                        if (ArmSubsystem.upCurrent > 1000) {
                            arm.setArm(DriveConstants.armSampleScoreHigh);
                            arm.setOutArm(DriveConstants.armOutSampleScoreHigh);
                        }
                    } else if (driver1.getButton(GamepadKeys.Button.DPAD_RIGHT) || driver2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                        drive.drive(leftX * 0.5, leftY * 0.5, rightX * 0.5, true);
                        if (ArmSubsystem.upCurrent < DriveConstants.armSampleScoreLow - 65) {
                            arm.setArm(DriveConstants.armSampleScoreLow);
                            arm.setOutArm(50);
                        } else {
                            arm.setArm(DriveConstants.armSampleScoreLow);
                            arm.setOutArm(DriveConstants.armOutSampleScoreLow);
                        }
                    } else if (driver1.getButton(GamepadKeys.Button.A) || driver2.getButton(GamepadKeys.Button.A)) {
                        drive.drive(leftX * 0.75, leftY * 0.75, rightX * 0.75, true);
                        arm.setArm(DriveConstants.armSamplePick);
                        arm.setOutArm(DriveConstants.armOutSpecimenPick);
                        claw.grabberPick();
                    } else if (driver1.getButton(GamepadKeys.Button.Y) || driver2.getButton(GamepadKeys.Button.Y)) {
                        drive.drive(leftX, leftY, rightX, true);
                        upIsPressed = true;
                        arm.setArm(DriveConstants.armSpecimenClip);
                        arm.setOutArm(DriveConstants.armOutSpecimenClip);
                        timmer.reset();
                    } else {
                        drive.drive(leftX, leftY, rightX, true);
                        if (ArmSubsystem.outCurrent > 500) {
                            arm.setArm(ArmSubsystem.upCurrent);
                            arm.setOutArm(DriveConstants.armOutSampleRest);
                        } else {
                            arm.setArm(DriveConstants.armSampleRest);
                            arm.setOutArm(DriveConstants.armOutSampleRest);
                        }
                    }
                }

                if (!driver1.getButton(GamepadKeys.Button.DPAD_DOWN) && !driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    claw.grabberPlaceToPower(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
                } else {
                    if (!driver1.getButton(GamepadKeys.Button.DPAD_DOWN) && !driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && !driver1.getButton(GamepadKeys.Button.DPAD_LEFT) && !driver2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        claw.grabberStop();
                    }
                }


                //Move Wrist
                if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) || driver2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    if (wristCenter && !leftBumperPressed) {
                        claw.SetWristLeft();
                        leftBumperPressed = true;
                    } else if (!wristCenter && !leftBumperPressed) {
                        claw.SetWristCenter();
                        leftBumperPressed = true;
                    }
                } else if (!driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && !driver2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    leftBumperPressed = false;
                }


                /*if (upIsPressed == true) {
                    drive.drive(leftX, leftY, rightX, true);
                    if (timmer.seconds() < 2.5) {
                        arm.setArm(DriveConstants.armSpecimenClip);
                        arm.setOutArm(DriveConstants.armOutSpecimenClip);
                    } else if (timmer.seconds() < 4) {
                        arm.setArm(DriveConstants.armSpecimenClip - 75);
                        arm.setOutArm(DriveConstants.armOutSpecimenClip);
                    } else if (timmer.seconds() < 6) {
                        arm.setArm(800);
                        arm.setOutArm(DriveConstants.armOutSpecimenRest);
                    } else if (timmer.seconds() < 7) {
                        upIsPressed = false;
                    }
                }*/





            if (rumble.seconds() > 80 && !warning) {
                driver1.gamepad.rumbleBlips(3);
                warning = true;
            } else if (rumble.seconds() > 90 && !endGame) {
                driver1.gamepad.rumbleBlips(6);
                endGame = true;
            }
            telemetry.addData("Timmer For End Game", rumble);


            drive.getCurrentPose();

            updateTelemetry(drive.getDriveTelemetry());

            telemetry.update();


        }
    }
    public void updateTelemetry(String[] telem) {
        for (String s : telem) {
            telemetry.addLine(s);
        }
    }
}
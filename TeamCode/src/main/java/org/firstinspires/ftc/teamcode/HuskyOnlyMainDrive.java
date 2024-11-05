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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp
@Disabled
public class HuskyOnlyMainDrive extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static final boolean FIELD_CENTRIC = true;

    boolean sampleScoring; //true = sample false = specimin

    boolean YIsPressed = false;

    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;

    boolean started = false;

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

        /*ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312)
        );*/

        /*ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );
        CameraSubsystem camera = new CameraSubsystem(

        );*/

        drive.setReadType(); //Set Husky Cam to color mode

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)


        // the extended gamepad object
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);


        waitForStart();
        //camera.initAprilTag();

        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            //updateTelemetry(drive.getDriveTelemetry());
            //drive.huskyRead(DriveConstants.colorID);


            /*if(driver1.getButton(GamepadKeys.Button.A)) {
                DriveConstants.colorID = 1;
            }
            if(driver1.getButton(GamepadKeys.Button.B)) {
                DriveConstants.colorID = 2;
            }
            if(driver1.getButton(GamepadKeys.Button.X)) {
                DriveConstants.colorID = 3;
            }

            if(driver1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                DriveConstants.autoPickPosition = 1; //Left
            }
            if(driver1.getButton(GamepadKeys.Button.DPAD_UP)) {
                DriveConstants.autoPickPosition = 2; //Center
            }
            if(driver1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                DriveConstants.autoPickPosition = 3; //Right
            }*/

            //if (driver1.getButton(GamepadKeys.Button.Y)) {

                //double[] readHusky = drive.huskyReadOnly(DriveConstants.colorID);

                /*if (!DriveConstants.foundBlock) {
                    if (DriveConstants.autoPickPosition == 1) {
                        //strafe right
                        telemetry.addLine("I need to strafe right!");
                    } else if (DriveConstants.autoPickPosition == 2) {
                        //strafe right or left code
                        telemetry.addLine("I need to move right or left!");
                    } else if (DriveConstants.autoPickPosition == 3) {
                        //strafe left code
                        telemetry.addLine("I need to strafe left!");
                    }
                } else {*/
                    /*if (DriveConstants.xDis > 8 || DriveConstants.xDis < -8 || DriveConstants.yDis > 8 || DriveConstants.yDis < -8) {
                        drive.huskyReadDrive(2);
                        telemetry.addLine("I want to pick up a piece but it's too far!");
                    } else {
                        //arm.setArm(0);
                        //claw.grabberPick();
                        //sleep(1000);
                        //arm.setArm(150);
                        //claw.grabberStop();
                        telemetry.addLine("I want to pick up a piece because I can reach it!");
                    }*/

            if (DriveSubsystem.ready == true) {
                telemetry.addLine("I want to pick up a piece because I can reach it!");
            }
            drive.huskyReadDrive(2);

                    telemetry.addData("xDis", DriveConstants.xDis);
            telemetry.addData("yDis", DriveConstants.yDis);
                //}

            //}

            //telemetry.addLine(String.valueOf(readHusky[0]));
            //telemetry.addLine(String.valueOf(readHusky[1]));

            //drive.getCurrentPose();
            updateTelemetry(drive.getDriveTelemetry());

            telemetry.addData("color ID wanted", DriveConstants.colorID);
            telemetry.addData("pickup pos", DriveConstants.autoPickPosition);
            telemetry.update();
        }
    }
    public void updateTelemetry(String[] telem) {
        for (String s : telem) {
            telemetry.addLine(s);
        }
    }
}
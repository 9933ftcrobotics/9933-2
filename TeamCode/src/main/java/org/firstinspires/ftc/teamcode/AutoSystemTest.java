package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@TeleOp
@Disabled
public class AutoSystemTest extends LinearOpMode {

    int step = 0;

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

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
                new SimpleServo(hardwareMap, "wrist", 0, 1)
        );

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
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {

            drive.Drive_System_Test(step == 90, step == 100, step == 110,step == 120);

            if (driverOp.getButton(GamepadKeys.Button.A)) {
                step = 10;
            }

            if (driverOp.getButton(GamepadKeys.Button.B)) {
                step = 90;
            }

            //Arm and end effector
            switch (step) {
                case 10:
                    arm.setArm(500);
                    sleep(500);
                    step = 20;
                    break;
                case 20:
                    arm.setArm(0);
                    sleep(500);
                    step = 30;
                    break;
                case 30:
                    claw.SetWristRight();
                    sleep(500);
                    step = 40;
                    break;
                case 40:
                    claw.SetWristLeft();
                    sleep(500);
                    step = 50;
                    break;
                case 50:
                    claw.SetWristCenter();
                    sleep(500);
                    step = 60;
                    break;
                case 60:
                    claw.grabberPick();
                    sleep(500);
                    step = 70;
                    break;
                case 70:
                    claw.grabberPlace();
                    sleep(500);
                    step = 80;
                    break;
                case 80:
                    claw.grabberStop();
                    sleep(500);
                    step = 0;
                    break;
            }

            //Drive-Train
            switch (step) {
                case 90:
                    sleep(500);
                    step = 100;
                    break;
                case 100:
                    sleep(500);
                    step = 110;
                    break;
                case 110:
                    sleep(500);
                    step = 120;
                    break;
                case 120:
                    sleep(500);
                    step = 0;
                    break;
            }

            telemetry.addLine("Press A to start arm and end effector test. Press B to test Wheels. \n" +
                    "Wheels will spin in READ order. \n" +
                    "1: Left Front \n" +
                    "2: Right Front \n" +
                    "3: Left Rear \n" +
                    "4: Right Rear");
            telemetry.update();


        }
    }

}
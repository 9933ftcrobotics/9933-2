package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@TeleOp
public class simpleSystemTest extends LinearOpMode {
    double WheelPower = 1; //This will change
    int armPower;
    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER


        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
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



            //Wheel Speed
            //Arm
            if (driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
                //arm.setOutArm(100);
            } else if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                //arm.setOutArm(200);
            } else {
                //arm.setOutArm(0);
            }
            arm.powerArm(driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            arm.powerArm(-driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            telemetry.addData("Wheel Power. Press Right Bumper To Change. Either 1 or 0.25.", WheelPower);
            telemetry.addLine("A=Right Rear, B=Right Front, Y=Left Front X=Left Rear \n " +
                    "Dpad Left=Wrist Left, Dpad Right=Wrist Right, neither=Wrist center \n" +
                    "Dpad Up=Arm Power=1, Dpad Down=Arm Power = -1/Backwards, WARNING! NEITHER DPAD UP OR DOWN, ARM POWER=0 \n" +
                    "Left Stick Button=Grabber Pick Up, Right Stick Button=Grabber Put Down, ELSE, grabber stop");
            telemetry.update();


        }
    }

}
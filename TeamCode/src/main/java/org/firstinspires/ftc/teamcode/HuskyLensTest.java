package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensSubsystem;

@TeleOp
public class HuskyLensTest extends LinearOpMode {
    private HuskyLens huskyLens;

    int XCenter = 160;
    int YCenter = 120;

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime myElapsedTime;
        HuskyLens.Block[] myHuskyLensBlocks;
        HuskyLens.Block myHuskyLensBlock;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        //CommandScheduler.getInstance().run();

        DriveSubsystem drive = new DriveSubsystem(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312),
                new RevIMU(hardwareMap)
        );

        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312)
        );

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        /*HuskyLensSubsystem husky = new ClawSubsystem(
                new HuskyLens(hardwareMap, "huskyLens")
        );*/
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

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        myElapsedTime = new ElapsedTime();

        waitForStart();

        while (!isStopRequested()) {

            if (myElapsedTime.seconds() >= 1) {
                myElapsedTime.reset();
                myHuskyLensBlocks = huskyLens.blocks();
                telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
                for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks) {
                    int XPower = 0;
                    int YPower = 0;
                    myHuskyLensBlock = myHuskyLensBlock_item;
                    telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y);
                    if (myHuskyLensBlock.width > myHuskyLensBlock.height) {
                        if (myHuskyLensBlock.x > XCenter) {
                            XPower = 1;
                        } else {
                            XPower = 0;
                        }
                        if (myHuskyLensBlock.x < XCenter) {
                            XPower = -1;
                        } else {
                            XPower = 0;
                        }
                        if (myHuskyLensBlock.y < YCenter) {
                            YPower = 1;
                        } else {
                            YPower = 0;
                        }
                        if (myHuskyLensBlock.y > YCenter) {
                            YPower = -1;
                        } else {
                            YPower = 0;
                        }
                        //drive.drive(XPower, YPower, 0, false);
                        telemetry.addData("Y", YPower);
                        telemetry.addData("X", XPower);
                    }
                }
                telemetry.update();
            }



            telemetry.update();
        }
    }
}
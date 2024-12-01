package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    public static boolean ready = false;

    double XPower = 0;
    double YPower = 0;

    int XCenter = 160;
    int YCenter = 120;

    double XkP = 0.004;
    double YkP = 0.004;

    double imuAngle;

    private Motor leftFront, rightFront, rightRear, leftRear;
    private int leftEncoder, rightEncoder, backEncoder;
    private GyroEx gyro;
    private HolonomicOdometry odometry;
    private DifferentialDriveKinematics kinematics;
    private MecanumDrive drive;
    private RevIMU imu;

    private HuskyLens huskyLens;

    ElapsedTime myElapsedTime;
    HuskyLens.Block[] myHuskyLensBlocks;
    HuskyLens.Block myHuskyLensBlock;


    //String[] telemetry = new String[]{
    //"Left Encoder: ",
    //"Right Encoder: ",
    //"Back Encoder: ",
    //"XPower HuskyLens: ",
    //"YPower HuskyLens: ",
    //"xPos: ",
    //"yPos: ",
    //"posAngle"
    //};


    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor rightRear, Motor leftRear, RevIMU imu, HuskyLens huskyLens) {
        leftFront.setInverted(true);
        leftRear.setInverted(true);
        rightFront.setInverted(false);
        //rightRear.setInverted(false);
        rightRear.setInverted(false);
        drive = new MecanumDrive(false,
                leftFront, rightFront, leftRear, rightRear
        );

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.imu = imu;
        this.huskyLens = huskyLens;

        imu.init();

        odometry = new HolonomicOdometry(leftFront::getCurrentPosition, rightFront::getCurrentPosition, leftRear::getCurrentPosition, DriveConstants.TRACK_WIDTH, DriveConstants.CENTER_WHEEL_OFFSET);
        kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
    }

    @Override
    public void periodic() {
        //leftEncoder = leftFront.getCurrentPosition();
        //rightEncoder = rightFront.getCurrentPosition();
        //backEncoder = leftRear.getCurrentPosition();
        //telemetry[0] += "10000000";//leftFront.getCurrentPosition();
        //telemetry[1] += rightFront.getCurrentPosition();
        //telemetry[2] += leftRear.getCurrentPosition();
        //telemetry[5] += odometry.getPose().getX();
        //telemetry[6] += odometry.getPose().getY();
        //telemetry[7] += odometry.getPose().getRotation();
        imuAngle = imu.getRotation2d().getDegrees();
        //odometry.updatePose();
    }



    public Pose2d getCurrentPose() {
        return odometry.getPose();
    }

    public void drive(double LeftX, double LeftY, double RightX, boolean FIELD_CENTRIC) {
        // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
        // These are related to the left stick x value, left stick y value, and
        // right stick x value respectively. These values are passed in to represent the
        // strafing speed, the forward speed, and the turning speed of the robot frame
        // respectively from [-1, 1].

        if (!FIELD_CENTRIC) {

            // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
            // will move the robot in the direction of its current heading. Every movement
            // is relative to the frame of the robot itself.
            //
            //                 (0,1,0)
            //                   /
            //                  /
            //           ______/_____
            //          /           /
            //         /           /
            //        /___________/
            //           ____________
            //          /  (0,0,1)  /
            //         /     ↻     /
            //        /___________/

            // optional fourth parameter for squared inputs
            drive.driveRobotCentric(
                    LeftX,
                    LeftY,
                    RightX,
                    false
            );
        } else {

            // Below is a model for how field centric will drive when given the inputs
            // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
            // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
            // regardless of the heading.
            //
            //                   heading
            //                     /
            //            (0,1,0) /
            //               |   /
            //               |  /
            //            ___|_/_____
            //          /           /
            //         /           / ---------- (1,0,0)
            //        /__________ /

            // optional fifth parameter for squared inputs
            drive.driveFieldCentric(
                    LeftX,
                    LeftY,
                    RightX,
                    imu.getRotation2d().getDegrees()
            );
        }
    }

    public void Drive_System_Test(boolean run_leftFront, boolean run_rightFront, boolean run_leftRear, boolean run_rightRear){
        drive.driveWithMotorPowers((run_leftFront ? 1.0:0.0),(run_rightFront ? 1.0:0.0),(run_leftRear ? 1.0:0.0),(run_rightRear ? 1.0:0.0) );
    }

    public void setReadType() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        myElapsedTime = new ElapsedTime();

    }
    public void huskyReadDrive(int colorId) {
        boolean PowerSent = false;

        if (myElapsedTime.seconds() >= 0.01) {
            myElapsedTime.reset();
            myHuskyLensBlocks = huskyLens.blocks();
            for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks ) {
                myHuskyLensBlock = myHuskyLensBlock_item;
                DriveConstants.xDis = XCenter - myHuskyLensBlock.x;
                DriveConstants.yDis = YCenter - myHuskyLensBlock.y;
                if (myHuskyLensBlock.width > myHuskyLensBlock.height) {
                    if (myHuskyLensBlock.id == colorId /*&& !PowerSent*/) {
                        if (DriveConstants.xDis > 8 || DriveConstants.xDis < -8 || DriveConstants.yDis > 8 || DriveConstants.yDis < -8) {
                            DriveConstants.foundBlock = true;
                            XPower = -DriveConstants.xDis * XkP;
                            YPower = DriveConstants.yDis * YkP;
                            drive(XPower, YPower, 0, false);
                            ready = false;
                            PowerSent = true;
                        } else {
                            ready = true;
                            drive(0, 0, 0, false);
                        }
                    } else {
                        drive(0, 0, 0, false);
                    }
                } else {
                    drive(0, 0, 0, false);
                }
            }
        }
    }

    public double[] huskyReadOnly(int colorId) {
        double[] huskyData = {0, 0};
        boolean PowerSent = false;
        if (myElapsedTime.seconds() >= 0.1) {
            myElapsedTime.reset();
            myHuskyLensBlocks = huskyLens.blocks();

            for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks ) {
                myHuskyLensBlock = myHuskyLensBlock_item;
                DriveConstants.xDis = XCenter - myHuskyLensBlock.x;
                DriveConstants.yDis = YCenter - myHuskyLensBlock.y;
                if (myHuskyLensBlock.width > myHuskyLensBlock.height) {
                    //if (DriveConstants.xDis > 8 && DriveConstants.xDis < -8 && DriveConstants.yDis > 8 && DriveConstants.yDis < -8) {
                    if (myHuskyLensBlock.id == colorId /*&& !PowerSent*/) {
                        DriveConstants.foundBlock = true;
                        huskyData[0] = DriveConstants.xDis * XkP;
                        huskyData[1] = DriveConstants.yDis * YkP;
                        //PowerSent = true;
                    }
                    //}
                }
            }
        }
        return huskyData;
    }

    public void starfeRight() {
        leftFront.set(1);
        leftRear.set(-1);
        rightFront.set(-1);
        rightRear.set(1);
    }
    public void starfeLeft() {
        leftFront.set(-1);
        leftRear.set(1);
        rightFront.set(1);
        rightRear.set(-1);
    }

    public String[] getDriveTelemetry() {
        String[] telem = {

                "Robot Angle: " + String.valueOf(imuAngle),
                "Robot Auto Offset Angle: " + String.valueOf(DriveConstants.angle),
                "Left Odom Pod: " + String.valueOf(leftEncoder),
                "Right Odom Pod: " + String.valueOf(rightEncoder),
                "Back Odom Pod: " + String.valueOf(backEncoder)
                /*"Front Left Pow: " + String.valueOf(leftFront.get()),
                "Front Right Pow: " + String.valueOf(rightFront.get()),
                "Rear Left Pow: " + String.valueOf(leftRear.get()),
                "Rear Right Pow: " + String.valueOf(rightRear.get())*/

        };
        return telem;
    }
}
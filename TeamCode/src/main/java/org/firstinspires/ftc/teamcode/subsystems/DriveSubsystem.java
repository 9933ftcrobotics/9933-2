package org.firstinspires.ftc.teamcode.subsystems;

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
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private Motor leftFront, rightFront, rightRear, leftRear;
    private double leftEncoder, rightEncoder, backEncoder;
    private GyroEx gyro;
    private DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics;
    private final MecanumDrive drive;
    private RevIMU imu;

    public DriveSubsystem(Motor leftFront, Motor rightFront,  Motor leftRear,Motor rightRear, RevIMU imu) {
        leftFront.setInverted(true);
        leftRear.setInverted(true);
        rightFront.setInverted(false);
        rightRear.setInverted(false);
        drive = new MecanumDrive(false,
                leftFront, rightFront, leftRear, rightRear
        );
        this.imu = imu;
        imu.init();

        leftFront.set(0);
        rightFront.set(0);
        leftRear.set(0);
        rightRear.set(0);

        kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
    }

    @Override
    public void periodic() {
        leftEncoder = leftFront.getCurrentPosition();
        rightEncoder = leftFront.getCurrentPosition();
        backEncoder = leftFront.getCurrentPosition();
    }


    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
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

    public String[] getDriveTelemetry() {
        String[] telem = {
                "Robot Angle:" + String.valueOf((imu.getRotation2d().getDegrees())),
                "Left Odom Pod: " + String.valueOf(leftEncoder),
                "Right Odom Pod: " + String.valueOf(rightEncoder),
                "Back Odom Pod: " + String.valueOf(backEncoder)
//                "Front Left Pow: " + String.valueOf(leftFront.get()),
//                "Front Right Pow: " + String.valueOf(rightFront.get()),
//                "Rear Left Pow: " + String.valueOf(leftRear.get()),
//                "Rear Right Pow: " + String.valueOf(rightRear.get())


        };
        return telem;
    }

    public void Drive_System_Test(boolean run_leftFront, boolean run_rightFront, boolean run_leftRear, boolean run_rightRear){
        drive.driveWithMotorPowers((run_leftFront ? 1.0:0.0),(run_rightFront ? 1.0:0.0),(run_leftRear ? 1.0:0.0),(run_rightRear ? 1.0:0.0) );
    }
}

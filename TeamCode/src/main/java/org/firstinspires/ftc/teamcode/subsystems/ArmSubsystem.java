package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class ArmSubsystem extends SubsystemBase {
    private Motor arm;
    private Motor outArm;
    private double targetPos = 0;
    private double targetPos2 = 0;
    private double outTargetPos = 0;
    public static int outCurrent;

    public static double f = 1;

    public static int target = 0;
    public static int outTarget = 0;

    private final double ticks_in_degree =  5281.1 / 360;

    public ArmSubsystem(Motor arm, Motor outArm) {
        arm.setInverted(false);
        outArm.setInverted(true);
        this.arm = arm;
        this.outArm = outArm;
        arm.resetEncoder();
        outArm.resetEncoder();
    }

    public void setArm(int Pos) {

        targetPos = Pos;
        // set the run mode

        PIDController pid = new PIDController(0.001,0,0);
        double ff = Math.cos(Math.toRadians(Pos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop
        if (Math.abs(Pos-arm.getCurrentPosition()) > tolerence) {
            arm.set(pid.calculate(arm.getCurrentPosition(),Pos) + ff);
        } else {
            arm.stopMotor(); // stop the motor
        }
    }

    public void setOutArm(int outPos) {

        outCurrent = outArm.getCurrentPosition();
        outTargetPos = outPos;
        // set the run mode

        PIDController pid = new PIDController(0.005,0,0);
        double ff = Math.cos(Math.toRadians(outPos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop
        if (Math.abs(outPos-arm.getCurrentPosition()) > tolerence) {
            outArm.set(pid.calculate(arm.getCurrentPosition(),outPos) + ff);
        } else {
            outArm.stopMotor(); // stop the motor
        }
    }

    public void powerArm(double Power) {
        arm.set(Power);
    }

    public void armCurrent() {
        DriveConstants.armCurrent = arm.getCurrentPosition();
    }


    public void resetOutArm() {outArm.resetEncoder();}

    public String[] getArmTelemetry() {
        return new String[]{
                ("Current Arm Pos: " + String.valueOf(arm.getCurrentPosition())),
                ("Target Arm Pos: " + String.valueOf(targetPos)),
                ("Current Out Arm Pos: " + String.valueOf(outArm.getCurrentPosition())),
                ("Target Out Arm Pos: " + String.valueOf(outTargetPos))

        };
    }

}
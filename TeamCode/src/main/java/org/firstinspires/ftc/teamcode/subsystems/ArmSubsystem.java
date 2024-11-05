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
    private double outTargetPos = 0;

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

        PIDController pid = new PIDController(0.005,0,0);


        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop
        if (Math.abs(Pos-arm.getCurrentPosition()) > tolerence) {
            arm.set(pid.calculate(arm.getCurrentPosition(),Pos));
        } else {
            arm.stopMotor(); // stop the motor
        }
    }

    public void setOutArm(int outPos) {

        targetPos = outPos;
        // set the run mode

        PIDController pid = new PIDController(0.005,0,0);


        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop
        if (Math.abs(outPos-arm.getCurrentPosition()) > tolerence) {
            arm.set(pid.calculate(arm.getCurrentPosition(),outPos));
        } else {
            arm.stopMotor(); // stop the motor
        }
    }

    public void powerArm(double Power) {
        arm.set(Power);
    }

    public void armCurrent() {
        DriveConstants.armCurrent = arm.getCurrentPosition();
    }

    public String[] getArmTelemetry() {
        return new String[]{
                ("Current Arm Pos: " + String.valueOf(arm.getCurrentPosition())),
                ("Target Arm Pos: " + String.valueOf(targetPos)),
                ("Current Out Arm Pos: " + String.valueOf(arm.getCurrentPosition())),
                ("Target Out Arm Pos: " + String.valueOf(outTargetPos))

        };
    }

}
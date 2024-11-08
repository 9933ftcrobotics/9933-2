package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class ArmSubsystem extends SubsystemBase {
    private Motor arm;
    private Motor outArm;

    private PIDController controller;

    private double targetPos = 0;
    private double targetPos2 = 0;
    private double outTargetPos = 0;
    public static int outCurrent;
    public static int upCurrent;

    //public static double f = 1;

    public static int target = 0;
    public static int outTarget = 0;

    private final double ticks_in_degree =  5281.1 / 360;

    public static double p = 0.05, i = 0, d = 0;
    public static double f = 1;

    public ArmSubsystem(Motor arm, Motor outArm) {
        arm.setInverted(false);
        outArm.setInverted(true);
        this.arm = arm;
        this.outArm = outArm;
        arm.resetEncoder();
        outArm.resetEncoder();
        controller = new PIDController(p,i,d);
    }           

    public void setArm(int Pos) {
        upCurrent = arm.getCurrentPosition();
        /*targetPos = Pos;
        // set the run mode

        PIDController pid = new PIDController(0.05,40,0);
        double ff = Math.cos(Math.toRadians(Pos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop*/
        /*if (Math.abs(Pos-arm.getCurrentPosition()) > tolerence) {
            arm.set(pid.calculate(arm.getCurrentPosition(),Pos) + ff);
        } else {
            arm.stopMotor(); // stop the motor
        }*/

        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();

        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree))*f;


        double power = pid + ff;

        arm.set(power);

    }

    public void setOutArm(int outPos) {

        outCurrent = outArm.getCurrentPosition();
        outTargetPos = outPos;
        // set the run mode

        PIDController pid = new PIDController(0.005,0,0);
        //double ff = Math.cos(Math.toRadians(outPos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 13.6;   // allowed maximum error
        // perform the control loop
        if (Math.abs(outPos-outArm.getCurrentPosition()) > tolerence) {
            outArm.set(pid.calculate(outArm.getCurrentPosition(),outPos)/* + ff*/);
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
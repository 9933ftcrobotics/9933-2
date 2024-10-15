package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class ArmSubsystem extends SubsystemBase {
    private Motor arm;
    private double targetPos = 0;

    public ArmSubsystem(Motor arm) {
        arm.setInverted(false);
        this.arm = arm;
        arm.resetEncoder();
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

    public void powerArm(int Power) {
        arm.set(Power);
    }

    public String[] getArmTelemetry() {
        return new String[]{
                ("Current Arm Pos: " + String.valueOf(arm.getCurrentPosition())),
                ("Target Arm Pos: " + String.valueOf(targetPos))

        };
    }

}
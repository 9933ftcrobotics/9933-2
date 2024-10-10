package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class ArmSubsystem extends SubsystemBase {
    private Motor arm;
    public ArmSubsystem(Motor arm) {
        arm.setInverted(false);
        this.arm = arm;
        arm.resetEncoder();
    }

    public void setArm(int Pos) {
        // set the run mode
        arm.setRunMode(Motor.RunMode.PositionControl);

// set and get the position coefficient
        arm.setPositionCoefficient(0.01);
        double kP = arm.getPositionCoefficient();

// set the target position
        arm.setTargetPosition(Pos);      // an integer representing
        // desired tick count

        arm.set(0);

// set the tolerance
        arm.setPositionTolerance(13.6);   // allowed maximum error

// perform the control loop
        while (!arm.atTargetPosition()) {
            arm.set(1);
        }
        arm.stopMotor(); // stop the motor

        /* ALTERNATIVE TARGET DISTANCE */

// configure a distance per pulse,
// which is the distance traveled in a single tick
// dpp = distance traveled in one rotation / CPR
        /*arm.setDistancePerPulse(0.015);

// set the target
        arm.setTargetDistance(18.0);

// this must be called in a control loop
        arm.set(0.5); // mode must be PositionControl*/
    }

    public void powerArm(int Power) {
        arm.set(Power);
    }

    public double ArmCurrent() {
        return arm.getCurrentPosition();
    }

    public double TargetArm() {
        return arm.get();
    }


    @Override
    public void periodic() {

    }
}
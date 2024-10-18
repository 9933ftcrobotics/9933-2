package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.hardware.dfrobot.HuskyLens;

public class HuskyLensSubsystem extends SubsystemBase {
    private HuskyLens huksy;

    public HuskyLensSubsystem(HuskyLens husky) {
        this.huksy = husky;

    }
}
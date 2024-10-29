package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraSubsystem extends SubsystemBase {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -10,3, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;



    public CameraSubsystem() {
        //builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    }

    @Override
    public void periodic() {

    }

    public void readCam() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            //If not found tag yet
            //if (!DriveConstants.foundTag) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DriveConstants.DESIRED_TAG_ID < 0) || (detection.id == DriveConstants.DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    DriveConstants.targetFound = true;
                    DriveConstants.desiredTag = detection;
                    DriveConstants.xCameraPos = detection.center.x;
                    DriveConstants.yCameraPos = detection.center.y;
                    DriveConstants.yawCameraPos = detection.rawPose.z;
                    //DriveConstants.foundTag = true;
                    //telemetry.addLine("Found Tag");
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    //telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    DriveConstants.targetFound = false;
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                //telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
            //} //else if (detection.id != 11 && detection.id != 12 && detection.id != 13 && detection.id != 14 && detection.id != 15 && detection.id != 16 && !DriveConstants.driving) {
            //DriveConstants.foundTag = false;
            //}
        }
    }


    public void initAprilTag() {
        ///More info in the april tag sample code

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                .setCameraPose(cameraPosition, cameraOrientation)


                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

    }   // end method initAprilTag()






}

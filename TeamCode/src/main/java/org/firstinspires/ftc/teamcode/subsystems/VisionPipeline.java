package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.tan;
import static java.lang.Math.atan;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
public class VisionPipeline extends Robot.HardwareDevices {
    HardwareMap hardwareMap;
    TfodProcessor tfod;
    VisionPortal visionPortal;
    Telemetry telemetry;
    AprilTagProcessor aprilTag;
    String webcamName;
    Orientation robotOrientation;
    int cameraAngleOffset;
    Boolean init = false;

    public void VisionPipelineInit(HardwareMap hardwareMap, Telemetry telemetry, String webcamName, int cameraAngleOffset) {
        this.hardwareMap = hardwareMap;
        this.tfod = null;
        this.visionPortal = null;
        this.telemetry = telemetry;
        this.aprilTag = null;
        this.webcamName = webcamName;
        this.robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.cameraAngleOffset = cameraAngleOffset;
        this.init = true;
    }
    public void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, webcamName), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        telemetry.update();

    }   // end method telemetryTfod()

    public void initAprilTag() {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, webcamName), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.update();

    }   // end method telemetryAprilTag()

    private List<AprilTagDetection> AprilTagDetect() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        return currentDetections;
    }

    class CameraAngle {
        int id;
        double angle;
        public CameraAngle(int id, double angle) {
            this.id = id;
            this.angle = angle;
        }
    }
    public List<CameraAngle> aprilTagPos() {
        double cameraWidth = visionPortal.getActiveCamera().getCameraCharacteristics().getDefaultSize(0).getWidth();
        double cameraFOV = 78; // not sure if this is correct
        List<CameraAngle> Angles = null;
        for (AprilTagDetection detection : this.AprilTagDetect()) {
            double angle = Math.toDegrees(Math.atan(2 * Math.tan(Math.toRadians(cameraFOV / 2)) * (detection.center.x - (cameraWidth / 2) / (cameraWidth / 2))));
            angle = robotOrientation.firstAngle + (cameraAngleOffset + angle);
            if (angle < 0) {
                angle = 360 + angle;
            }
            Angles.add(new CameraAngle(detection.id, angle));
        }
        return Angles;
    }
}

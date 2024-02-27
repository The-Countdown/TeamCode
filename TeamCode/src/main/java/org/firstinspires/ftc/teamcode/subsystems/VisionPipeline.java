package org.firstinspires.ftc.teamcode.subsystems;
// there is still camera error stuff that i need to fix
import static java.lang.Math.decrementExact;
import static java.lang.Math.tan;
import static java.lang.Math.atan;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
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

import java.util.ArrayList;
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
    Boolean streaming = false;

    public void VisionPipelineInit(HardwareMap hardwareMap, Telemetry telemetry, String webcamName, int cameraAngleOffset) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.webcamName = webcamName;
        this.robotOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.cameraAngleOffset = cameraAngleOffset;
        this.init = true;
    }

    public void streamingOn() {
        if (!this.streaming) {
            this.visionPortal.resumeStreaming();
            this.streaming = true;
        }
    }

    public void streamingOff() {
        if (this.streaming) {
            this.visionPortal.stopStreaming();
            this.streaming = false;
        }
    }

    public void toggleStreaming() {
        if (this.streaming) {
            this.visionPortal.stopStreaming();
            this.streaming = false;
        } else {
            this.visionPortal.resumeStreaming();
            this.streaming = true;
        }
    }
    public void initTfod() {
        // Create the TensorFlow processor the easy way.
        this.tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            this.visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, webcamName), tfod);
        } else {
            this.visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()
    public void telemetryTfod() {
        List<Recognition> currentRecognitions = this.tfod.getRecognitions();
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
        this.aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            this.visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, webcamName), this.aprilTag);
        } else {
            this.visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, this.aprilTag);
        }
        // find a way to get camera is streaming
    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        if (!this.streaming) {
            this.streamingOn();
        }
        List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
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
        this.streamingOff();
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
        telemetry.addData("Trying to ", "detect april tags");
        CameraCharacteristics camera = hardwareMap.get(WebcamName.class, webcamName).getCameraCharacteristics();
        double width = camera.getDefaultSize(camera.getAndroidFormats()[camera.getAndroidFormats().length - 1]).getWidth(); // width of the camera
        width = camera.getAllCameraModes().get(0).size.getWidth();
        double fov = Math.toRadians(78); // field of view of the camera in radians
        List<CameraAngle> Angles = new ArrayList<>();
        for (AprilTagDetection detection : this.AprilTagDetect()) {
            double tx = detection.center.x;
            double ty = detection.center.y;
            double angle = Math.abs(Math.toDegrees(Math.atan(ty / Math.abs((width / 2) - tx) / Math.tan(fov / 2))) - 90);
//            angle = robotOrientation.firstAngle + (cameraAngleOffset + angle);
//            if (angle < 0) {
//                angle = 360 + angle;
//            }
            telemetry.addData("April id: ", detection.id);
            telemetry.addData("Camera Width: ", width);
            telemetry.addData("Camera FOV (radians)", fov);
            telemetry.addData("targetPosition: ", tx);
            telemetry.addData("targetPosition: ", ty);
            telemetry.addData("Detection Angle: ", angle);
            telemetry.addData("Robot Angle: ", robotOrientation.firstAngle);
            telemetry.addData("cameraAngleOffset", cameraAngleOffset);
            Angles.add(new CameraAngle(detection.id, angle));
        }
        telemetry.update();
        return Angles;
    }
}

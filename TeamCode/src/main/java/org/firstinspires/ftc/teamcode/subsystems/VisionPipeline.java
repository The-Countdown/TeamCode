package org.firstinspires.ftc.teamcode.subsystems;
// there is still camera error stuff that i need to fix
import static java.lang.Math.decrementExact;
import static java.lang.Math.nextAfter;
import static java.lang.Math.tan;
import static java.lang.Math.atan;

import android.hardware.camera2.CameraDevice;
import android.util.Size;

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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class VisionPipeline extends Robot.HardwareDevices {
    HardwareMap hardwareMap;
    TfodProcessor tfod;
    VisionPortal visionPortal;
    Telemetry telemetry;
    AprilTagProcessor aprilTag;
    String webcamName;
    int viewportContainerId;
    Orientation robotOrientation;
    int cameraAngleOffset;
    Boolean init = false;
    OpenCvCamera camera;

    public VisionPipeline(HardwareMap hardwareMap, Telemetry telemetry, String webcamName, int cameraAngleOffset, int viewportContainerId, VisionPortal visionPortal) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.webcamName = webcamName;
        //this.robotOrientation = imu.getAngularOrientation();
        this.viewportContainerId = viewportContainerId;
        this.cameraAngleOffset = cameraAngleOffset;
        this.visionPortal = visionPortal;

        initAprilTag();

        this.init = true;
    }

    public void initAprilTag() {
        try {
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));
            builder.setCameraResolution(new Size(320, 240));
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
            builder.addProcessor(aprilTag);
            builder.setLiveViewContainerId(viewportContainerId);
            visionPortal = builder.build();
            visionPortal.resumeStreaming();
        } catch (Exception e) {
            while (true) {
                telemetry.addData("cam error:" + e.getMessage() + "\nwebcamName: " + webcamName + "\nviewportContainerId: " + viewportContainerId, "");
                telemetry.update();
            }
        }
    }
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

    class AprilOffset {
        int id;
        double ftcposx;
        double ftcposy;
        double ftcposz;
        double pitch;
        double roll;
        double yaw;
        public AprilOffset(int id, double ftcposx, double ftcposy, double ftcposz, double pitch, double roll, double yaw) {
            this.id = id;
            this.ftcposx = ftcposx;
            this.ftcposy = ftcposy;
            this.ftcposz = ftcposz;
            this.pitch = pitch;
            this.roll = roll;
            this.yaw = yaw;
        }
    }
    public List<CameraAngle> aprilTagPos() {
//        telemetry.addData("Trying to ", "detect april tags");
        CameraCharacteristics camera = hardwareMap.get(WebcamName.class, webcamName).getCameraCharacteristics();
        double width = 320;
        double height = 240;
        double fov = Math.toRadians(78); // field of view of the camera in radians
        List<CameraAngle> Angles = new ArrayList<>();
        for (AprilTagDetection detection : this.AprilTagDetect()) {
            double tx = detection.center.x;
            double ty = detection.center.y;
            int focal_length = 500;  // Focal length of the camera

            double angle = 0;
            angle = Math.tan(width - tx / height - ty);
            if (tx > (width / 2)) {
                angle = -angle;
            }
//            angle = robotOrientation.firstAngle + (cameraAngleOffset + angle);
//            if (angle < 0) {
//                angle = 360 + angle;
//            }
//            telemetr y.addData("April id: ", detection.id);
//            telemetry.addData("Camera Width: ", width);
//            telemetry.addData("Camera FOV (radians)", fov);
//            telemetry.addData("targetPosition: ", tx);
//            telemetry.addData("targetPosition: ", ty);
//            telemetry.addData("Detection Angle: ", angle);
            //telemetry.addData("Robot Angle: ", robotOrientation.firstAngle);
//            telemetry.addData("cameraAngleOffset", cameraAngleOffset);
            Angles.add(new CameraAngle(detection.id, angle));
        }
//        telemetry.update();
        return Angles;
    }
    public List<AprilOffset> getData() {
//        telemetry.addData("Trying to ", "detect april tags");
        CameraCharacteristics camera = hardwareMap.get(WebcamName.class, webcamName).getCameraCharacteristics();
        List<AprilOffset> offsets = new ArrayList();
        for (AprilTagDetection detection : this.AprilTagDetect()) {

//            telemetr y.addData("April id: ", detection.id);
//            telemetry.addData("Camera Width: ", width);
//            telemetry.addData("Camera FOV (radians)", fov);
//            telemetry.addData("targetPosition: ", tx);
//            telemetry.addData("targetPosition: ", ty);
//            telemetry.addData("Detection Angle: ", angle);
            //telemetry.addData("Robot Angle: ", robotOrientation.firstAngle);
//            telemetry.addData("cameraAngleOffset", cameraAngleOffset);
            offsets.add(new AprilOffset(detection.id, detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        }
//        telemetry.update();
        return offsets;
    }
}

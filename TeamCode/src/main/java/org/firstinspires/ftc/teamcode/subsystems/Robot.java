package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class Robot {
    public static class HardwareDevices {
        //drive base
        public static DcMotorEx MotorFL; // this is the motor plugged into 0
        public static DcMotorEx MotorFR; // this is the motor plugged into 1
        public static DcMotorEx MotorBL; // this is the motor plugged into 2
        public static DcMotorEx MotorBR; // this is the motor plugged into 3

        //slide
        public static DcMotorEx LinearSlideR;
        public static DcMotorEx LinearSlideL;

        //arm (the servos)
        public static Servo Arm;
        public static Servo ClawR;
        public static Servo ClawL;

        //sensors
        public static BNO055IMU imu;
        public static DistanceSensor LeftDistance;
        public static DistanceSensor RightDistance;
        public static ColorSensor col;
        public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        HardwareDevices.MotorFL = hardwareMap.get(DcMotorEx.class, "MotorFL");
        HardwareDevices.MotorFR = hardwareMap.get(DcMotorEx.class, "MotorFR");
        HardwareDevices.MotorBL = hardwareMap.get(DcMotorEx.class, "MotorBL");
        HardwareDevices.MotorBR = hardwareMap.get(DcMotorEx.class, "MotorBR");
        HardwareDevices.LinearSlideL = hardwareMap.get(DcMotorEx.class, "ArmL");
        HardwareDevices.LinearSlideR = hardwareMap.get(DcMotorEx.class, "ArmR");
        HardwareDevices.Arm = hardwareMap.get(Servo.class, "ClawArm");
        HardwareDevices.ClawL = hardwareMap.get(Servo.class, "ClawHand1");
        HardwareDevices.ClawR = hardwareMap.get(Servo.class, "ClawHand2");
        HardwareDevices.imu = hardwareMap.get(BNO055IMU.class, "imu");
        HardwareDevices.col = hardwareMap.get(ColorSensor.class, "col");
        HardwareDevices.LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        HardwareDevices.RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        HardwareDevices.imu.initialize(parameters);

        HardwareDevices.MotorFL.setDirection(DcMotorEx.Direction.FORWARD);
        HardwareDevices.MotorFR.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.MotorBL.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.MotorBR.setDirection(DcMotorEx.Direction.REVERSE);

        HardwareDevices.LinearSlideL.setDirection(DcMotorEx.Direction.FORWARD);
        HardwareDevices.LinearSlideR.setDirection(DcMotorEx.Direction.REVERSE);

        vision1.VisionPipelineInit(hardwareMap, telemetry, "webcam 1", 90);
        vision1.initTfod();
        vision1.initAprilTag();
        vision2.VisionPipelineInit(hardwareMap, telemetry, "webcam 2", 270);
        vision2.initTfod();
        vision2.initAprilTag();
    }
    public Drive drive = new Drive();
    public LinearSlide slide = new LinearSlide();
    public Arm arm = new Arm();
    public Claw claw = new Claw();
    public VisionPipeline vision1 = new VisionPipeline();
    public VisionPipeline vision2 = new VisionPipeline();
    public Positioning robotPosition = new Positioning();
}
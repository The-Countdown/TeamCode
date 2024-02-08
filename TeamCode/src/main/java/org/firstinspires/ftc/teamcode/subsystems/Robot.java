package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
public class Robot {
    public static class Components {
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
    }

    public Robot(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        Components.MotorFL = hardwareMap.get(DcMotorEx.class, "MotorFL");
        Components.MotorFR = hardwareMap.get(DcMotorEx.class, "MotorFR");
        Components.MotorBL = hardwareMap.get(DcMotorEx.class, "MotorBL");
        Components.MotorBR = hardwareMap.get(DcMotorEx.class, "MotorBR");
        Components.LinearSlideL = hardwareMap.get(DcMotorEx.class, "ArmL");
        Components.LinearSlideR = hardwareMap.get(DcMotorEx.class, "ArmR");
        Components.Arm = hardwareMap.get(Servo.class, "ClawArm");
        Components.ClawL = hardwareMap.get(Servo.class, "ClawHand1");
        Components.ClawR = hardwareMap.get(Servo.class, "ClawHand2");
        Components.imu = hardwareMap.get(BNO055IMU.class, "imu");
        Components.col = hardwareMap.get(ColorSensor.class, "col");
        Components.LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        Components.RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        Components.imu.initialize(parameters);

        Components.MotorFL.setDirection(DcMotorEx.Direction.FORWARD);
        Components.MotorFR.setDirection(DcMotorEx.Direction.REVERSE);
        Components.MotorBL.setDirection(DcMotorEx.Direction.REVERSE);
        Components.MotorBR.setDirection(DcMotorEx.Direction.REVERSE);

        Components.LinearSlideL.setDirection(DcMotorEx.Direction.FORWARD);
        Components.LinearSlideR.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public Drive drive = new Drive();
    public LinearSlide slide = new LinearSlide();
    public Arm arm = new Arm();
    public Claw claw = new Claw();
}
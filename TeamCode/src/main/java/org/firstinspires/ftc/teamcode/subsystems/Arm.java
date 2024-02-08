package org.firstinspires.ftc.teamcode.subsystems;

public class Arm extends Robot.HardwareDevices {
    public static final class positions {
        public static final double pickup = 0.5;
        public static final double outOfTheWay = 0.32;
        public static final double scoring = 0.15;
        public static final double hanging = 0;
    }

    public void setPosition(double position) {
        Arm.setPosition(position);
        Arm.setPosition(position);
    }
}

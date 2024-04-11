package org.firstinspires.ftc.teamcode.subsystems;

public class LinearSlide extends Robot.HardwareDevices {
    public void stop() {
        LinearSlideR.setVelocity(0);
        LinearSlideL.setVelocity(0);
    }
    public void move(double velocity) {
        LinearSlideR.setVelocity(velocity);
        LinearSlideL.setVelocity(velocity);
    }

    public void move(double velocity, double amount) {
        double start = LinearSlideR.getCurrentPosition();

        move(velocity);

        if (amount > 0) {
            while (Math.abs(LinearSlideR.getCurrentPosition() - start) < amount) {
                //wait
            }
        } else {
            while (LinearSlideR.getCurrentPosition() - start > amount) {
                //wait
            }
        }

        stop();
    }
}

package org.firstinspires.ftc.teamcode.Library;

import org.firstinspires.ftc.teamcode.HardwareMaps.Sensors;
import org.firstinspires.ftc.teamcode.HardwareMaps.Drive;

public class MecanumEnhanced {
    private Drive drive = Drive.getInstance();
    private Sensors sensors = Sensors.getInstance();
    private double kP = 0.3;
    private boolean inAutonomous = false;

    public double[] calculatePowers(double magnitude, double x, double y, double targetHeading) {
        double[] powerRatios = calculatePowerRatios(x, y, targetHeading);
        return scalePowers(magnitude, powerRatios);
    }

    private double[] calculatePowerRatios(double x, double y, double heading) {
        double headingError = getHeadingError(heading);
        double leftFrontPower = PIDController.pController(y + x, headingError, -kP);
        double leftBackPower = PIDController.pController(y - x, headingError, -kP);
        double rightFrontPower = PIDController.pController(y - x, headingError, kP);
        double rightBackPower = PIDController.pController(y + x, headingError, kP);

        return new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};
    }

    private double[] scalePowers(double magnitude, double[] powerRatios) {
        double greatestMagnitude = getGreatestMagnitude(powerRatios);
        double ratio = magnitude/greatestMagnitude;
        double[] scaledPowers = new double[4];
        for (int i = 0; i < 4; i++) {
            scaledPowers[i] = powerRatios[i] * ratio;
        }
        return scaledPowers;
    }

    private double getGreatestMagnitude(double[] array) {
        double greatestMagnitude = 0;
        for (double value : array) {
            if (Math.abs(value) > greatestMagnitude) {
                greatestMagnitude = Math.abs(value);
            }
        }
       return greatestMagnitude;
    }
    
    private double getHeadingError(double heading) {
        double error;
        if (inAutonomous) {
            //In autonomous, heading is returned as an absolute number in degrees
            error = sensors.getError(heading);
        } else {
            /* In teleop, heading is relative and is returned as a joystick value from -1 to 1
            The multiplier of 45 allows the driver to make more significant corrections
            (assuming all the way to the right wants a correction of 45 degrees)
             */
            error = 15 / kP * heading;
        }
        return error;
    }

    public double getMaxVel(double magnitude, double x, double y) {
        double sum = x + y;
        double scaledX = x * drive.MAX_STRAFE_VELOCITY;
        double scaledY = y * drive.MAX_FORWARD_VELOCITY;
        return magnitude * (scaledX + scaledY)/sum;
    }

    public void setInAutonomous(boolean inAutonomous) {
        this.inAutonomous = inAutonomous;
    }
}

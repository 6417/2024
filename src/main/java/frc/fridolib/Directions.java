package frc.fridolib;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.Pair;

public class Directions {
    // public enum RawDirection {
    // NONE(0, 0),
    // UP(1, 0),
    // RIGHT(0, 1),
    // DOWN(-1, 0),
    // LEFT(0, -1),

    // public Direction(int x, int y) {
    // }
    // }

    public static enum Pov {
        UP(0),
        UP_RIGHT(45),
        RIGHT(90),
        DOWN_RIGHT(135),
        DOWN(180),
        DOWN_LEFT(225),
        LEFT(270),
        UP_LEFT(315);

        private final int angle;

        private Pov(int angle) {
            this.angle = angle;
        }

        public int getDegrees() {
            return angle;
        }

        public double getRadians() {
            return Math.toRadians(angle);
        }
    }

    public static enum JoystickLeft {
        RIGHT(90);

        private final int angle;

        private JoystickLeft(int angle) {
            this.angle = angle;
        }

        public int getDegrees() {
            return angle;
        }

        public double getRadians() {
            return Math.toRadians(angle);
        }
    }
}

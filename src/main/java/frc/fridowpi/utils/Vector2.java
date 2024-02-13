package frc.fridowpi.utils;

public class Vector2 {
    public double x;
    public double y;

    static final Vector2 ZERO = new Vector2(0, 0);
    static final Vector2 LEFT = new Vector2(-1, 0);
    static final Vector2 RIGHT = new Vector2(1, 0);
    static final Vector2 UP = new Vector2(0, 1);
    static final Vector2 DOWN = new Vector2(0, -1);

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2 zero() {
        return new Vector2(0.0, 0.0);
    }

    public Vector2 add(Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }

    public Vector2 minus(Vector2 other) {
        return new Vector2(x - other.x, y - other.y);
    }

    public Vector2 neg() {
        return new Vector2(-x, -y);
    }

    public Vector2 smul(double s) {
        return new Vector2(s * x, s * y);
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public double dot(Vector2 other) {
        return x * other.x + y * other.y;
    }

    public double signCrossDot(Vector2 other) {
        return Math.signum(cross(other)) * dot(other);
    }

    public double cross(Vector2 other) {
        return x * other.y - other.x * y;
    }

    public double angleTo(Vector2 other) {
        return Math.acos(this.dot(other) / (other.magnitude() * this.magnitude()));
    }

    public static Vector2 fromRad(double angle) {
        return new Vector2(Math.cos(angle), Math.sin(angle));
    }

    public double toRadians() {
        return Math.atan2(y, x);
    }

    @Override
    public String toString() {
        return String.format("{ %f, %f }", x, y);
    }

    public void normalize() {
        this.x /= magnitude();
        this.y /= magnitude();
    }
}

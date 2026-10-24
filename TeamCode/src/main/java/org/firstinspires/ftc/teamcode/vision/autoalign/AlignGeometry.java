package org.firstinspires.ftc.teamcode.vision.autoalign;

/** Pure geometry, independent of Android and the drivetrain. All distances are meters. */
public final class AlignGeometry {
    private AlignGeometry() { }

    public static final class Vector {
        public final double x, y, z;
        public Vector(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        public Vector plus(Vector b) { return new Vector(x + b.x, y + b.y, z + b.z); }
        public Vector minus(Vector b) { return new Vector(x - b.x, y - b.y, z - b.z); }
        public Vector times(double s) { return new Vector(x * s, y * s, z * s); }
        public double length() { return Math.sqrt(x * x + y * y + z * z); }
        public boolean finite() {
            return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
        }
    }

    /** Native Limelight pose rotations: degrees about X, Y, Z; Rz * Ry * Rx. */
    public static Vector rotate(Vector v, double rx, double ry, double rz) {
        double x = Math.toRadians(rx), y = Math.toRadians(ry), z = Math.toRadians(rz);
        double ax = v.x;
        double ay = v.y * Math.cos(x) - v.z * Math.sin(x);
        double az = v.y * Math.sin(x) + v.z * Math.cos(x);
        double bx = ax * Math.cos(y) + az * Math.sin(y);
        double bz = -ax * Math.sin(y) + az * Math.cos(y);
        return new Vector(bx * Math.cos(z) - ay * Math.sin(z),
                bx * Math.sin(z) + ay * Math.cos(z), bz);
    }

    /**
     * Camera vector (right, down, optical-forward) -> robot (right, forward, up).
     * Mount angles: roll clockwise looking through camera, pitch up, yaw right.
     * Rotation order is mount roll, then pitch, then yaw. No translation for axes.
     */
    public static Vector cameraToRobot(Vector c, double roll, double pitch, double yaw) {
        double r = Math.toRadians(roll), p = Math.toRadians(pitch), y = Math.toRadians(yaw);
        double right = c.x, forward = c.z, up = -c.y;
        double rr = right * Math.cos(r) + up * Math.sin(r);
        double ru = up * Math.cos(r) - right * Math.sin(r);
        double pf = forward * Math.cos(p) - ru * Math.sin(p);
        double pu = forward * Math.sin(p) + ru * Math.cos(p);
        return new Vector(rr * Math.cos(y) + pf * Math.sin(y),
                pf * Math.cos(y) - rr * Math.sin(y), pu);
    }
}

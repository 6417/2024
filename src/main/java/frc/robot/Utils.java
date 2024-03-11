package frc.robot;

/**
 * Utils
 */
public class Utils {

	public static void log(String message) {
		System.out.println(message);
	}

	public static void logerr(String message) {
		System.err.println(message);
	}

	public static void logerr(Exception e) {
		System.err.println(e);
		e.printStackTrace();
	}

	public static class Maths {
		public static double clamp(double value, double min, double max) {
			return Math.min(Math.max(value, min), max);
		}
	}
}

package frc.robot.subsystems;

import javax.swing.Timer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.fridowpi.module.Module;
import frc.fridowpi.utils.Vector3;

/**
 * LED Controls
 */
public class LED extends Module {

	private static AddressableLED led = new AddressableLED(2);

	private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(34);

	public static class RGB {
		public int r;
		public int g;
		public int b;

		public static RGB WHITE = new RGB(255, 255, 255);
		public static RGB BLACK = new RGB(0, 0, 0);
		public static RGB RED = new RGB(255, 0, 0);
		public static RGB GREEN = new RGB(0, 255, 0);
		public static RGB BLUE = new RGB(0, 0, 255);
		public static RGB YELLOW = new RGB(255, 255, 0);

		public RGB(int r, int g, int b) {
			this.r = r;
			this.g = g;
			this.b = b;
		}

		public static RGB from(Vector3 v) {
			if (v.magnitude() < 1) {
				return new RGB((int) v.x * 255, (int) v.y * 255, (int) v.z * 255);
			}
			return new RGB((int) v.x, (int) v.y, (int) v.z);
		}

		public Vector3 toVector3() {
			return new Vector3(r, g, b);
		}
	}

	@Override
	public void init() {
		led.setLength(buffer.getLength());
		led.setData(buffer);
		led.start();
		setColor(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue? RGB.BLUE: RGB.RED);
	}

	public void setColor(RGB color) {
		for (int i = 0; i < buffer.getLength(); i++) {
			buffer.setRGB(i, color.r, color.g, color.b);
		}
		led.setData(buffer);
	}

	@Override
	public void periodic() {
	}

	private Timer timerTmp = null;
	private Timer timerContinuous1 = null;
	private Timer timerContinuous2 = null;
	private int iContinuous = 0;
	private int iTmp = 0;

	public void setColorFluid(RGB color) {
		if (timerTmp != null) {
			timerTmp.stop();
		}
		timerTmp = new Timer(100, e -> {
			buffer.setRGB(iTmp++, color.r, color.g, color.b);
			led.setData(buffer);
			// if (iTmp > )S
		});
		timerTmp.start();
	}

	public void setColorContinuous(RGB color) {
		timerContinuous1 = new Timer(100, e -> {
			buffer.setRGB(iTmp, color.r, color.g, color.b);
			led.setData(buffer);
			iTmp %= buffer.getLength();
		});
		timerContinuous2 = new Timer(100, e -> {
			buffer.setRGB(iContinuous - 5, color.r, color.g, color.b);
			iContinuous ++;
			led.setData(buffer);
			iContinuous %= buffer.getLength();
		});
		timerContinuous2.setInitialDelay(50);

		timerContinuous2.setRepeats(true);
		timerContinuous1.setRepeats(true);

		timerContinuous2.start();
		timerContinuous1.start();
	}

	public void stopContinuous() {
		timerContinuous1.stop();
		timerContinuous2.stop();
	}

	boolean continuousOn = false;

	public void toggleContinuous(RGB color) {
		if (continuousOn) {
			stopContinuous();
		} else {
			setColorContinuous(color);
		}
		continuousOn = !continuousOn;
	}
}

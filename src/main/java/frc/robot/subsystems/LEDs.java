// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.Timer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.fridowpi.module.Module;

public class LEDs extends Module {
    private static LEDs instance;

    private AddressableLED leds;

    private AddressableLEDBuffer ledsBuffer;

    private Timer walkTroughTimer = new Timer(50, new ActionListener() {
        int i = 0;

        @Override
        public void actionPerformed(ActionEvent e) {
            ledsBuffer.setRGB(i-1, 0, 0, 255);
            ledsBuffer.setRGB(i, 255, 0, 0);
            i %= Constants.LEDs.bufferLength;
            setData();
        }
    });

    public static class RGB {
        public int red;
        public int green;
        public int blue;

        public RGB(int gray) {
            red = gray;
            green = gray;
            blue = gray;
        }

        public RGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    public LEDs() {
    }

    @Override
    public void init() {

        walkTroughTimer.start();

        leds = new AddressableLED(Constants.LEDs.pwmPort);
        leds.setLength(Constants.LEDs.bufferLength);

        ledsBuffer = new AddressableLEDBuffer(Constants.LEDs.bufferLength);

        leds.start();

        switchLEDs(redAlliance);
        setData();
    }

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }
        return instance;
    }

    private void setData() {
        leds.setData(ledsBuffer);
    }

    @Override
    public void periodic() {
    }

    private RGB normalLeds = new RGB(255, 0, 0);
    private RGB blueAlliance = new RGB(0, 0, 255);
    private RGB redAlliance = new RGB(255, 0, 0);
    private RGB turnedOff = new RGB(0);

    public void switchLEDs(RGB farbe) {
        System.err.println("sldkfj");
        for (int i = 0; i < Constants.LEDs.bufferLength; i++) {
            ledsBuffer.setRGB(i, farbe.red, farbe.green, farbe.blue);
        }
        setData();
    }
}
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

    private static AddressableLEDBuffer ledsBuffer;

    private Timer walkTroughTimer = new Timer(50, new ActionListener() {
        @Override
        public void actionPerformed(ActionEvent e) {
            regenbogen();
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

    private int hueL = 0;
    private int hueR = 0;
    private int round = 0;

    private void regenbogen() {
        for (int i = 0; i < Constants.LEDs.bufferLengthLeft; i++) {
            hueL += ((round + 1) * 360 / 15);
            if (hueL >= 360) {
                hueL -= 360;
            }
            ledsBuffer.setHSV(i, hueL, 1, 1);
        }

        for (int i = 15; i < Constants.LEDs.bufferLengthRight; i++) {
            hueR += ((round + 1) * 360 / 18);
            if (hueR >= 360) {
                hueR -= 360;
            }
            ledsBuffer.setHSV(i, hueR, 1, 1);
        }
        round+=1;
        setData();
    }

    // static void updateAnimation() {
    // // Löschen des vorherigen Schweifs und Erlöschen der LEDs, die nicht mehr zum
    // // Schweif gehören
    // for (int i = 0; i < 15; i++) {
    // if (i >= index && i < index + 4) {
    // int brightness = 255 + (index - i) * 50; // Dunkler werden für die 4 LEDs
    // hinter der Schweifspitze
    // ledsBuffer.setRGB(i, brightness, 0, 0); // Rote LEDs auf dem rechten Streifen
    // } else if (i >= index + 4) {
    // ledsBuffer.setRGB(i, 255, 0, 0); // Erlöschen der LEDs, die nicht mehr zum
    // Schweif gehören
    // }

    // if (i + 15 >= index + 15 && i + 15 < index + 4 + 15) {
    // int brightness = 255 + (index - i) * 50; // Dunkler werden für die 4 LEDs
    // hinter der Schweifspitze
    // ledsBuffer.setRGB(i + 15, brightness, 0, 0); // Rote LEDs auf dem linken
    // Streifen
    // } else if (i + 15 >= index + 4 + 15) {
    // ledsBuffer.setRGB(i + 15, 0, 0, 0); // Erlöschen der LEDs, die nicht mehr zum
    // Schweif gehören
    // }
    // }
    // // Aktualisieren des Index für den nächsten Schweif
    // index = (index + 1) % 15;
    // }
}
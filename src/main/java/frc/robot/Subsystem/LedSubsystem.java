package frc.robot.Subsystem;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.lib.math.MathUtils;
//import frc.robot.Constants.LightsConstants;

public class LedSubsystem extends SubsystemBase {
    private static final CANdle candle = new CANdle(20);
    

    // Team colors
    public static final Color red = new Color(255, 0, 0);
    public static final Color black = new Color(0, 0, 0);

    // Game piece colors
    public static final Color yellow = new Color(255, 30, 0);
    

    // Indicator colors
    public static final Color white = new Color(255, 230, 220);
    public static final Color green = new Color(56, 209, 0);
    public static final Color blue = new Color(8, 32, 255);
    

    public LedSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.v5Enabled = false;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(candleConfiguration, 100);
       //LEDSegment.StartStrip.setRainbowAnimation(.4);
        //LEDSegment.EndStrip.setRainbowAnimation(.4);
        
        setDefaultCommand(defaultCommand());
        
        
    }
    
    public  void setColorred(){
            LEDSegment.Strip.setColor(red);
        }
    public   void setcolorblue(){
            LEDSegment.Strip.setColor(blue);
    }
    public   void SetGamepiece(){
        LEDSegment.Strip.setColor(yellow);
    }
    public   void setAnimation(){
    
        LEDSegment.Strip.setFadeAnimation(red, .8);
        //LEDSegment.Strip.setFadeAnimation(red, .8);
    }
    public void setBrightness(double percent) {
        candle.configBrightnessScalar(percent, 100);
    }
    public void clearAnimation(){
            LEDSegment.Strip.clearAnimation();
    }

    public Command defaultCommand() {
        return runOnce(() -> {
            // LEDSegment.BatteryIndicator.fullClear();
            // LEDSegment.PressureIndicator.fullClear();
            // LEDSegment.MastEncoderIndicator.fullClear();
            // LEDSegment.BoomEncoderIndicator.fullClear();
            // LEDSegment.WristEncoderIndicator.fullClear();
            // LEDSegment.StartStrip.fullClear();
            
        });
    }

    public Command clearSegmentCommand(LEDSegment segment) {
        return runOnce(() -> {
            segment.clearAnimation();
            
        });
    }

    public static enum LEDSegment {
        BatteryIndicator(0, 2, 0),
        PressureIndicator(2, 2, 1),
        MastEncoderIndicator(4, 1, -1),
        BoomEncoderIndicator(5, 1, -1),
        WristEncoderIndicator(6, 1, -1),
        DriverStationIndicator(7, 1, -1),
        //EndStrip(68, 60, 1),
        //StartStrip(0, 68, 2);
        Strip(0, 136, 1);
        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            candle.animate(animation, animationSlot);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            candle.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(black);
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward, startIndex));
        }

        public void setBackFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Backward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 40, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

       
        
    }
}


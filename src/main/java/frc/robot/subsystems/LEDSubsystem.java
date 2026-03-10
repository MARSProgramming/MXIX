package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle = new CANdle(24);

    public enum LEDSegment {
        CANDLE,
        LEFT_BAR,
        RIGHT_BAR,
        BOTH_BARS,
        ALL
    }

    // LED ranges
    private static final int CANDLE_START = 0;
    private static final int CANDLE_COUNT = 8;

    private static final int LEFT_START = 8;
    private static final int LEFT_COUNT = 31;

    private static final int RIGHT_START = 39;
    private static final int RIGHT_COUNT = 31;

    private static final int BAR_COUNT = 62;

    public LEDSubsystem() {
        candle.getConfigurator().apply(new CANdleConfiguration());
    }

    private int getStart(LEDSegment segment) {
        switch (segment) {
            case CANDLE:
                return CANDLE_START;
            case LEFT_BAR:
                return LEFT_START;
            case RIGHT_BAR:
                return RIGHT_START;
            case BOTH_BARS:
                return LEFT_START;
            case ALL:
                return 0;
        }
        return 0;
    }

    private int getCount(LEDSegment segment) {
        switch (segment) {
            case CANDLE:
                return CANDLE_COUNT;
            case LEFT_BAR:
                return LEFT_COUNT;
            case RIGHT_BAR:
                return RIGHT_COUNT;
            case BOTH_BARS:
                return BAR_COUNT;
            case ALL:
                return 70;
        }
        return 0;
    }

    private void setRange(Color color, LEDSegment segment) {
        int r = (int) (color.red * 255);
        int g = (int) (color.green * 255);
        int b = (int) (color.blue * 255);

        candle.setControl(
                new SolidColor(getStart(segment), getCount(segment))
                        .withColor(new RGBWColor(r, g, b)));
    }

    private RGBWColor getRGBWColor(Color color) {
        int r = (int) (color.red * 255);
        int g = (int) (color.green * 255);
        int b = (int) (color.blue * 255);

        return new RGBWColor(r,g,b);

    }

    public void setColor(Color color, LEDSegment segment) {
        setRange(color, segment);
    }

    public void off() {
        setColor(Color.kBlack, LEDSegment.ALL);
    }

    // -------------------------
    // Animations
    // -------------------------

    public void rainbow(LEDSegment segment) {
        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        RainbowAnimation rainbow = new RainbowAnimation(start, end)
                .withBrightness(1.0)
                .withFrameRate(50)
                .withDirection(AnimationDirectionValue.Forward);
        

        candle.setControl(rainbow);
    }

    public void strobe(LEDSegment segment) {
        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        StrobeAnimation strobeAnimation = new StrobeAnimation(start, end)
        .withColor(RGBWColor.fromHex(Color.kGreen.toHexString()).get())
        .withFrameRate(50);

        candle.setControl(strobeAnimation);
    }

    public void rolling(Color color, LEDSegment segment) {
        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        int r = (int) (color.red * 255);
        int g = (int) (color.green * 255);
        int b = (int) (color.blue * 255);

        ColorFlowAnimation flow = new ColorFlowAnimation(start, end)
                .withColor(new RGBWColor(r, g, b))
                .withFrameRate(50)
                .withDirection(AnimationDirectionValue.Forward);

        candle.setControl(flow);
    }

    public void scanner(Color color, LEDSegment segment) {
        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        int r = (int) (color.red * 255);
        int g = (int) (color.green * 255);
        int b = (int) (color.blue * 255);

        LarsonAnimation larson = new LarsonAnimation(start, end)
    
                .withColor(new RGBWColor(r, g, b))
                .withFrameRate(10)
                .withSize(15)
                .withBounceMode(LarsonBounceValue.Front);

        candle.setControl(larson);

    }

    public void rgbFade(LEDSegment segment) {
             int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        RgbFadeAnimation fadeAnimation = new RgbFadeAnimation(start, end)
        .withFrameRate(50);

        candle.setControl(fadeAnimation);
   
    }

    public void fadeColor(LEDSegment segment, Color color) {
             int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        SingleFadeAnimation singleFade = new SingleFadeAnimation(start, end)
        .withColor(getRGBWColor(color))
        .withFrameRate(50);

        candle.setControl(singleFade);
    }
}
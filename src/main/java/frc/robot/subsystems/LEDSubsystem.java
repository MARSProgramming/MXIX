package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;

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

    private static final int CANDLE_START = 0;
    private static final int CANDLE_COUNT = 8;
    private static final int LEFT_START   = 8;
    private static final int LEFT_COUNT   = 31;
    private static final int RIGHT_START  = 39;
    private static final int RIGHT_COUNT  = 31;
    private static final int BAR_COUNT    = 62;

    public LEDSubsystem() {
        CANdleConfiguration configs = new CANdleConfiguration();
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(configs);
        rainbow(LEDSegment.BOTH_BARS);
    }

    // -------------------------
    // Segment Helpers
    // -------------------------

    private int getStart(LEDSegment segment) {
        switch (segment) {
            case CANDLE: return CANDLE_START;
            case LEFT_BAR: return LEFT_START;
            case RIGHT_BAR: return RIGHT_START;
            case BOTH_BARS: return LEFT_START;
            case ALL: return 0;
            default: return 0;
        }
    }

    private int getCount(LEDSegment segment) {
        switch (segment) {
            case CANDLE: return CANDLE_COUNT;
            case LEFT_BAR: return LEFT_COUNT;
            case RIGHT_BAR: return RIGHT_COUNT;
            case BOTH_BARS: return BAR_COUNT;
            case ALL: return 70;
            default: return 0;
        }
    }

    private RGBWColor toRGBW(Color color) {
        return new RGBWColor(
            (int)(color.red * 255),
            (int)(color.green * 255),
            (int)(color.blue * 255)
        );
    }

    private void clearAnimation() {
        candle.setControl(new EmptyAnimation(0));
    }

    // -------------------------
    // LED Effects
    // -------------------------

    public void setColor(Color color, LEDSegment segment) {
        clearAnimation();

        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        candle.setControl(
            new SolidColor(start, end)
                .withColor(toRGBW(color))
        );
    }

    public void off() {
        setColor(Color.kBlack, LEDSegment.ALL);
    }

    public void rainbow(LEDSegment segment) {
        clearAnimation();

        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        candle.setControl(
            new RainbowAnimation(start, end)
                .withBrightness(1.0)
                .withFrameRate(50)
                .withDirection(AnimationDirectionValue.Forward)
                .withSlot(0)
        );
    }

    public void strobe(Color color, LEDSegment segment) {
        clearAnimation();

        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        candle.setControl(
            new StrobeAnimation(start, end)
                .withColor(RGBWColor.fromHex(Color.kGreen.toHexString()).get())
                .withFrameRate(25)
                .withSlot(0)
        );
    }

    public void rolling(Color color, LEDSegment segment) {
        clearAnimation();

        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        candle.setControl(
            new ColorFlowAnimation(start, end)
                .withColor(toRGBW(color))
                .withFrameRate(50)
                .withDirection(AnimationDirectionValue.Forward)
                .withSlot(0)
        );
    }

    public void scanner(Color color, LEDSegment segment) {
        clearAnimation();

        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        candle.setControl(
            new LarsonAnimation(start, end)
                .withColor(toRGBW(color))
                .withFrameRate(10)
                .withSize(15)
                .withBounceMode(LarsonBounceValue.Front)
                .withSlot(0)
        );
    }

    public void rgbFade(LEDSegment segment) {
        clearAnimation();

        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        candle.setControl(
            new RgbFadeAnimation(start, end)
                .withFrameRate(50)
                .withSlot(0)
        );
    }

    public void fadeColor(Color color, LEDSegment segment) {
        clearAnimation();

        int start = getStart(segment);
        int end = start + getCount(segment) - 1;

        candle.setControl(
            new SingleFadeAnimation(start, end)
                .withColor(toRGBW(color))
                .withFrameRate(20)
                .withSlot(0)
        );
    }
}
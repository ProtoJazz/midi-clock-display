use embedded_graphics::{
    image::ImageRaw,
    mono_font::{mapping::StrGlyphMapping, DecorationDimensions, MonoFont, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder},
};
use esp_idf_svc::hal::uart::{config::Config, UartDriver};
use esp_idf_svc::hal::units::FromValueType;
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::hal::{
    gpio,
    i2c::{I2cConfig, I2cDriver},
};
use esp_idf_svc::hal::{
    gpio::Gpio43, gpio::Gpio44, gpio::Gpio5, gpio::Gpio6, i2c::I2C0, peripherals::Peripherals,
    uart::UART1,
};
use esp_idf_svc::sys::esp_timer_get_time;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

const SEVENT_SEGMENT_FONT: MonoFont = MonoFont {
    image: ImageRaw::new(include_bytes!("./seven-segment-font.raw"), 224),
    glyph_mapping: &StrGlyphMapping::new("0123456789", 0),
    character_size: Size::new(22, 40),
    character_spacing: 4,
    baseline: 7,
    underline: DecorationDimensions::default_underline(40),
    strikethrough: DecorationDimensions::default_strikethrough(40),
};
const UART_BUFFER_SIZE: usize = 1024;
fn main() -> anyhow::Result<()> {
    let peripherals = Peripherals::take().unwrap();

    let uart1 = peripherals.uart1;
    let gpio43 = peripherals.pins.gpio43;
    let gpio44 = peripherals.pins.gpio44;
    let uart = setup_midi_uart(uart1, gpio43, gpio44);

    let i2c0 = peripherals.i2c0;
    let gpio5 = peripherals.pins.gpio5;
    let gpio6 = peripherals.pins.gpio6;
    let mut display = setup_i2c_display(i2c0, gpio5, gpio6)?;

    let character_style = MonoTextStyle::new(&SEVENT_SEGMENT_FONT, BinaryColor::On);
    let text_style = TextStyleBuilder::new()
        .baseline(Baseline::Top)
        .alignment(Alignment::Center)
        .build();

    const MIDI_CLOCK: u8 = 0xF8;
    const MIDI_START: u8 = 0xF2;
    const EMPTY_BUFFER: u8 = 0x00;
    const CLOCKS_PER_BEAT: u32 = 24;

    let mut beat: u32 = 1;
    let mut bpm = 0.0 as f64;
    let mut clock_count = 0;
    let mut last_time = unsafe { esp_timer_get_time() };

    update_display(
        &mut display,
        beat,
        bpm.round() as u32,
        character_style,
        text_style,
    );
    loop {
        let mut buffer = [0u8; 1];
        let timeout = 400;

        if let Ok(_) = uart.read(&mut buffer, timeout) {
            let midi_byte = buffer[0];
            match midi_byte {
                MIDI_CLOCK => {
                    clock_count += 1;
                    if clock_count == CLOCKS_PER_BEAT {
                        let now = unsafe { esp_timer_get_time() };
                        let time_per_beat = now - last_time;
                        last_time = now;
                        bpm = (60.0 * 1_000_000.0) / time_per_beat as f64;
                        beat += 1;
                        if beat > 4 {
                            beat = 1;
                        }
                        clock_count = 0;

                        update_display(
                            &mut display,
                            beat,
                            bpm.round() as u32,
                            character_style,
                            text_style,
                        );
                    }
                }
                MIDI_START => {
                    beat = 1;
                    update_display(
                        &mut display,
                        beat,
                        bpm.round() as u32,
                        character_style,
                        text_style,
                    );
                }
                EMPTY_BUFFER => continue,
                _ => continue,
            }
        }
    }
}

fn setup_i2c_display(
    i2c0: I2C0,
    gpio5: Gpio5,
    gpio6: Gpio6,
) -> Result<
    Ssd1306<
        I2CInterface<I2cDriver<'static>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    anyhow::Error,
> {
    let i2c_config = I2cConfig::new().baudrate(100_u32.kHz().into());
    let i2c_driver = I2cDriver::new(
        i2c0,
        gpio5, // SDA
        gpio6, // SCL
        &i2c_config,
    )?;
    let i2c_interface = I2CDisplayInterface::new(i2c_driver);
    let mut display = Ssd1306::new(i2c_interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    match display.init() {
        Ok(_) => println!("Display initialized"),
        Err(e) => println!("Failed to initialize display: {:?}", e),
    }
    Ok(display)
}

fn setup_midi_uart(uart1: UART1, gpio43: Gpio43, gpio44: Gpio44) -> UartDriver<'static> {
    let uart_config = Config::default()
        .baudrate(Hertz::from(31_250))
        .rx_fifo_size(UART_BUFFER_SIZE);

    let uart = UartDriver::new(
        uart1,
        gpio43,
        gpio44,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &uart_config,
    )
    .unwrap();
    uart
}
fn update_display(
    display: &mut Ssd1306<
        I2CInterface<I2cDriver<'_>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    count: u32,
    bpm: u32,
    character_style: MonoTextStyle<BinaryColor>,
    text_style: TextStyle,
) {
    Rectangle::new(Point::zero(), Size::new(128, 64))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(0, 20), Size::new(15, 15))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(display)
        .unwrap();
    Text::with_text_style(
        &bpm.to_string(),
        Point::new(45, 20),
        character_style,
        text_style,
    )
    .draw(display)
    .unwrap();
    Text::with_text_style(
        &count.to_string(),
        Point::new(110, 20),
        character_style,
        text_style,
    )
    .draw(display)
    .unwrap();
    display.flush().unwrap();
}

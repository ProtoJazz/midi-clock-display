use std::{
    ops::ControlFlow,
    sync::atomic::{AtomicBool, Ordering},
};

use embedded_graphics::{
    image::ImageRaw,
    mono_font::{
        ascii::FONT_6X12, mapping::StrGlyphMapping, DecorationDimensions, MonoFont, MonoTextStyle,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder},
};
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
use esp_idf_svc::hal::{
    gpio::{InterruptType, PinDriver, Pull},
    uart::{config::Config, UartDriver},
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
static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);
const MIDI_CLOCK: u8 = 0xF8;
const MIDI_START: u8 = 0xF2;
const EMPTY_BUFFER: u8 = 0x00;
const CLOCKS_PER_BEAT: u32 = 24;
const CONTROL_CHANNEL: u32 = 10;
const CONTROL_BYTE: u8 = (0xB0 + (CONTROL_CHANNEL - 1)) as u8;
const MIDI_PAUSE: u8 = 0xFC;
const MIDI_LOGGING: bool = false;
fn gpio_isr_handler() {
    // Set the BUTTON_PRESSED flag to true
    BUTTON_PRESSED.store(true, Ordering::SeqCst);
}

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

    // Configures the button
    let mut button = PinDriver::input(peripherals.pins.gpio2)?;
    button.set_pull(Pull::Up)?;
    button.set_interrupt_type(InterruptType::NegEdge)?;

    unsafe {
        button.subscribe(gpio_isr_handler)?;
    }
    button.enable_interrupt()?;
    let character_style = MonoTextStyle::new(&SEVENT_SEGMENT_FONT, BinaryColor::On);
    let text_style = TextStyleBuilder::new()
        .baseline(Baseline::Top)
        .alignment(Alignment::Center)
        .build();

    let small_character_style = MonoTextStyle::new(&FONT_6X12, BinaryColor::On);

    let mut beat: u32 = 1;
    let mut bpm = 0.0 as f64;
    let mut clock_count = 0;
    let mut last_time = unsafe { esp_timer_get_time() };

    let mut beats_per_bar: u32 = 4;

    update_display(
        &mut display,
        beat,
        bpm.round() as u32,
        character_style,
        text_style,
    );
    update_settings_display(
        &mut display,
        beats_per_bar,
        small_character_style,
        text_style,
    );
    display.flush().unwrap();
    loop {
        let mut buffer = [0u8; 1];
        let timeout = 400;
        if BUTTON_PRESSED.load(Ordering::SeqCst) {
            // Handle the button press event (change your value here)
            println!("Button pressed! Changing value...");
            beats_per_bar = match beats_per_bar {
                3 => 4,
                4 => 5,
                5 => 7,
                7 => 4,
                _ => 4,
            };
            // Reset the flag
            BUTTON_PRESSED.store(false, Ordering::SeqCst);
            update_settings_display(
                &mut display,
                beats_per_bar,
                small_character_style,
                text_style,
            );
            button.enable_interrupt()?;
        }
        if uart.read(&mut buffer, timeout).is_ok() {
            let status_byte = buffer[0];

            let num_data_bytes = get_data_byte_count(status_byte);

            // Read the required number of data bytes
            let mut data_bytes = vec![0u8; num_data_bytes];
            for i in 0..num_data_bytes {
                if uart.read(&mut buffer, timeout).is_ok() {
                    data_bytes[i] = buffer[0];
                } else {
                    continue;
                }
            }
            if let ControlFlow::Break(_) = handle_midi(
                MidiMessage {
                    status_byte,
                    data_bytes,
                },
                &mut clock_count,
                &mut last_time,
                &mut bpm,
                &mut beat,
                beats_per_bar,
                &mut display,
                character_style,
                text_style,
            ) {
                continue;
            }
        }
    }
}

fn handle_midi(
    midi_message: MidiMessage,
    clock_count: &mut u32,
    last_time: &mut i64,
    bpm: &mut f64,
    beat: &mut u32,
    beats_per_bar: u32,
    display: &mut Ssd1306<
        I2CInterface<I2cDriver<'_>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    character_style: MonoTextStyle<'_, BinaryColor>,
    text_style: TextStyle,
) -> ControlFlow<()> {
    let channel = get_midi_channel(midi_message.status_byte);
    if MIDI_LOGGING {
        println!(
            "MIDI message - Status Byte: {:#02X}, Channel: {}, Data Bytes: {:?}",
            midi_message.status_byte, // The status byte as a hexadecimal value
            channel + 1,              // The channel (adding 1 since MIDI channels are 1-16)
            midi_message.data_bytes   // The data bytes as a vector, printed using Debug format
        );
    }
    match midi_message.status_byte {
        MIDI_CLOCK => {
            *clock_count += 1;
            if *clock_count == CLOCKS_PER_BEAT {
                let now = unsafe { esp_timer_get_time() };
                let time_per_beat = now - *last_time;
                *last_time = now;
                *bpm = (60.0 * 1_000_000.0) / time_per_beat as f64;
                *beat += 1;
                if *beat > beats_per_bar {
                    *beat = 1;
                }
                *clock_count = 0;

                update_display(
                    display,
                    *beat,
                    bpm.round() as u32,
                    character_style,
                    text_style,
                );
                display.flush().unwrap();
            }
        }
        MIDI_START | MIDI_PAUSE => {
            *beat = 1;
            *clock_count = 0;
            let now = unsafe { esp_timer_get_time() };
            *last_time = now;
            update_display(
                display,
                *beat,
                bpm.round() as u32,
                character_style,
                text_style,
            );
            display.flush().unwrap();
        }
        CONTROL_BYTE => {
            println!("Stuff goes here to control the clock by midi");
        }
        EMPTY_BUFFER => return ControlFlow::Break(()),
        _ => return ControlFlow::Break(()),
    }
    ControlFlow::Continue(())
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

fn update_settings_display(
    display: &mut Ssd1306<
        I2CInterface<I2cDriver<'_>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    beats: u32,
    character_style: MonoTextStyle<BinaryColor>,
    text_style: TextStyle,
) {
    Rectangle::new(Point::zero(), Size::new(12, 64))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display)
        .unwrap();
    Text::with_text_style(
        &beats.to_string(),
        Point::new(9, 20),
        character_style,
        text_style,
    )
    .draw(display)
    .unwrap();
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
    Rectangle::new(Point::new(12, 0), Size::new(118, 64))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
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
}

fn get_midi_channel(status_byte: u8) -> u8 {
    status_byte & 0x0F // Extract the lower nibble
}

// Function to determine the number of data bytes based on the status byte
fn get_data_byte_count(status_byte: u8) -> usize {
    match status_byte & 0xF0 {
        0x80 => 2, // Note Off (status byte + 2 data bytes)
        0x90 => 2, // Note On (status byte + 2 data bytes)
        0xA0 => 2, // Polyphonic Key Pressure (status byte + 2 data bytes)
        0xB0 => 2, // Control Change (status byte + 2 data bytes)
        0xC0 => 1, // Program Change (status byte + 1 data byte)
        0xD0 => 1, // Channel Pressure (status byte + 1 data byte)
        0xE0 => 2, // Pitch Bend Change (status byte + 2 data bytes)
        _ => 0,    // System messages like Timing Clock (no data bytes)
    }
}

// Struct to represent a complete MIDI message
struct MidiMessage {
    status_byte: u8,
    data_bytes: Vec<u8>,
}

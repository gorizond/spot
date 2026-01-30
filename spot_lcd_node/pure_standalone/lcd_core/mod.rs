use anyhow::Result;
use crate::config::LcdConfig;
use std::thread;
use std::time::Duration;

#[cfg(target_os = "linux")]
use rppal::gpio::{Gpio, OutputPin};

// Define a mock OutputPin for non-Linux platforms
#[cfg(not(target_os = "linux"))]
pub struct MockOutputPin;

#[cfg(not(target_os = "linux"))]
impl MockOutputPin {
    pub fn set_high(&mut self) {}
    pub fn set_low(&mut self) {}
}

#[cfg(target_os = "linux")]
pub struct LcdDisplay {
    rs: OutputPin,
    en: OutputPin,
    data_pins: Vec<OutputPin>,
    config: LcdConfig,
}

#[cfg(not(target_os = "linux"))]
pub struct LcdDisplay {
    config: LcdConfig,
}

#[cfg(target_os = "linux")]
impl LcdDisplay {
    pub fn new(gpio: &Gpio, config: LcdConfig) -> Result<Self> {
        let rs = gpio.get(config.pin_rs)?.into_output();
        let en = gpio.get(config.pin_en)?.into_output();
        let mut data_pins = Vec::new();
        
        for pin_num in config.pins_data.iter() {
            data_pins.push(gpio.get(*pin_num)?.into_output());
        }

        let mut lcd = LcdDisplay { rs, en, data_pins, config };
        lcd.init()?;
        Ok(lcd)
    }
}

#[cfg(not(target_os = "linux"))]
impl LcdDisplay {
    pub fn new(config: LcdConfig) -> Result<Self> {
        println!("LCD: GPIO not available on this platform, running in simulation mode");
        let mut lcd = LcdDisplay { config };
        lcd.init()?;
        Ok(lcd)
    }
}

#[cfg(target_os = "linux")]
impl LcdDisplay {
    fn init(&mut self) -> Result<()> {
        thread::sleep(Duration::from_millis(15)); // Wait for LCD to power up
        
        // Initialize in 4-bit mode
        self.send_command(0x33)?; // Function set: 8-bit mode
        thread::sleep(Duration::from_millis(5));
        
        self.send_command(0x32)?; // Function set: 4-bit mode
        thread::sleep(Duration::from_millis(1));
        
        self.send_command(0x28)?; // Function set: 4-bit, 2-line, 5x8 dots
        self.send_command(0x0C)?; // Display on, cursor off, blink off
        self.send_command(0x06)?; // Entry mode set: increment cursor
        self.send_command(0x01)?; // Clear display
        
        Ok(())
    }

    fn write_nibble(&mut self, nibble: u8) {
        for (i, pin) in self.data_pins.iter_mut().enumerate() {
            if (nibble >> i) & 1 != 0 {
                pin.set_high();
            } else {
                pin.set_low();
            }
        }
        self.en.set_high();
        thread::sleep(Duration::from_micros(1));
        self.en.set_low();
        thread::sleep(Duration::from_micros(50));
    }

    fn write_byte(&mut self, byte: u8, rs_state: bool) {
        if rs_state {
            self.rs.set_high();
        } else {
            self.rs.set_low();
        }
        
        self.write_nibble((byte >> 4) & 0x0F);
        self.write_nibble(byte & 0x0F);
    }

    fn send_command(&mut self, cmd: u8) -> Result<()> {
        self.write_byte(cmd, false);
        thread::sleep(Duration::from_micros(50));
        Ok(())
    }

    pub fn write_char(&mut self, ch: char) {
        self.write_byte(ch as u8, true);
    }

    pub fn write_string(&mut self, s: &str) {
        for ch in s.chars() {
            self.write_char(ch);
        }
    }

    pub fn clear(&mut self) -> Result<()> {
        self.send_command(0x01)
    }

    pub fn set_cursor(&mut self, row: u8, col: u8) -> Result<()> {
        let addr = if row == 0 { 0x00 } else { 0x40 };
        self.send_command(0x80 | (addr + col))?;
        Ok(())
    }

    pub fn write_line(&mut self, row: u8, text: &str) -> Result<()> {
        self.set_cursor(row, 0)?;
        // Обрезаем текст до размера дисплея
        let display_text = if text.len() > self.config.cols {
            &text[..self.config.cols]
        } else {
            text
        };
        self.write_string(display_text);
        Ok(())
    }
}

#[cfg(not(target_os = "linux"))]
impl LcdDisplay {
    fn init(&mut self) -> Result<()> {
        println!("LCD: Initializing in simulation mode");
        // Simulate initialization delay
        thread::sleep(Duration::from_millis(15));
        Ok(())
    }

    fn send_command(&mut self, cmd: u8) -> Result<()> {
        println!("LCD: Sending command 0x{:X}", cmd);
        thread::sleep(Duration::from_micros(50));
        Ok(())
    }

    pub fn write_char(&mut self, ch: char) {
        println!("LCD: Writing character '{}'", ch);
    }

    pub fn write_string(&mut self, s: &str) {
        println!("LCD: Writing string '{}'", s);
    }

    pub fn clear(&mut self) -> Result<()> {
        println!("LCD: Clearing display");
        Ok(())
    }

    pub fn set_cursor(&mut self, row: u8, col: u8) -> Result<()> {
        println!("LCD: Setting cursor to row {}, col {}", row, col);
        Ok(())
    }

    pub fn write_line(&mut self, row: u8, text: &str) -> Result<()> {
        // Обрезаем текст до размера дисплея
        let display_text = if text.len() > self.config.cols {
            &text[..self.config.cols]
        } else {
            text
        };
        println!("LCD: Writing line {}: '{}'", row, display_text);
        Ok(())
    }
}
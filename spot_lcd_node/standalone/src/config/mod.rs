use std::env;

#[derive(Debug, Clone)]
pub struct Config {
    pub lcd: LcdConfig,
    pub temp_monitor: TempMonitorConfig,
    pub uptime_monitor: UptimeMonitorConfig,
    pub ros_publisher: RosPublisherConfig,
}

#[derive(Debug, Clone)]
pub struct LcdConfig {
    pub pin_rs: u8,
    pub pin_en: u8,
    pub pins_data: Vec<u8>,
    pub cols: usize,
    pub rows: usize,
}

#[derive(Debug, Clone)]
pub struct TempMonitorConfig {
    pub interval: u64,
}

#[derive(Debug, Clone)]
pub struct UptimeMonitorConfig {
    pub interval: u64,
}

#[derive(Debug, Clone)]
pub struct RosPublisherConfig {
    pub temp_topic: String,
    pub uptime_topic: String,
}

impl Config {
    pub fn from_env() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Config {
            lcd: LcdConfig {
                pin_rs: env::var("LCD_PIN_RS").unwrap_or("25".to_string()).parse().unwrap_or(25),
                pin_en: env::var("LCD_PIN_EN").unwrap_or("24".to_string()).parse().unwrap_or(24),
                pins_data: env::var("LCD_PINS_DATA")
                    .unwrap_or("23,17,18,22".to_string())
                    .split(',')
                    .map(|s| s.trim().parse().unwrap_or(0))
                    .collect(),
                cols: env::var("LCD_COLS").unwrap_or("16".to_string()).parse().unwrap_or(16),
                rows: env::var("LCD_ROWS").unwrap_or("2".to_string()).parse().unwrap_or(2),
            },
            temp_monitor: TempMonitorConfig {
                interval: env::var("TEMP_INTERVAL").unwrap_or("5".to_string()).parse().unwrap_or(5),
            },
            uptime_monitor: UptimeMonitorConfig {
                interval: env::var("UPTIME_INTERVAL").unwrap_or("10".to_string()).parse().unwrap_or(10),
            },
            ros_publisher: RosPublisherConfig {
                temp_topic: env::var("TEMP_TOPIC").unwrap_or("lcd_temperature".to_string()),
                uptime_topic: env::var("UPTIME_TOPIC").unwrap_or("lcd_uptime".to_string()),
            },
        })
    }
}
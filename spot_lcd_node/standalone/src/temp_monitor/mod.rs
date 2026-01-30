use crate::config::TempMonitorConfig;
use std::fs;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

pub struct TempMonitor {
    callback: Arc<dyn Fn(&str) + Send + Sync>,
    config: TempMonitorConfig,
}

impl TempMonitor {
    pub fn new(callback: Arc<dyn Fn(&str) + Send + Sync>, config: TempMonitorConfig) -> Self {
        TempMonitor { callback, config }
    }

    pub fn start_monitoring(&self) {
        let callback = Arc::clone(&self.callback);
        let interval = self.config.interval;
        thread::spawn(move || {
            loop {
                match read_cpu_temperature() {
                    Ok(temp) => callback(&temp),
                    Err(_) => callback("N/A"),
                }
                thread::sleep(Duration::from_secs(interval));
            }
        });
    }
}

fn read_cpu_temperature() -> Result<String, std::io::Error> {
    // Try different paths for CPU temperature
    let paths = [
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/hwmon/hwmon0/temp1_input",
    ];

    for path in &paths {
        if let Ok(content) = fs::read_to_string(path) {
            let temp_mC: f32 = content.trim().parse().unwrap_or(0.0) / 1000.0;
            return Ok(format!("{:.1}°C", temp_mC));
        }
    }

    // If no temperature file found, return error
    Err(std::io::Error::new(
        std::io::ErrorKind::NotFound,
        "Temperature sensor not found",
    ))
}
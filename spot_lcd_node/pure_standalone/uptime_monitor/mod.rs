use crate::config::UptimeMonitorConfig;
use std::fs;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

pub struct UptimeMonitor {
    callback: Arc<dyn Fn(&str) + Send + Sync>,
    config: UptimeMonitorConfig,
}

impl UptimeMonitor {
    pub fn new(callback: Arc<dyn Fn(&str) + Send + Sync>, config: UptimeMonitorConfig) -> Self {
        UptimeMonitor { callback, config }
    }

    pub fn start_monitoring(&self) {
        let callback = Arc::clone(&self.callback);
        let interval = self.config.interval;
        thread::spawn(move || {
            loop {
                match read_uptime() {
                    Ok(uptime) => callback(&uptime),
                    Err(_) => callback("N/A"),
                }
                thread::sleep(Duration::from_secs(interval));
            }
        });
    }
}

fn read_uptime() -> Result<String, std::io::Error> {
    let content = fs::read_to_string("/proc/uptime")?;
    let uptime_seconds: f32 = content.split_whitespace().next()
        .unwrap_or("0")
        .parse()
        .unwrap_or(0.0);

    let days = (uptime_seconds / (24.0 * 3600.0)).floor();
    let hours = ((uptime_seconds % (24.0 * 3600.0)) / 3600.0).floor();
    let minutes = ((uptime_seconds % 3600.0) / 60.0).floor();

    Ok(format!("{}d {}h {}m", days as u32, hours as u32, minutes as u32))
}
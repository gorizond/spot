use crate::lcd_core::LcdDisplay;
use anyhow::Result;
use crate::config::LcdConfig;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

pub struct DisplayService {
    #[cfg(target_os = "linux")]
    lcd: Option<Arc<Mutex<LcdDisplay>>>,
    #[cfg(not(target_os = "linux"))]
    lcd: Option<Arc<Mutex<LcdDisplay>>>,
    data: Arc<Mutex<HashMap<String, String>>>,
    config: LcdConfig,
}

impl DisplayService {
    #[cfg(target_os = "linux")]
    pub fn new(lcd: Arc<Mutex<LcdDisplay>>, config: LcdConfig) -> Self {
        DisplayService {
            lcd: Some(lcd),
            data: Arc::new(Mutex::new(HashMap::new())),
            config,
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn new(config: LcdConfig) -> Self {
        DisplayService {
            lcd: None,
            data: Arc::new(Mutex::new(HashMap::new())),
            config,
        }
    }

    // Метод для получения ссылки на хранилище данных (для использования в коллбэках)
    pub fn get_lcd_reference(&self) -> Arc<Mutex<HashMap<String, String>>> {
        Arc::clone(&self.data)
    }

    pub fn update_data(&self, key: &str, value: &str) {
        let mut data = self.data.lock().unwrap();
        data.insert(key.to_string(), value.to_string());
    }

    pub fn start_display_loop(&self) {
        let data = Arc::clone(&self.data);
        let config = self.config.clone();

        #[cfg(target_os = "linux")]
        {
            if let Some(ref lcd) = self.lcd {
                let lcd = Arc::clone(lcd);
                thread::spawn(move || {
                    loop {
                        let data_copy = {
                            let data = data.lock().unwrap();
                            data.clone()
                        };

                        let mut lcd_lock = lcd.lock().unwrap();
                        
                        // Clear display
                        let _ = lcd_lock.clear();

                        // Display temperature in top-left quadrant
                        let temp = data_copy.get("temperature").unwrap_or(&"N/A".to_string());
                        let _ = lcd_lock.write_line(0, &format!("Temp: {}", temp));

                        // Display uptime in bottom row
                        let uptime = data_copy.get("uptime").unwrap_or(&"N/A".to_string());
                        let _ = lcd_lock.write_line(1, &format!("Up: {}", uptime));

                        drop(lcd_lock);
                        thread::sleep(Duration::from_secs(1));
                    }
                });
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            thread::spawn(move || {
                loop {
                    let data_copy = {
                        let data = data.lock().unwrap();
                        data.clone()
                    };

                    // Simulate display on non-Linux platforms
                    let temp = data_copy.get("temperature").unwrap_or(&"N/A".to_string()).clone();
                    let uptime = data_copy.get("uptime").unwrap_or(&"N/A".to_string()).clone();
                    
                    println!("LCD Simulation - Temp: {}, Uptime: {}", temp, uptime);

                    thread::sleep(Duration::from_secs(1));
                }
            });
        }
    }
}

fn truncate_str(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        s.chars().take(max_len).collect()
    }
}
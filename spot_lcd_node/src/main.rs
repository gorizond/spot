mod config;
mod lcd_core;
mod lcd_display_service;
mod temp_monitor;
mod uptime_monitor;

// Only include ros_publisher when ros2 feature is enabled
#[cfg(feature = "ros2")]
mod ros_publisher;

use anyhow::Result;
use config::Config;
use lcd_core::LcdDisplay;
use lcd_display_service::DisplayService;
#[cfg(target_os = "linux")]
use rppal::gpio::Gpio;
use std::sync::{Arc, Mutex};
use temp_monitor::TempMonitor;
use std::collections::HashMap;
use uptime_monitor::UptimeMonitor;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Which modules to run (comma-separated list: all, lcd, temp, uptime)
    #[arg(short, long, default_value = "all")]
    module: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    println!("Starting LCD service modules: {}", args.module);
    
    // Load configuration from environment variables
    let config = Config::from_env().unwrap_or_else(|_| {
        println!("Warning: Could not load config from environment, using defaults");
        // Return default config
        Config {
            lcd: config::LcdConfig {
                pin_rs: 25,
                pin_en: 24,
                pins_data: vec![23, 17, 18, 22],
                cols: 16,
                rows: 2,
            },
            temp_monitor: config::TempMonitorConfig {
                interval: 5,
            },
            uptime_monitor: config::UptimeMonitorConfig {
                interval: 10,
            },
            ros_publisher: config::RosPublisherConfig {
                temp_topic: "lcd_temperature".to_string(),
                uptime_topic: "lcd_uptime".to_string(),
            },
        }
    });
    
    // Parse modules to run
    let modules: Vec<&str> = args.module.split(',').map(|s| s.trim()).collect();
    
    // Determine if we should run all modules
    let run_all = modules.contains(&"all");
    
    // Initialize shared resources based on required modules
    let mut display_service: Option<DisplayService> = None;
    
    // Initialize LCD and display service if needed
    if run_all || modules.contains(&"lcd") {
        #[cfg(target_os = "linux")]
        {
            match Gpio::new() {
                Ok(gpio) => {
                    match LcdDisplay::new(&gpio, config.lcd.clone()) {
                        Ok(lcd) => {
                            let lcd_arc = Arc::new(Mutex::new(lcd));
                            let svc = DisplayService::new(Arc::clone(&lcd_arc), config.lcd.clone());
                            svc.start_display_loop();
                            display_service = Some(svc);
                        }
                        Err(e) => {
                            eprintln!("Failed to initialize LCD: {}", e);
                        }
                    }
                }
                Err(e) => {
                    eprintln!("GPIO not available on this platform: {}", e);
                }
            }
        }
        
        #[cfg(not(target_os = "linux"))]
        {
            let svc = DisplayService::new(config.lcd.clone());
            svc.start_display_loop();
            display_service = Some(svc);
        }
    }
    
    // Setup temperature monitoring if needed
    if run_all || modules.contains(&"temp") {
        let display_service_temp = if let Some(ref svc) = display_service {
            Arc::new(svc.get_lcd_reference())
        } else {
            Arc::new(Arc::new(Mutex::new(HashMap::<String, String>::new())))
        };
        
        #[cfg(feature = "ros2")]
        let temp_callback = {
            use ros_publisher::RosPublisher;
            let ros_publisher = Arc::new(RosPublisher::new()?);
            ros_publisher.start_ros_bridge();
            
            let display_service_temp = Arc::clone(&display_service_temp);
            let ros_publisher_temp = Arc::clone(&ros_publisher);
            
            Arc::new(move |temp: &str| {
                let mut data = display_service_temp.lock().unwrap();
                data.insert("temperature".to_string(), temp.to_string());
                ros_publisher_temp.publish_temperature(temp);
            })
        };
        
        #[cfg(not(feature = "ros2"))]
        let temp_callback = {
            let display_service_temp = Arc::clone(&display_service_temp);
            
            Arc::new(move |temp: &str| {
                let mut data = display_service_temp.lock().unwrap();
                data.insert("temperature".to_string(), temp.to_string());
                println!("Temperature: {}", temp);
            })
        };
        
        let temp_monitor = TempMonitor::new(temp_callback, config.temp_monitor.clone());
        temp_monitor.start_monitoring();
    }
    
    // Setup uptime monitoring if needed
    if run_all || modules.contains(&"uptime") {
        let display_service_uptime = if let Some(ref svc) = display_service {
            Arc::new(svc.get_lcd_reference())
        } else {
            Arc::new(Arc::new(Mutex::new(HashMap::<String, String>::new())))
        };
        
        #[cfg(feature = "ros2")]
        let uptime_callback = {
            use ros_publisher::RosPublisher;
            let ros_publisher = Arc::new(RosPublisher::new()?);
            ros_publisher.start_ros_bridge();
            
            let display_service_uptime = Arc::clone(&display_service_uptime);
            let ros_publisher_uptime = Arc::clone(&ros_publisher);
            
            Arc::new(move |uptime: &str| {
                let mut data = display_service_uptime.lock().unwrap();
                data.insert("uptime".to_string(), uptime.to_string());
                ros_publisher_uptime.publish_uptime(uptime);
            })
        };
        
        #[cfg(not(feature = "ros2"))]
        let uptime_callback = {
            let display_service_uptime = Arc::clone(&display_service_uptime);
            
            Arc::new(move |uptime: &str| {
                let mut data = display_service_uptime.lock().unwrap();
                data.insert("uptime".to_string(), uptime.to_string());
                println!("Uptime: {}", uptime);
            })
        };
        
        let uptime_monitor = UptimeMonitor::new(uptime_callback, config.uptime_monitor.clone());
        uptime_monitor.start_monitoring();
    }
    
    // Keep the main thread alive
    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
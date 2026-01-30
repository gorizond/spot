use anyhow::Result;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::runtime::Runtime;

#[cfg(feature = "ros2")]
use rclrs;
#[cfg(feature = "ros2")]
use std_msgs::msg::String as RosString;

pub struct RosPublisher {
    runtime: Runtime,
    temp_publisher: Arc<std::sync::Mutex<Option<String>>>,
    uptime_publisher: Arc<std::sync::Mutex<Option<String>>>,
    #[cfg(feature = "ros2")]
    node: Option<rclrs::Node>,
    #[cfg(feature = "ros2")]
    temp_ros_publisher: Option<rclrs::Publisher<RosString>>,
    #[cfg(feature = "ros2")]
    uptime_ros_publisher: Option<rclrs::Publisher<RosString>>,
}

impl RosPublisher {
    pub fn new() -> Result<Self> {
        let runtime = Runtime::new()?;
        let temp_publisher = Arc::new(std::sync::Mutex::new(None));
        let uptime_publisher = Arc::new(std::sync::Mutex::new(None));

        #[cfg(feature = "ros2")]
        let (node, temp_ros_publisher, uptime_ros_publisher) = Self::create_ros_publishers()?;

        #[cfg(not(feature = "ros2"))]
        let (node, temp_ros_publisher, uptime_ros_publisher) = (None, None, None);

        Ok(RosPublisher {
            runtime,
            temp_publisher,
            uptime_publisher,
            #[cfg(feature = "ros2")]
            node,
            #[cfg(feature = "ros2")]
            temp_ros_publisher,
            #[cfg(feature = "ros2")]
            uptime_ros_publisher,
        })
    }

    #[cfg(feature = "ros2")]
    fn create_ros_publishers() -> Result<(Option<rclrs::Node>, Option<rclrs::Publisher<RosString>>, Option<rclrs::Publisher<RosString>>)> {
        match rclrs::Context::new([]) {
            Ok(context) => {
                match context.create_node("lcd_display_node") {
                    Ok(node) => {
                        let temp_publisher = node.create_publisher::<RosString>(
                            "lcd_temperature", 
                            rclrs::QOS_PROFILE_DEFAULT
                        ).ok();

                        let uptime_publisher = node.create_publisher::<RosString>(
                            "lcd_uptime", 
                            rclrs::QOS_PROFILE_DEFAULT
                        ).ok();

                        Ok((Some(node), temp_publisher, uptime_publisher))
                    },
                    Err(_) => Ok((None, None, None)),
                }
            },
            Err(_) => Ok((None, None, None)),
        }
    }

    pub fn publish_temperature(&self, temp: &str) {
        // Update internal storage
        let mut publisher = self.temp_publisher.lock().unwrap();
        *publisher = Some(temp.to_string());
        
        // Publish to ROS2 if feature is enabled
        #[cfg(feature = "ros2")]
        {
            if let Some(ref ros_publisher) = self.temp_ros_publisher {
                let mut msg = RosString::new();
                msg.data = temp.to_string();
                let _ = ros_publisher.publish(&msg);
            }
        }
        
        // For non-ROS2 builds, we can log the publication
        #[cfg(not(feature = "ros2"))]
        {
            println!("Temperature published: {}", temp);
        }
    }

    pub fn publish_uptime(&self, uptime: &str) {
        // Update internal storage
        let mut publisher = self.uptime_publisher.lock().unwrap();
        *publisher = Some(uptime.to_string());
        
        // Publish to ROS2 if feature is enabled
        #[cfg(feature = "ros2")]
        {
            if let Some(ref ros_publisher) = self.uptime_ros_publisher {
                let mut msg = RosString::new();
                msg.data = uptime.to_string();
                let _ = ros_publisher.publish(&msg);
            }
        }
        
        // For non-ROS2 builds, we can log the publication
        #[cfg(not(feature = "ros2"))]
        {
            println!("Uptime published: {}", uptime);
        }
    }

    pub fn start_ros_bridge(&self) {
        let temp_subscriber = Arc::clone(&self.temp_publisher);
        let uptime_subscriber = Arc::clone(&self.uptime_publisher);

        self.runtime.spawn(async move {
            loop {
                // Получаем сообщения для публикации
                let temp_msg = {
                    let temp = temp_subscriber.lock().unwrap();
                    temp.as_ref().map(|s| s.clone())
                };
                
                let uptime_msg = {
                    let uptime = uptime_subscriber.lock().unwrap();
                    uptime.as_ref().map(|s| s.clone())
                };
                
                // Публикуем в ROS2 топики
                if let Some(temp) = temp_msg {
                    // В реальной реализации будет публикация в ROS2 топик
                    // Для отладки выводим в консоль
                    println!("Publishing temperature: {}", temp);
                }
                
                if let Some(uptime) = uptime_msg {
                    // В реальной реализации будет публикация в ROS2 топик
                    // Для отладки выводим в консоль
                    println!("Publishing uptime: {}", uptime);
                }
                
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        });
    }
}

// Async executor for ROS2 context if needed
#[cfg(feature = "ros2")]
impl RosPublisher {
    pub fn spin_once(&self) -> Result<()> {
        if let Some(ref node) = self.node {
            // Spin the ROS2 context to process callbacks
            rclrs::spin_once(node.context(), std::time::Duration::from_millis(10))?;
        }
        Ok(())
    }
}
#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <micro_ros_platformio.h>
#include "camera_pins.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <sensor_msgs/msg/compressed_image.h>

#define WIFI_SSID "xxxxxxxxxxxxxxxx"
#define WIFI_PWD "xxxxxxxxxxxxxxx"
#define AGENT_PORT 8888

#define CAMERA_LINK "camera_link"
#define IMAGE_TOPIC "image/compressed"
#define NODE_NAME "espcam"
#define PUBLISH_FRAME_STAMPS true // Convenient for further processing but costs performance

IPAddress AGENT_IP(192,168,1,196);

/*
user agent with docker
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6

image view in ml container (humble)
apt install ros-humble-image-transport-plugins ros-humble-image-view

camera pinout
https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera/blob/main/CameraWebServer_for_esp-arduino_3.0.x/camera_pins.h

ros1_bridge easy mode with Docker (hopefully)
https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/blob/main/Dockerfile

ros1_bridge information
https://github.com/ros2/ros1_bridge/blob/master/README.md

microros platformio
https://github.com/micro-ROS/micro_ros_platformio

microros image streaming optimisation
https://github.com/micro-ROS/micro_ros_setup/issues/676

change camera settings
https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/

###############

Network debugging

apt install netcat

nc -kl 11411 -> TCP
nc -lu 11411 -> UDP

###############

On my machine, the daemon can only connect when the firewall is disabled.

sudo ufw disable

Don't forget to re-enable after you are done.

sudo ufw enable

*/

rcl_publisher_t publisher;
sensor_msgs__msg__CompressedImage img_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

size_t allocated_buffer = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(10000);
    Serial.println("RCCHECK failed. Please restart MCU to attempt reconnection.");
  }
}

// Update message data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (PUBLISH_FRAME_STAMPS) {
      // Set header with current time
      rcl_time_point_value_t now = 0;
      rcl_ret_t rc = rcl_clock_get_now(&support.clock, &now);
      if (rc != RCL_RET_OK) {
        Serial.println("Message callback failed to fetch time. Skipping message...");
        return;
      }
      img_msg.header.stamp.sec = (int32_t)(now / 1000000000ULL);
      img_msg.header.stamp.nanosec = (uint32_t)(now % 1000000000ULL);
    }

    // Capture a frame from the camera
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Message callback failed to get frame from camera. Skipping frame...");
      return;
    }

    // Allocate data and copy the JPEG frame
    // Only reallocate when we actually need it (for performance)
    if (allocated_buffer != fb->len) {
      rosidl_runtime_c__uint8__Sequence__fini(&img_msg.data);
      if (!rosidl_runtime_c__uint8__Sequence__init(&img_msg.data, fb->len)) {
        Serial.println("Failed to allocate compressed image data buffer");
        esp_camera_fb_return(fb);
        return;
      }
      allocated_buffer = fb->len;
    }
    // if (img_msg.data.data != NULL) {
    //   rosidl_runtime_c__uint8__Sequence__fini(&img_msg.data);
    // }
    // if (!rosidl_runtime_c__uint8__Sequence__init(&img_msg.data, fb->len)) {
    //   Serial.println("Failed to allocate compressed image data buffer");
    //   esp_camera_fb_return(fb);
    //   return;
    // }
    memcpy(img_msg.data.data, fb->buf, fb->len); // copy frame and return buffer to cam

    // Return the frame buffer to camera task
    esp_camera_fb_return(fb);

    // Publish message
    RCSOFTCHECK(rcl_publish(&publisher, &img_msg, NULL));
  }
}

void setup() {
  // Configure serial transport for debugging
  Serial.begin(115200);

  // Wifi for ROS
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");


  /*  CAMERA  */
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_240X240; // FRAMESIZE_UXGA
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 8; //12
  config.fb_count = 2; // 1
  config.grab_mode = CAMERA_GRAB_LATEST; // originally not present

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  //s->reset(s);

  // initial sensors are flipped vertically and colors are a bit saturated
  //if (s->id.PID == OV3660_PID) {
  s->set_vflip(s, 0);        // flip it back
  //s->set_denoise(s, 0);
  s->set_brightness(s, 2);   // up the brightness just a bit
  s->set_saturation(s, -1);  // lower the saturation

  s->set_lenc(s, 0); // Disable lens correction for performance
  //s->set_awb_gain(s, 0); // Disable auto whitebalance for performance
  //s->set_exposure_ctrl(s, 0);
 
  /*  ROS  */
  // Initialize the micro-ROS transport over Wi-Fi
  set_microros_wifi_transports(WIFI_SSID, WIFI_PWD, AGENT_IP, AGENT_PORT);
  delay(2000);

  // Create init_options and set up node
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
    IMAGE_TOPIC));

  // Create timer
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Set image format and frame
  rosidl_runtime_c__String__assign(&img_msg.format, "jpeg");
  rosidl_runtime_c__String__assign(&img_msg.header.frame_id, CAMERA_LINK);
}

void loop() {
  delay(25);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(25)));
  //Serial.println("Running! :)");
}
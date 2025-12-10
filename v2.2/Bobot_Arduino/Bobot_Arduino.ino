#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h" // For frame2jpg
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Wire.h> // Added for Sensors
#include "camera_pins.h"

// --- Config ---
#define MOT1_A 12
#define MOT1_B 13
#define MOT2_A 14
#define MOT2_B 15

// --- BLE ---
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_STREAM_UUID "6e400004-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pStreamCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String rxString = "";

// --- Camera ---
bool camera_is_init = false;
bool stream_active = false;

// --- Neural Network ---
#include "model_data.h"
TinyModel model;

// --- Sensors State ---
struct SensorState {
    float temp = 0;
    float humidity = 0;
    float pressure = 0;
    float light = 0;
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float mx = 0, my = 0, mz = 0;
};
SensorState sensors;

// --- Runtime State ---
String current_ai_class = "unknown";
unsigned long last_ai_time = 0;

// --- Functions ---

void setupCamera() {
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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE; 
    config.frame_size = FRAMESIZE_QQVGA; // 160x120 (Native size for model)
    config.jpeg_quality = 12; 
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
    camera_is_init = true;
    Serial.println("Camera initialized");
}

void setMotor(int m1, int m2) {
    // Simple PWM logic (using analogWrite for Arduino ESP32)
    // Map -100..100 to PWM
    
    auto drive = [](int pinA, int pinB, int speed) {
        int pwm = map(abs(speed), 0, 100, 0, 255);
        if (speed > 0) {
            analogWrite(pinA, pwm);
            analogWrite(pinB, 0);
        } else if (speed < 0) {
            analogWrite(pinA, 0);
            analogWrite(pinB, pwm);
        } else {
            analogWrite(pinA, 0);
            analogWrite(pinB, 0);
        }
    };

    drive(MOT1_A, MOT1_B, m1);
    drive(MOT2_A, MOT2_B, m2);
}

// --- Sensor Logic ---
void initSensors() {
    // Initialize I2C. Note: Camera uses pins 1(SDA) and 2(SCL).
    // We try to share the bus or join it.
    Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
    Serial.println("Sensors I2C Init on Pins 1 & 2");
    
    // Check for sensors (Scanner)
    /*
    for(int i=1; i<127; i++) {
        Wire.beginTransmission(i);
        if(Wire.endTransmission() == 0) {
            Serial.printf("I2C Device at 0x%X\n", i);
        }
    }
    */
}

void updateSensors() {
    // Mock Data Implementation (Replace with real library calls)
    // Real implementation requires libraries for:
    // HTS221, LIS2MDL, LPS22HH, LSM6DSO, LIS2DW12, STTS751, VEML6040
    
    // Simulate changing values for testing logic
    static float t = 0;
    t += 0.01;
    
    sensors.temp = 24.0 + sin(t) * 5.0;      // 19 - 29 C
    sensors.humidity = 50.0 + cos(t) * 10.0; // 40 - 60 %
    sensors.pressure = 1013.0;
    sensors.light = 500 + sin(t*2) * 200;
    
    // Mock IMU (LSM6DSO + LIS2MDL)
    sensors.gx = sin(t*3) * 10;
    sensors.gy = cos(t*3) * 10;
    sensors.gz = sin(t*0.5) * 5;
    
    sensors.mx = 30 + sin(t) * 5;
    sensors.my = 10 + cos(t) * 5;
    sensors.mz = 45;
}

// Helper math for vector
std::vector<float> dense(const std::vector<float>& in, const std::vector<float>& w, const std::vector<float>& b, int out_dim, bool relu) {
    std::vector<float> out(out_dim);
    int in_dim = in.size();
    for (int i = 0; i < out_dim; i++) {
        float sum = b[i];
        for (int j = 0; j < in_dim; j++) {
            sum += in[j] * w[j * out_dim + i];
        }
        out[i] = (relu && sum < 0) ? 0 : sum;
    }
    return out;
}

int argmax(const std::vector<float>& v) {
    int idx = 0;
    float max_v = v[0];
    for (size_t i = 1; i < v.size(); i++) {
        if (v[i] > max_v) { max_v = v[i]; idx = i; }
    }
    return idx;
}

// Helper math for pointer
std::vector<float> dense_ptr(const float* in, const std::vector<float>& w, const std::vector<float>& b, int out_dim, bool relu, int in_dim) {
    std::vector<float> out(out_dim);
    
    // Check pointers
    if(in == NULL) { Serial.println("dense_ptr: NULL input"); return out; }
    if(w.size() == 0) { Serial.println("dense_ptr: Empty weights"); return out; }
    
    for (int i = 0; i < out_dim; i++) {
        float sum = b[i];
        for (int j = 0; j < in_dim; j++) {
            sum += in[j] * w[j * out_dim + i];
        }
        out[i] = (relu && sum < 0) ? 0 : sum;
    }
    return out;
}

String runInference(float* input) {
    if (!model.loaded) return "no_model";
    
    // L1
    std::vector<float> h1 = dense_ptr(input, model.w1, model.b1, model.hidden_dim, true, model.input_dim);
    
    // L2
    std::vector<float> out = dense(h1, model.w2, model.b2, model.output_dim, false);
    
    int cls = argmax(out);
    if (cls >= 0 && cls < model.classes.size()) return model.classes[cls];
    return "unknown";
}

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// --- Global Upload State ---
bool receiving_model = false;
bool parse_model_flag = false;
std::vector<uint8_t> model_raw_buffer;
// Buffers
size_t expected_model_size = 0;
float* ai_input_buffer = NULL; // Use raw pointer to control PSRAM usage 

const size_t MODEL_BYTE_SIZE = 1230000; 

// Helper for loading compressed bytes
void loadModelFromBytes(std::vector<uint8_t>& bytes) {
    if (bytes.size() < 100) return; // Basic check
    
    size_t idx = 0;
    
    // Read Scales (4 floats = 16 bytes)
    float s_w1, s_b1, s_w2, s_b2;
    memcpy(&s_w1, bytes.data() + idx, 4); idx += 4;
    memcpy(&s_b1, bytes.data() + idx, 4); idx += 4;
    memcpy(&s_w2, bytes.data() + idx, 4); idx += 4;
    memcpy(&s_b2, bytes.data() + idx, 4); idx += 4;
    
    // Resize vectors
    int in_dim = 160*120; // 19200
    int hid_dim = model.hidden_dim; 
    if (hid_dim < 1) hid_dim = 32;
    
    // out_dim set by CONFIG_MODEL command
    int out_dim = model.output_dim; 
    if (out_dim < 1) out_dim = 2; // Safety default
    
    // Update Model Dimensions Structure
    model.input_dim = in_dim;
    model.hidden_dim = hid_dim;
    model.output_dim = out_dim;
    
    size_t required_data_size = (in_dim * hid_dim) + hid_dim + (hid_dim * out_dim) + out_dim;
    size_t header_size = 16;
    
    Serial.printf("Parse: In=%d Hid=%d Out=%d. Need %d bytes + 16 header. Got %d\n", 
                  in_dim, hid_dim, out_dim, required_data_size, bytes.size());
                  
    if (bytes.size() < required_data_size + header_size) {
        Serial.println("Error: Buffer too small for this model config!");
        return;
    }
    
    model.w1.resize(in_dim * hid_dim);
    model.b1.resize(hid_dim);
    model.w2.resize(hid_dim * out_dim);
    model.b2.resize(out_dim);
    
    // Decompress W1
    int8_t* ptr = (int8_t*)(bytes.data() + idx);
    for(size_t i=0; i<model.w1.size(); i++) model.w1[i] = (float)(*ptr++) * s_w1;
    idx += model.w1.size();
    
    // Decompress B1
    for(size_t i=0; i<model.b1.size(); i++) model.b1[i] = (float)(*ptr++) * s_b1;
    idx += model.b1.size();
    
    // Decompress W2
    for(size_t i=0; i<model.w2.size(); i++) model.w2[i] = (float)(*ptr++) * s_w2;
    idx += model.w2.size();
    
    // Decompress B2
    for(size_t i=0; i<model.b2.size(); i++) model.b2[i] = (float)(*ptr++) * s_b2;
    idx += model.b2.size();
    
    model.input_dim = in_dim;
    model.hidden_dim = hid_dim;
    // model.output_dim is already set
    
    // Classes also set by CONFIG_MODEL
    
    model.loaded = true;
}

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      
      if (rxValue.length() > 0) {
        // 1. Binary Model Upload Mode
        if (receiving_model) {
            for (int i = 0; i < rxValue.length(); i++) {
                model_raw_buffer.push_back((uint8_t)rxValue[i]);
            }
            // Progress logging (every 10KB)
            if (model_raw_buffer.size() % 10000 < 20) {
                 Serial.printf("DL: %d / %d\n", model_raw_buffer.size(), expected_model_size);
            }

            if (model_raw_buffer.size() >= expected_model_size) {
                receiving_model = false;
                Serial.println("Model Downloaded. Scheduling parse...");
                parse_model_flag = true; // Defer parsing
            }
            return;
        }

        // 2. Text/JSON Mode
        // Accumulate JSON
        for (int i = 0; i < rxValue.length(); i++) {
            rxString += rxValue[i];
            if (rxValue[i] == '\n') {
                Serial.print("CMD: ");
                Serial.println(rxString.c_str());
                handleCommand(rxString);
                rxString = "";
            }
        }
      }
    }
};

// Scripting
DynamicJsonDocument scriptDoc(4096); // Buffer for rules
bool script_loaded = false;

// Helper to execute a single action command
void executeAction(JsonObject action) {
    const char* cmd = action["cmd"];
    if (strcmp(cmd, "M") == 0) {
        setMotor(action["l"], action["r"]);
    }
    else if (strcmp(cmd, "T") == 0) {
        int ms = action.containsKey("ms") ? (int)action["ms"] : 0;
        if (ms > 0 && ms < 10000) {
            delay(ms);
        }
    }
}

// Logic Evaluator
bool checkCondition(JsonVariant cond, String ai_class) {
    if (cond.is<const char*>()) {
        // Simple string matches AI Class
        return String(cond.as<const char*>()) == ai_class;
    }
    
    if (cond.is<JsonObject>()) {
        String type = cond["type"];
        
        // Sensor Logic
        if (type == "sensor") {
            String s = cond["sensor"];
            String op = cond["op"];
            float val = cond["val"];
            float current = 0;
            
            if (s == "temp") current = sensors.temp;
            else if (s == "hum") current = sensors.humidity;
            else if (s == "press") current = sensors.pressure;
            else if (s == "light") current = sensors.light;
            else if (s == "gyro_x") current = sensors.gx;
            else if (s == "gyro_y") current = sensors.gy;
            else if (s == "gyro_z") current = sensors.gz;
            else if (s == "mag_x") current = sensors.mx;
            else if (s == "mag_y") current = sensors.my;
            else if (s == "mag_z") current = sensors.mz;
            else return false; // Unknown sensor
            
            if (op == "gt") return current > val;
            if (op == "lt") return current < val;
            if (op == "gte") return current >= val;
            if (op == "lte") return current <= val;
            if (op == "eq") return abs(current - val) < 0.1;
        }
    }
    return false;
}

void runScript(String ai_class) {
    if (!script_loaded) return;
    
    updateSensors(); // Read fresh data
    
    JsonArray rules = scriptDoc.as<JsonArray>();
    for (JsonObject rule : rules) {
        JsonVariant condition = rule["if"];
        
        if (checkCondition(condition, ai_class)) {
            executeAction(rule["do"]);
            return; 
        }
    }
}

void handleCommand(String json) {
    StaticJsonDocument<512> doc; // Increased size
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
        Serial.println("JSON parse error");
        return;
    }
    
    const char* cmd = doc["cmd"];
    if (strcmp(cmd, "S") == 0) {
        stream_active = doc["val"];
    }
    else if (strcmp(cmd, "M") == 0) {
        setMotor(doc["l"], doc["r"]);
    }
    else if (strcmp(cmd, "SCRIPT") == 0) {
         scriptDoc.clear();
         JsonArray newRules = doc["rules"];
         for(auto v : newRules) scriptDoc.add(v);
         
         script_loaded = true;
         Serial.println("Script Updated!");
    }
    else if (strcmp(cmd, "CONFIG_MODEL") == 0) {
        // Set classes
        model.classes.clear();
        JsonArray cls = doc["classes"];
        for(String n : cls) {
            model.classes.push_back(n);
        }
        
        if (doc.containsKey("hid")) {
            model.hidden_dim = doc["hid"];
        } else {
            model.hidden_dim = 32; 
        }
        
        model.output_dim = model.classes.size();
        Serial.printf("Config: %d classes, Hid: %d\n", model.output_dim, model.hidden_dim);
    }
    else if (strcmp(cmd, "UL_START") == 0) {
        receiving_model = true;
        model_raw_buffer.clear();
        
        if (doc.containsKey("size")) {
             expected_model_size = doc["size"];
        } else {
             expected_model_size = 614500; // Fallback for 160x120x32 model
        }
        
        model_raw_buffer.reserve(expected_model_size);
        Serial.printf("Waiting for %d bytes...\n", expected_model_size);
    }
    // Add simple inference trigger for testing
    else if (strcmp(cmd, "I") == 0) {
         // Only if we implement frame capture for inference
         Serial.println("Inference triggered (mock)");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Bobot v2.2 Starting...");
    
    // I2C Init
    initSensors();
    
    if(psramFound()){
        Serial.printf("PSRAM: %d bytes\n", ESP.getPsramSize());
        ai_input_buffer = (float*)ps_malloc(19200 * sizeof(float));
    } else {
        Serial.println("No PSRAM found! Using Heap.");
        ai_input_buffer = (float*)malloc(19200 * sizeof(float));
    }
    
    loadModel(model);

    // Script is empty by default. User must upload it via Web.
    scriptDoc.clear();
    script_loaded = false;

    // Motors
    pinMode(MOT1_A, OUTPUT); pinMode(MOT1_B, OUTPUT);
    pinMode(MOT2_A, OUTPUT); pinMode(MOT2_B, OUTPUT);

    // Camera
    setupCamera();
    
    // Memory Allocation (After camera)
    if(psramFound()){
        Serial.printf("PSRAM: %d bytes\n", ESP.getPsramSize());
        ai_input_buffer = (float*)ps_malloc(19200 * sizeof(float));
    } else {
        Serial.println("No PSRAM found! Using Heap.");
        ai_input_buffer = (float*)malloc(19200 * sizeof(float));
    }
    
    loadModel(model);

    // BLE
    BLEDevice::init("Bobot-v2");
    BLEDevice::setMTU(517); // Boost speed!
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_RX_UUID,
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_WRITE_NR
                                       );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pTxCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_TX_UUID,
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
    pTxCharacteristic->addDescriptor(new BLE2902());

    pStreamCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_STREAM_UUID,
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
    pStreamCharacteristic->addDescriptor(new BLE2902());

    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE Started");
}

void loop() {
    // 0. Handle Model Parsing (Main Thread)
    if (parse_model_flag) {
        Serial.println("Parsing model in main loop...");
        loadModelFromBytes(model_raw_buffer);
        Serial.println("Model Activated!");
        model_raw_buffer.clear();
        parse_model_flag = false;
    }

    // 1. Handle Connection State
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // 2. Camera Processing (AI + Stream)
    // Always run if camera initialized (Autonomous Mode)
    if (camera_is_init) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (fb) {
            // A. Inference (Always run if model loaded)
            if (model.loaded && ai_input_buffer != NULL) {
                 int w = fb->width;
                 int h = fb->height;
                 
                 for(int y=0; y<120; y++) {
                     for(int x=0; x<160; x++) {
                         // Native resolution, no subsampling needed if camera is QQVGA
                         // Just copy and normalize
                         if (x >= w || y >= h) continue; // Safety
                         
                         uint8_t pixel = fb->buf[y * w + x];
                         ai_input_buffer[y*160 + x] = pixel / 255.0f;
                     }
                 }
                 
                 String result = runInference(ai_input_buffer);
                 
                 // Debug to Serial
                 static long last_print = 0;
                 if(millis() - last_print > 500) {
                    Serial.printf("AI: %s | Temp: %.1f\n", result.c_str(), sensors.temp);
                    last_print = millis();
                 }
                 
                 // Run Script Rules
                 if (script_loaded) {
                     runScript(result);
                 }
            }

            // B. Stream (Convert GRAYSCALE to JPEG)
            // Only send if connected AND requested
            if (deviceConnected && stream_active) {
                uint8_t * jpg_buf = NULL;
                size_t jpg_len = 0;
                // Quality 30 to reduce size for QVGA
                bool jpeg_converted = frame2jpg(fb, 30, &jpg_buf, &jpg_len); 
                
                if (jpeg_converted) {
                    size_t chunk_size = 240; 
                    for (size_t i = 0; i < jpg_len; i += chunk_size) {
                        if (!deviceConnected) break;
                        size_t s = (jpg_len - i > chunk_size) ? chunk_size : (jpg_len - i);
                        pStreamCharacteristic->setValue(jpg_buf + i, s);
                        pStreamCharacteristic->notify();
                        delay(10); 
                    }
                    free(jpg_buf); 
                }
            }
            
            esp_camera_fb_return(fb);
        } else {
            Serial.println("Camera capture failed");
        }
    }
    delay(1); // Prevent WDT triggers
}

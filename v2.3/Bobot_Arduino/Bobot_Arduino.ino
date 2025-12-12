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
    config.pixel_format = PIXFORMAT_GRAYSCALE; // Grayscale for faster processing
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

// v2.3: Optimized dense layer using compressed weights
std::vector<float> dense_compressed(const float* in, const int8_t* w_compressed, const int8_t* b_compressed, 
                                     float scale_w, float scale_b, int out_dim, bool relu, int in_dim) {
    std::vector<float> out(out_dim);
    
    if(in == NULL || w_compressed == NULL || b_compressed == NULL) {
        Serial.println("dense_compressed: NULL pointer");
        return out;
    }
    
    for (int i = 0; i < out_dim; i++) {
        float sum = b_compressed[i] * scale_b; // Decompress bias
        for (int j = 0; j < in_dim; j++) {
            sum += in[j] * (w_compressed[j * out_dim + i] * scale_w); // Decompress weight on-the-fly
        }
        out[i] = (relu && sum < 0) ? 0 : sum;
    }
    return out;
}

// --- Inference quality improvements ---
// Softmax confidence + short temporal smoothing reduces flicker for faces/gestures/objects.
static float g_last_prob = 0.0f;
static int g_last_cls = -1;

static float softmaxProbForIndex(const std::vector<float>& logits, int idx) {
    if (idx < 0 || idx >= (int)logits.size()) return 0.0f;
    float maxv = logits[0];
    for (size_t i = 1; i < logits.size(); i++) if (logits[i] > maxv) maxv = logits[i];
    float sum = 0.0f;
    for (size_t i = 0; i < logits.size(); i++) sum += expf(logits[i] - maxv);
    if (sum <= 0.0f) return 0.0f;
    return expf(logits[idx] - maxv) / sum;
}

String runInference(float* input) {
    if (!model.loaded) return "no_model";
    
    // v2.3: Use compressed weights with on-the-fly decompression
    std::vector<float> h1 = dense_compressed(input, model.w1_compressed, model.b1_compressed,
                                             model.scale_w1, model.scale_b1, 
                                             model.hidden_dim, true, model.input_dim);
    
    float* h1_arr = h1.data();
    std::vector<float> logits = dense_compressed(h1_arr, model.w2_compressed, model.b2_compressed,
                                                 model.scale_w2, model.scale_b2,
                                                 model.output_dim, false, model.hidden_dim);
    
    int cls = argmax(logits);
    float prob = softmaxProbForIndex(logits, cls);
    g_last_prob = prob;
    g_last_cls = cls;

    const float CONF_TH = 0.60f;
    const int WIN = 5;
    static int hist[WIN] = {-1, -1, -1, -1, -1};
    static float phist[WIN] = {0, 0, 0, 0, 0};
    static int pos = 0;
    static int lastStable = -1;

    if (cls >= 0 && cls < model.output_dim && prob >= CONF_TH) {
        hist[pos] = cls;
        phist[pos] = prob;
        pos = (pos + 1) % WIN;
    }

    // pick best by count, then by summed confidence
    int best = -1;
    int bestCount = 0;
    float bestScore = 0.0f;
    for (int c = 0; c < model.output_dim && c < 16; c++) {
        int cnt = 0;
        float score = 0.0f;
        for (int i = 0; i < WIN; i++) {
            if (hist[i] == c) { cnt++; score += phist[i]; }
        }
        if (cnt > bestCount || (cnt == bestCount && score > bestScore)) {
            best = c; bestCount = cnt; bestScore = score;
        }
    }

    int chosen = -1;
    if (best >= 0 && bestCount >= 2) chosen = best;
    else if (prob >= CONF_TH) chosen = cls;

    if (chosen >= 0) lastStable = chosen;
    if (chosen < 0) chosen = lastStable;

    if (chosen >= 0 && chosen < (int)model.classes.size()) return model.classes[chosen];
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
// Grayscale input - 160x120x1 = 19200 floats
const int AI_INPUT_SIZE = 160 * 120; 

// v2.3: Color RGB input (160x120x3 = 57600)
// Model size: ~3.7MB compressed, ~14.7MB uncompressed (requires PSRAM)
const size_t MODEL_BYTE_SIZE = 3700000; 

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
    
    // Dimensions come from CONFIG_MODEL (web sends in_w/in_h/in_dim).
    // Default is full frame grayscale 160x120 = 19200.
    int in_dim = model.input_dim;
    if (in_dim < 1) in_dim = 160 * 120;
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
    
    // v2.3: Store compressed weights in PSRAM (saves 4x memory)
    size_t w1_size = in_dim * hid_dim;
    size_t b1_size = hid_dim;
    size_t w2_size = hid_dim * out_dim;
    size_t b2_size = out_dim;
    
    // Free old weights if any
    if (model.w1_compressed) free(model.w1_compressed);
    if (model.b1_compressed) free(model.b1_compressed);
    if (model.w2_compressed) free(model.w2_compressed);
    if (model.b2_compressed) free(model.b2_compressed);
    
    // Allocate in PSRAM if available
    bool use_psram = psramFound();
    if (use_psram) {
        model.w1_compressed = (int8_t*)ps_malloc(w1_size);
        model.b1_compressed = (int8_t*)ps_malloc(b1_size);
        model.w2_compressed = (int8_t*)ps_malloc(w2_size);
        model.b2_compressed = (int8_t*)ps_malloc(b2_size);
    } else {
        model.w1_compressed = (int8_t*)malloc(w1_size);
        model.b1_compressed = (int8_t*)malloc(b1_size);
        model.w2_compressed = (int8_t*)malloc(w2_size);
        model.b2_compressed = (int8_t*)malloc(b2_size);
    }
    
    if (!model.w1_compressed || !model.b1_compressed || !model.w2_compressed || !model.b2_compressed) {
        Serial.println("Error: Failed to allocate memory for model!");
        return;
    }
    
    // Copy compressed weights directly (no decompression)
    // Check bounds before copying
    if (idx + w1_size + b1_size + w2_size + b2_size > bytes.size()) {
        Serial.println("Error: Not enough data in buffer for all weights!");
        // Free allocated memory
        if (model.w1_compressed) { free(model.w1_compressed); model.w1_compressed = NULL; }
        if (model.b1_compressed) { free(model.b1_compressed); model.b1_compressed = NULL; }
        if (model.w2_compressed) { free(model.w2_compressed); model.w2_compressed = NULL; }
        if (model.b2_compressed) { free(model.b2_compressed); model.b2_compressed = NULL; }
        return;
    }
    
    // Safe pointer arithmetic with bounds checking
    const uint8_t* src = bytes.data() + idx;
    memcpy(model.w1_compressed, src, w1_size);
    idx += w1_size;
    src = bytes.data() + idx;
    
    memcpy(model.b1_compressed, src, b1_size);
    idx += b1_size;
    src = bytes.data() + idx;
    
    memcpy(model.w2_compressed, src, w2_size);
    idx += w2_size;
    src = bytes.data() + idx;
    
    memcpy(model.b2_compressed, src, b2_size);
    idx += b2_size;
    
    // Store scales for decompression during inference
    model.scale_w1 = s_w1;
    model.scale_b1 = s_b1;
    model.scale_w2 = s_w2;
    model.scale_b2 = s_b2;
    
    model.w1_size = w1_size;
    model.b1_size = b1_size;
    model.w2_size = w2_size;
    model.b2_size = b2_size;
    
    model.input_dim = in_dim;
    model.hidden_dim = hid_dim;
    // model.output_dim is already set
    
    Serial.printf("Model loaded: %d KB compressed (saved ~75%% memory)\n", 
        (w1_size + b1_size + w2_size + b2_size) / 1024);
    
    model.loaded = true;
}

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      
      if (rxValue.length() > 0) {
        // 1. Binary Model Upload Mode - process ALL data as binary
        // CRITICAL: When receiving_model=true, NEVER try to parse as JSON
        if (receiving_model) {
            // Optimized: Use memcpy for faster data copy (much faster than push_back loop)
            size_t oldSize = model_raw_buffer.size();
            size_t newSize = oldSize + rxValue.length();
            model_raw_buffer.resize(newSize);
            memcpy(model_raw_buffer.data() + oldSize, rxValue.c_str(), rxValue.length());
            
            // Log first chunk to confirm we're receiving data
            if (oldSize == 0) {
                Serial.printf("First chunk received: %d bytes\n", rxValue.length());
            }
            
            // Progress logging (every 50KB for less overhead)
            if (model_raw_buffer.size() % 50000 < rxValue.length() || 
                (oldSize == 0) || 
                (model_raw_buffer.size() % 10000 < rxValue.length() && model_raw_buffer.size() < 100000)) {
                 Serial.printf("DL: %d / %d (%.1f%%) - chunk: %d bytes\n", 
                     model_raw_buffer.size(), expected_model_size,
                     100.0 * model_raw_buffer.size() / expected_model_size,
                     rxValue.length());
            }

            if (model_raw_buffer.size() >= expected_model_size) {
                receiving_model = false;
                Serial.printf("Model Downloaded! Size: %d bytes (expected: %d)\n", 
                    model_raw_buffer.size(), expected_model_size);
                Serial.println("Scheduling parse...");
                parse_model_flag = true; // Defer parsing
            }
            return; // CRITICAL: Exit early - don't process as JSON
        }

        // 2. Text/JSON Mode - only process if NOT in binary mode
        // Accumulate JSON line by line
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

        // Optional: smaller input resolution for faster upload and inference
        if (doc.containsKey("in_w")) model.input_w = doc["in_w"];
        if (doc.containsKey("in_h")) model.input_h = doc["in_h"];
        if (doc.containsKey("in_dim")) model.input_dim = doc["in_dim"];
        if (model.input_w < 1 || model.input_h < 1 || model.input_dim < 1) {
            model.input_w = 160;
            model.input_h = 120;
            model.input_dim = 160 * 120;
        }
        
        model.output_dim = model.classes.size();
        Serial.printf("Config: %d classes, Hid: %d, In: %dx%d (%d)\n",
                      model.output_dim, model.hidden_dim, model.input_w, model.input_h, model.input_dim);
    }
    else if (strcmp(cmd, "UL_START") == 0) {
        // Reset any previous upload state
        if (receiving_model) {
            Serial.println("UL_START: Resetting previous upload state");
            receiving_model = false;
            model_raw_buffer.clear();
        }
        
        receiving_model = true;
        model_raw_buffer.clear();
        
        if (doc.containsKey("size")) {
             expected_model_size = doc["size"];
             Serial.printf("UL_START: Received size = %d bytes (%.1f KB)\n", 
                 expected_model_size, expected_model_size / 1024.0);
        } else {
             expected_model_size = 650000; // Fallback for 160x120x1x32 grayscale model
             Serial.println("UL_START: No size provided, using fallback");
        }
        
        // Pre-allocate buffer for faster upload (optimized for speed)
        model_raw_buffer.clear();
        model_raw_buffer.reserve(expected_model_size + 2048); // Extra space to avoid reallocations
        
        // Send ACK back to web interface (fast, non-blocking)
        if (pTxCharacteristic && deviceConnected) {
            String ack = "{\"op\":\"ack\",\"phase\":\"ul_start\",\"ok\":true,\"size\":" + String(expected_model_size) + "}\n";
            pTxCharacteristic->setValue(ack.c_str());
            pTxCharacteristic->notify();
            // No delay - let BLE stack handle it asynchronously
        }
        
        Serial.printf("Ready! Waiting for %d bytes (%.1f KB)...\n", 
            expected_model_size, expected_model_size / 1024.0);
    }
    // Add simple inference trigger for testing
    else if (strcmp(cmd, "I") == 0) {
         // Only if we implement frame capture for inference
         Serial.println("Inference triggered (mock)");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Bobot v2.3 Starting...");
    
    // I2C Init
    initSensors();

    // Script is empty by default. User must upload it via Web.
    scriptDoc.clear();
    script_loaded = false;

    // Motors
    pinMode(MOT1_A, OUTPUT); pinMode(MOT1_B, OUTPUT);
    pinMode(MOT2_A, OUTPUT); pinMode(MOT2_B, OUTPUT);

    // Camera
    setupCamera();
    
    // Memory Allocation (After camera) - Grayscale (19200 floats)
    if(psramFound()){
        Serial.printf("PSRAM: %d bytes\n", ESP.getPsramSize());
        ai_input_buffer = (float*)ps_malloc(AI_INPUT_SIZE * sizeof(float));
    } else {
        Serial.println("No PSRAM found! Using Heap.");
        ai_input_buffer = (float*)malloc(AI_INPUT_SIZE * sizeof(float));
    }
    
    loadModel(model);

    // BLE
    BLEDevice::init("Bobot-v2.3");
    BLEDevice::setMTU(517); // Maximum MTU for speed (517 is max for ESP32)
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
                 
                 // Input resolution can be smaller than camera (e.g. 80x60) to reduce model size & upload time
                 int outW = model.input_w > 0 ? model.input_w : 160;
                 int outH = model.input_h > 0 ? model.input_h : 120;
                 if (outW < 1) outW = 160;
                 if (outH < 1) outH = 120;
                 if (outW > w) outW = w;
                 if (outH > h) outH = h;

                 // Calculate stride (bytes per row, may be aligned)
                 size_t stride = fb->len / h;
                 if (stride < (size_t)w) stride = w;
                 
                 // Min/max + mean over sampled grid (matches web preprocessing)
                 uint8_t min_val = 255, max_val = 0;
                 float sum = 0.0f;
                 int count = 0;
                 for (int yy = 0; yy < outH; yy++) {
                     int srcY = (yy * h) / outH;
                     for (int xx = 0; xx < outW; xx++) {
                         int srcX = (xx * w) / outW;
                         uint8_t pixel = fb->buf[srcY * stride + srcX];
                         if (pixel < min_val) min_val = pixel;
                         if (pixel > max_val) max_val = pixel;
                         sum += pixel;
                         count++;
                     }
                 }
                 float range = (float)(max_val - min_val);
                 if (range < 10.0f) range = 255.0f;
                 float mean = (count > 0) ? (sum / (float)count) : 0.0f;
                 
                 // Fill input buffer (first input_dim values are used by inference)
                 int idx = 0;
                 for (int yy = 0; yy < outH; yy++) {
                     int srcY = (yy * h) / outH;
                     for (int xx = 0; xx < outW; xx++) {
                         int srcX = (xx * w) / outW;
                         uint8_t pixel = fb->buf[srcY * stride + srcX];
                         float stretched = (pixel - min_val) / range; // 0..1
                         float normalized = (stretched - (mean / 255.0f - 0.5f)) * 1.2f + 0.5f;
                         normalized = fmax(0.0f, fmin(1.0f, normalized));
                         ai_input_buffer[idx++] = normalized;
                     }
                 }
                 
                 String result = runInference(ai_input_buffer);
                 
                 // Debug to Serial
                 static long last_print = 0;
                 if(millis() - last_print > 500) {
                    Serial.printf("AI: %s (p=%.2f) | Temp: %.1f\n", result.c_str(), g_last_prob, sensors.temp);
                    last_print = millis();
                 }
                 
                 // Run Script Rules
                 if (script_loaded) {
                     runScript(result);
                 }
            }

            // B. Stream (Convert RGB565 to JPEG)
            // Only send if connected AND requested
            if (deviceConnected && stream_active) {
                uint8_t * jpg_buf = NULL;
                size_t jpg_len = 0;
                // Quality 30 to reduce size for QQVGA
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

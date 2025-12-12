#ifndef MODEL_DATA_H
#define MODEL_DATA_H
#include <Arduino.h>
#include <vector>

struct TinyModel {
    // v2.3: Store compressed weights in PSRAM to save memory
    int8_t* w1_compressed = nullptr;  // Compressed weights (int8)
    int8_t* b1_compressed = nullptr;
    int8_t* w2_compressed = nullptr;
    int8_t* b2_compressed = nullptr;
    float scale_w1 = 1.0f, scale_b1 = 1.0f, scale_w2 = 1.0f, scale_b2 = 1.0f;
    size_t w1_size = 0, b1_size = 0, w2_size = 0, b2_size = 0;
    
    std::vector<String> classes;
    int input_dim = 160 * 120;
    int input_w = 160;
    int input_h = 120;
    int hidden_dim;
    int output_dim;
    bool loaded = false;
    
    // Cleanup
    ~TinyModel() {
        if (w1_compressed) free(w1_compressed);
        if (b1_compressed) free(b1_compressed);
        if (w2_compressed) free(w2_compressed);
        if (b2_compressed) free(b2_compressed);
    }
};

// Paste your exported code here later
void loadModel(TinyModel& model) {
    model.loaded = false;
}
#endif


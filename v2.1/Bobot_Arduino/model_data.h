#ifndef MODEL_DATA_H
#define MODEL_DATA_H
#include <Arduino.h>
#include <vector>

struct TinyModel {
    std::vector<float> w1;
    std::vector<float> b1;
    std::vector<float> w2;
    std::vector<float> b2;
    std::vector<String> classes;
    int input_dim;
    int hidden_dim;
    int output_dim;
    bool loaded = false;
};

// Paste your exported code here later
void loadModel(TinyModel& model) {
    model.loaded = false;
}
#endif


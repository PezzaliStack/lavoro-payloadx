// bms.h – PayloadX

#pragma once

struct bmsData {
    bool ch1, ch2, ch3, ch4;
    float batV;
    float batPct;
    float sol1V;
    float sol2V;
    int errorCode;
};

void parseBMS(const String &line, bmsData &data);

// bms.cpp – PayloadX

#include "bms.h"

void parseBMS(const String &line, bmsData &data) {
    if (line.startsWith("$BMS")) {
        data.ch1 = line.substring(7, 8) == "1";
        data.ch2 = line.substring(9, 10) == "1";
        data.ch3 = line.substring(11, 12) == "1";
        data.ch4 = line.substring(13, 14) == "1";
        data.batV = line.substring(15, 19).toFloat();
        data.batPct = line.substring(20, 26).toFloat();
        data.sol1V = line.substring(27, 31).toFloat();
        data.sol2V = line.substring(32, 36).toFloat();
        data.errorCode = 0;  // estendibile
    }
}

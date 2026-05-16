// attitude.cpp – PayloadX
// Wrapper sottile attorno al filtro Madgwick (src/Madgwick.{h,cpp}).
// Tiene il filtro come istanza statica e converte sensorData -> quaternione.

#include "attitude.h"
#include "Madgwick.h"

// Gain del filtro. Valore tipico ~0.1: piu' alto = convergenza piu'
// rapida ma piu' rumore; piu' basso = piu' liscio ma piu' lento a
// rispondere a cambi di assetto. Esposto qui per facilita' di tuning.
static const float MADGWICK_BETA = 0.1f;

static Madgwick filter;
static bool initialized = false;

void initAttitude() {
    filter.begin(MADGWICK_BETA);
    initialized = true;
}

void updateAttitude(const sensorData &imu, attitudeData &att) {
    if (!initialized) {
        att.qw = 1.0f;
        att.qx = att.qy = att.qz = 0.0f;
        return;
    }
    // L'IMU restituisce gyro in deg/s e accel in g; Madgwick li accetta
    // direttamente. Mag in uT (qualsiasi unita' coerente: viene normalizzato).
    filter.update(imu.gx, imu.gy, imu.gz,
                  imu.ax, imu.ay, imu.az,
                  imu.mx, imu.my, imu.mz);
    att.qw = filter.getQuatW();
    att.qx = filter.getQuatX();
    att.qy = filter.getQuatY();
    att.qz = filter.getQuatZ();
}

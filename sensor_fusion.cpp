/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "sensor_fusion.h"
#include "libfusion.h"

#define DEBUG 1
#ifdef	DEBUG
#ifdef __ANDORID__
#include <android/log.h>
#define DBG(...)  __android_log_print(ANDROID_LOG_INFO, "Sensor Fusion", __VA_ARGS__)
#else
#define DBG(...)	printf(__VA_ARGS__)
#endif
#else
#define DBG(...)
#endif

SensorFusion::SensorFusion()
    :mGyroTime(0), mAccTime(0)
{

    mEnabled[FUSION_9AXIS] = false;
    mEnabled[FUSION_NOMAG] = false;
    mEnabled[FUSION_NOGYRO] = false;

    // 200 Hz for gyro events is a good compromise between precision
    // and power/cpu usage.
    mEstimatedGyroRate = 200;
    mTargetDelayNs = 1000000000LL/mEstimatedGyroRate;

    for (int i = 0; i<NUM_FUSION_MODE; ++i) {
        mFusions[i].init(i);
    }
}


void SensorFusion::process(const sf_data& data ) {

    if (data.type == TYPE_GYROSCOPE) {
        float dT;
        if ( data.timeStamp - mGyroTime > 0 &&
             data.timeStamp - mGyroTime < (int64_t)(5e7) ) { //0.05sec

            dT = (data.timeStamp - mGyroTime) / 1000000000.0f;
            // here we estimate the gyro rate (useful for debugging)
            const float freq = 1 / dT;
            if (freq >= 100 && freq<1000) { // filter values obviously wrong
                const float alpha = 1 / (1 + dT); // 1s time-constant
                mEstimatedGyroRate = freq + (mEstimatedGyroRate - freq)*alpha;
            }
			
			vec3_t gyro;
            gyro.x = data.axis.x;
            gyro.y = data.axis.y;
            gyro.z = data.axis.z;

            for (int i = 0; i<NUM_FUSION_MODE; ++i) {
                if (mEnabled[i]) {
                    // fusion in no gyro mode will ignore
                    mFusions[i].handleGyro(gyro, dT);
                }
            }
        }
        mGyroTime = data.timeStamp;
    } else if (data.type == TYPE_MAGNETIC_FIELD) {
		
		vec3_t mag;
        mag.x = data.axis.x;
        mag.y = data.axis.y;
        mag.z = data.axis.z;

        for (int i = 0; i<NUM_FUSION_MODE; ++i) {
            if (mEnabled[i]) {
                mFusions[i].handleMag(mag);// fusion in no mag mode will ignore
            }
        }
    } else if (data.type == TYPE_ACCELEROMETER) {
        float dT;
        if ( data.timeStamp - mAccTime> 0 &&
                data.timeStamp - mAccTime< (int64_t)(1e8) ) { //0.1sec
            dT = (data.timeStamp - mAccTime) / 1000000000.0f;

			vec3_t acc;
	        acc.x = data.axis.x;
	        acc.y = data.axis.y;
	        acc.z = data.axis.z;

            for (int i = 0; i<NUM_FUSION_MODE; ++i) {
                if (mEnabled[i]) {
                    mFusions[i].handleAcc(acc, dT);
                    mAttitudes[i] = mFusions[i].getAttitude();
                }
            }
        }
        mAccTime = data.timeStamp;
    }
}

template <typename T> inline T min(T a, T b) { return a<b ? a : b; }
template <typename T> inline T max(T a, T b) { return a>b ? a : b; }

int SensorFusion::activate(int mode, bool enabled) {

    DBG("SensorFusion::activate(mode=%d, enabled=%d)",mode, enabled);

    if (enabled != mEnabled[mode]) {
        mEnabled[mode] = enabled;
        if (enabled) {
            mFusions[mode].init(mode);
        }
    }

    return 0;
}


void SensorFusion::dump() {
    const Fusion& fusion_9axis(mFusions[FUSION_9AXIS]);
    DBG("9-axis fusion %s , gyro-rate=%7.2fHz, "
            "q=< %g, %g, %g, %g > (%g), "
            "b=< %g, %g, %g >\n",
            mEnabled[FUSION_9AXIS] ? "enabled" : "disabled",
            mEstimatedGyroRate,
            fusion_9axis.getAttitude().x,
            fusion_9axis.getAttitude().y,
            fusion_9axis.getAttitude().z,
            fusion_9axis.getAttitude().w,
            length(fusion_9axis.getAttitude()),
            fusion_9axis.getBias().x,
            fusion_9axis.getBias().y,
            fusion_9axis.getBias().z);

    const Fusion& fusion_nomag(mFusions[FUSION_NOMAG]);
    DBG("game fusion(no mag) %s , "
            "gyro-rate=%7.2fHz, "
            "q=< %g, %g, %g, %g > (%g), "
            "b=< %g, %g, %g >\n",
            mEnabled[FUSION_NOMAG] ? "enabled" : "disabled",
            mEstimatedGyroRate,
            fusion_nomag.getAttitude().x,
            fusion_nomag.getAttitude().y,
            fusion_nomag.getAttitude().z,
            fusion_nomag.getAttitude().w,
            length(fusion_nomag.getAttitude()),
            fusion_nomag.getBias().x,
            fusion_nomag.getBias().y,
            fusion_nomag.getBias().z);

    const Fusion& fusion_nogyro(mFusions[FUSION_NOGYRO]);
    DBG("geomag fusion (no gyro) %s , "
            "gyro-rate=%7.2fHz, "
            "q=< %g, %g, %g, %g > (%g), "
            "b=< %g, %g, %g >\n",
            mEnabled[FUSION_NOGYRO] ? "enabled" : "disabled",
            mEstimatedGyroRate,
            fusion_nogyro.getAttitude().x,
            fusion_nogyro.getAttitude().y,
            fusion_nogyro.getAttitude().z,
            fusion_nogyro.getAttitude().w,
            length(fusion_nogyro.getAttitude()),
            fusion_nogyro.getBias().x,
            fusion_nogyro.getBias().y,
            fusion_nogyro.getBias().z);
}

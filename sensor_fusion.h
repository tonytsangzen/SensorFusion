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

#ifndef ANDROID_SENSOR_FUSION_H
#define ANDROID_SENSOR_FUSION_H

#include <stdint.h>
#include <sys/types.h>

#include "fusion.h"

typedef struct _SensorData{
	int type;
	uint64_t timeStamp;
	vec3_t data;
}SensorData;

class SensorFusion  {

    Fusion mFusions[NUM_FUSION_MODE]; // normal, no_mag, no_gyro
    bool mEnabled[NUM_FUSION_MODE];
    vec4_t mAttitudes[NUM_FUSION_MODE];
    float mEstimatedGyroRate;
    long mTargetDelayNs;
    long mGyroTime;
    long mAccTime;

public:
    SensorFusion();
    void process(const SensorData& sdata);

    bool isEnabled() const {
        return mEnabled[FUSION_9AXIS] ||
                mEnabled[FUSION_NOMAG] ||
                mEnabled[FUSION_NOGYRO];
    }

    bool hasEstimate(int mode = FUSION_9AXIS) const {
        return mFusions[mode].hasEstimate();
    }

    mat33_t getRotationMatrix(int mode = FUSION_9AXIS) const {
        return mFusions[mode].getRotationMatrix();
    }

    vec4_t getAttitude(int mode = FUSION_9AXIS) const {
        return mAttitudes[mode];
    }

    vec3_t getGyroBias() const { return mFusions[FUSION_9AXIS].getBias(); }
    float getEstimatedRate() const { return mEstimatedGyroRate; }
    int activate(int mode, bool enabled);
    void dump();
};
#endif

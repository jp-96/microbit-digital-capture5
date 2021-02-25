/*
MIT License

Copyright (c) 2021 jp-rad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef MICROBIT_INDOOR_BIKE_STEP_SENSOR_H
#define MICROBIT_INDOOR_BIKE_STEP_SENSOR_H

#include "MicroBit.h"
#include "MicroBitCustom.h"
#include "MicroBitCustomComponent.h"
#include <queue>

/**
  * Status flags
  */
// Universal flags used as part of the status field
// #define MICROBIT_COMPONENT_RUNNING		0x01
#define MICROBIT_INDOOR_BIKE_STEP_SENSOR_ADDED_TO_IDLE              0x02

#define MIN_RESISTANCE_LEVEL10 10
#define MAX_RESISTANCE_LEVEL10 80

enum MicrobitIndoorBikeStepSensorPin
{
    EDGE_P0 = 0,
    EDGE_P1 = 1,
    EDGE_P2 = 2
};

static const int MICROBIT_INDOOR_BIKE_STEP_SENSOR_EVENT_IDs[] = {
    MICROBIT_ID_IO_P0,
    MICROBIT_ID_IO_P1,
    MICROBIT_ID_IO_P2
};

class MicroBitIndoorBikeStepSensor : public MicroBitCustomComponent
{
private:
    MicroBit &uBit;
    
    static const uint64_t SENSOR_UPDATE_PERIOD_US = 1000000; // 1.0s
    static const uint64_t INTERVAL_LIST_SIZE = 3;
    static const uint64_t MAX_STEPS_INTERVAL_TIME_US = 2500000; // 2.5s
    
public:
    // Constructor.
    MicroBitIndoorBikeStepSensor(MicroBit &_uBit, MicrobitIndoorBikeStepSensorPin pin = EDGE_P2, uint16_t id = MICROBIT_INDOORBIKE_STEP_SENSOR_ID);

    /**
      * Periodic callback from MicroBit idle thread.
      */
    virtual void idleTick();

private:
    // STEP信号の計測時間のリスト（単位: マイクロ秒 - 1秒/1000000）
    std::queue<uint64_t> intervalList;
    
    // 最新のインターバル時間（単位: マイクロ秒 - 1秒/1000000）
    uint32_t lastIntervalTime;
    // 最新のクランク回転数（単位：rpm の 2倍）
    uint32_t lastCadence2;
    // 最新の速度（単位： km/h の 100倍）
    uint32_t lastSpeed100;
    // 最新のパワー（単位： watt）
    int16_t lastPower;
    
    // 次のupdate実行時間
    uint64_t updateSampleTimestamp;
    
    // 負荷のレベル（範囲：10～80） - パワーの算出用
    uint8_t resistanceLevel10;

private:
    // クランク回転数と速度、パワーを再計算する（最新化）
    void update();
    // クランク間時間から、クランク回転数と速度、パワーを計算する。
    void calcIndoorBikeData(uint32_t crankIntervalTime, uint8_t resistanceLevel10, uint32_t* cadence2, uint32_t* speed100, int16_t* power);

public:
    // インターバル時間を取得する（単位: マイクロ秒 - 1秒/1000000）
    uint32_t getIntervalTime(void);
    // クランク回転数を取得する（単位：rpm の 2倍）
    uint32_t getCadence2(void);
    // 速度を取得する（単位： km/h の 100倍）
    uint32_t getSpeed100(void);
    // パワーを取得する（単位： watt）
    int16_t getPower(void);
    // 負荷のレベルを取得・設定する（範囲：10～80）
    uint8_t getResistanceLevel10(void);
    void setResistanceLevel10(uint8_t resistanceLevel10);

private:
    // STEPセンサーのイベントハンドラ
    void onStepSensor(MicroBitEvent);

private:
    // Coefficient of Cadence and Speed
    static const uint64_t K_STEP_CADENCE =  120000000;
    static const uint64_t K_STEP_SPEED   = 1800000000;

    // https://diary.cyclekikou.net/archives/15876
    static const double K_POWER = 0.8 * (70 * 9.80665) / (360 * 0.95 * 100); // weight(70kg)
    static const double K_INCLINE_A = 0.9; // Incline(%) - a
    static const double K_INCLINE_B = 0.6; // Incline(%) - b

};

#endif /* #ifndef MICROBIT_INDOOR_BIKE_STEP_SENSOR_H */
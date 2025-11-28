/*
 * Lmx2594.h
 *
 *  Created on: Nov 28, 2025
 *      Author: alex
 */

#ifndef LIBRARY_LMX2594_H_
//#define LIBRARY_LMX2594_H_

#include <cstdint>
#include <cstddef>
#include "stm32g4xx_hal.h"   // при необходимости подставь свой конкретный HAL-хедер

class Lmx2594 {
public:
    // Описание подключённых пинов
    struct Pins {
        GPIO_TypeDef* cs_port;   // CSB
        uint16_t      cs_pin;

        GPIO_TypeDef* ce_port;   // CE (chip enable)
        uint16_t      ce_pin;
    };

    // Конфигурация входного генератора (OSCin-путь)
    struct RefConfig {
        double  f_ref_Hz;     // частота генератора на OSCin, Гц
        bool    osc_doubler;  // OSC_2X (удвоитель на входе)
        uint8_t mult;         // MULT: 1 (bypass), 3..7 (програмируемый множитель)
        uint16_t r_pre;       // PLL_R_PRE (предделитель до MULT), >= 1
        uint16_t r;           // PLL_R (последний делитель до PFD), >= 1
    };

    // Конструктор – сохраняем SPI, пины и параметры опорника
    Lmx2594(SPI_HandleTypeDef* spi, const Pins& pins, const RefConfig& ref);

    // Базовая инициализация микросхемы (reset + статические вещи, без установки частоты)
    HAL_StatusTypeDef init();

    // Инициализация + установка стартовой частоты RFoutA
    HAL_StatusTypeDef init(double fout_Hz);

    // Установка выходной частоты (RFoutA), Гц
    HAL_StatusTypeDef setFrequency(double fout_Hz);

    // Жёсткое выключение/включение по CE-пину (аппаратный power-down)
    void powerDownHard();
    void powerUpHard();

    // Низкоуровневая запись регистра (addr = номер Rxx из даташита)
    HAL_StatusTypeDef writeReg(uint8_t addr, uint16_t value);

    // Запись массива регистров (например, экспорт из TICS Pro)
    struct Reg {
        uint8_t  addr;
        uint16_t value;
    };
    HAL_StatusTypeDef writeRegs(const Reg* table, size_t count);

    // Немного телеметрии
    double getFoutHz() const { return _f_out_Hz; }
    double getFvcoHz() const { return _f_vco_Hz; }
    double getFpdHz()  const { return _f_pd_Hz;  }

private:
    SPI_HandleTypeDef* _spi;
    Pins        _pins;
    RefConfig   _ref;

    double      _f_pd_Hz  = 0.0;   // частота фазового детектора
    double      _f_vco_Hz = 0.0;   // частота VCO
    double      _f_out_Hz = 0.0;   // фактическая выходная частота
    uint32_t    _pll_den  = 1u << 24; // используем DEN = 2^24 (разумный компромисс)

    struct FreqPlan {
        double   f_vco;
        double   f_out;
        uint8_t  chdiv_code;   // код CHDIV (R75[10:6]) согласно даташиту
        bool     use_chdiv;    // true = используем делитель канала, false = VCO напрямую
        uint32_t pll_n;
        uint32_t pll_num;
        uint32_t pll_den;
        uint8_t  mash_order;   // 0 – integer, 1..4 – порядок MASH
        uint8_t  pfd_dly_sel;  // PFD_DLY_SEL (R37[13:8])
        bool     integer_mode;
    };

    // Расчёт плана частоты (VCO, N, NUM/DEN, CHDIV и прочее)
    bool computeFreqPlan(double fout_Hz, FreqPlan& plan);

    // Запись регистров под уже рассчитанный план частоты (без FCAL_EN)
    HAL_StatusTypeDef applyFreqPlan(const FreqPlan& plan);

    // Запуск VCO calibration (R0.FCAL_EN=1) с корректными FCAL_* полями
    HAL_StatusTypeDef runFcal(const FreqPlan& plan);

    // Вспомогательные штуки для SPI
    void csLow();
    void csHigh();
};

#endif /* LIBRARY_LMX2594_H_ */

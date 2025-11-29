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
#include <cstddef>    // Для size_t
#include <iterator>   // Для std::size() (в C++17)
#include <array>      // Если используете std::array
#include <cstdlib>    // Альтернатива для size_t

// Или просто
#include <cstdint>    // Для стандартных типов

static const uint32_t lmx_5200_regs[] = { 0x700000, 0x6F0000, 0x6E0000,
		0x6D0000, 0x6C0000, 0x6B0000, 0x6A0000, 0x690021, 0x680000, 0x670000,
		0x663F80, 0x650011, 0x640000, 0x630000, 0x620200, 0x610888, 0x600000,
		0x5F0000, 0x5E0000, 0x5D0000, 0x5C0000, 0x5B0000, 0x5A0000, 0x590000,
		0x580000, 0x570000, 0x56FFFF, 0x55D2FF, 0x540001, 0x530000, 0x521E00,
		0x510000, 0x506666, 0x4F0026, 0x4E0003, 0x4D0000, 0x4C000C, 0x4B0800,
		0x4A0000, 0x49003F, 0x480001, 0x470081, 0x46C350, 0x450000, 0x4403E8,
		0x430000, 0x4201F4, 0x410000, 0x401388, 0x3F0000, 0x3E0322, 0x3D00A8,
		0x3C0000, 0x3B0001, 0x3A9001, 0x390020, 0x380000, 0x370000, 0x360000,
		0x350000, 0x340820, 0x330080, 0x320000, 0x314180, 0x300300, 0x2F0300,
		0x2E07FC, 0x2DC0DF, 0x2C1FA3, 0x2B0000, 0x2A0000, 0x290000, 0x280000,
		0x2703E8, 0x260000, 0x250404, 0x240034, 0x230004, 0x220000, 0x211E21,
		0x200393, 0x1F43EC, 0x1E318C, 0x1D318C, 0x1C0488, 0x1B0002, 0x1A0DB0,
		0x190C2B, 0x18071A, 0x17007C, 0x160001, 0x150401, 0x14E048, 0x1327B7,
		0x120064, 0x11012C, 0x100080, 0x0F064F, 0x0E1E70, 0x0D4000, 0x0C5001,
		0x0B0018, 0x0A10D8, 0x091604, 0x082000, 0x0740B2, 0x06C802, 0x0500C8,
		0x040A43, 0x030642, 0x020500, 0x010808, 0x00251C };

static const uint32_t lmx_5210_regs[] = { 0x700000, 0x6F0000, 0x6E0000,
		0x6D0000, 0x6C0000, 0x6B0000, 0x6A0000, 0x690021, 0x680000, 0x670000,
		0x663F80, 0x650011, 0x640000, 0x630000, 0x620200, 0x610888, 0x600000,
		0x5F0000, 0x5E0000, 0x5D0000, 0x5C0000, 0x5B0000, 0x5A0000, 0x590000,
		0x580000, 0x570000, 0x56FFFF, 0x55D2FF, 0x540001, 0x530000, 0x521E00,
		0x510000, 0x506666, 0x4F0026, 0x4E0003, 0x4D0000, 0x4C000C, 0x4B0800,
		0x4A0000, 0x49003F, 0x480001, 0x470081, 0x46C350, 0x450000, 0x4403E8,
		0x430000, 0x4201F4, 0x410000, 0x401388, 0x3F0000, 0x3E0322, 0x3D00A8,
		0x3C0000, 0x3B0001, 0x3A9001, 0x390020, 0x380000, 0x370000, 0x360000,
		0x350000, 0x340820, 0x330080, 0x320000, 0x314180, 0x300300, 0x2F0300,
		0x2E07FC, 0x2DC0DF, 0x2C1FA3, 0x2B0064, 0x2A0000, 0x290000, 0x280000,
		0x2703E8, 0x260000, 0x250404, 0x240034, 0x230004, 0x220000, 0x211E21,
		0x200393, 0x1F43EC, 0x1E318C, 0x1D318C, 0x1C0488, 0x1B0002, 0x1A0DB0,
		0x190C2B, 0x18071A, 0x17007C, 0x160001, 0x150401, 0x14E048, 0x1327B7,
		0x120064, 0x11012C, 0x100080, 0x0F064F, 0x0E1E70, 0x0D4000, 0x0C5001,
		0x0B0018, 0x0A10D8, 0x091604, 0x082000, 0x0740B2, 0x06C802, 0x0500C8,
		0x040A43, 0x030642, 0x020500, 0x010808, 0x00251C };

class Lmx2594 {
public:
	// Описание подключённых пинов
	struct Pins {
		GPIO_TypeDef *cs_port;   // CSB
		uint16_t cs_pin;

		GPIO_TypeDef *ce_port;   // CE (chip enable)
		uint16_t ce_pin;
	};

	// Конфигурация входного генератора (OSCin-путь)
	struct RefConfig {
		double f_ref_Hz;     // частота генератора на OSCin, Гц
		bool osc_doubler;  // OSC_2X (удвоитель на входе)
		uint8_t mult;       // MULT: 1 (bypass), 3..7 (програмируемый множитель)
		uint16_t r_pre;       // PLL_R_PRE (предделитель до MULT), >= 1
		uint16_t r;           // PLL_R (последний делитель до PFD), >= 1
	};

	// Конструктор – сохраняем SPI, пины и параметры опорника
	Lmx2594(SPI_HandleTypeDef *spi, const Pins &pins, const RefConfig &ref);

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
	// то же но
	void write_my_reg(uint8_t addr, uint16_t value);
	// Запись массива регистров (например, экспорт из TICS Pro)
	struct Reg {
		uint8_t addr;
		uint16_t value;
	};
	HAL_StatusTypeDef writeRegs(const Reg *table, size_t count);
	// массив регистров из терминальной программы TI
	HAL_StatusTypeDef initFromTics();
	HAL_StatusTypeDef initFromTics_5210();



	// Немного телеметрии
	double getFoutHz() const {
		return _f_out_Hz;
	}
	double getFvcoHz() const {
		return _f_vco_Hz;
	}
	double getFpdHz() const {
		return _f_pd_Hz;
	}

private:
	SPI_HandleTypeDef *_spi;
	Pins _pins;
	RefConfig _ref;

	double _f_pd_Hz = 0.0;   // частота фазового детектора
	double _f_vco_Hz = 0.0;   // частота VCO
	double _f_out_Hz = 0.0;   // фактическая выходная частота
	uint32_t _pll_den = 1u << 24; // используем DEN = 2^24 (разумный компромисс)

	struct FreqPlan {
		double f_vco;
		double f_out;
		uint8_t chdiv_code;   // код CHDIV (R75[10:6]) согласно даташиту
		bool use_chdiv; // true = используем делитель канала, false = VCO напрямую
		uint32_t pll_n;
		uint32_t pll_num;
		uint32_t pll_den;
		uint8_t mash_order;   // 0 – integer, 1..4 – порядок MASH
		uint8_t pfd_dly_sel;  // PFD_DLY_SEL (R37[13:8])
		bool integer_mode;
	};

	// Расчёт плана частоты (VCO, N, NUM/DEN, CHDIV и прочее)
	bool computeFreqPlan(double fout_Hz, FreqPlan &plan);

	// Запись регистров под уже рассчитанный план частоты (без FCAL_EN)
	HAL_StatusTypeDef applyFreqPlan(const FreqPlan &plan);

	// Запуск VCO calibration (R0.FCAL_EN=1) с корректными FCAL_* полями
	HAL_StatusTypeDef runFcal(const FreqPlan &plan);

	// Вспомогательные штуки для SPI
	void csLow();
	void csHigh();
};

#endif /* LIBRARY_LMX2594_H_ */

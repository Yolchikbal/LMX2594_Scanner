#include "lmx2594.h"
#include <cmath>

Lmx2594::Lmx2594(SPI_HandleTypeDef* spi, const Pins& pins, const RefConfig& ref)
    : _spi(spi), _pins(pins), _ref(ref)
{
    // Минимальные sanity-check-и на параметры опорника
    if (_ref.r_pre == 0) _ref.r_pre = 1;
    if (_ref.r     == 0) _ref.r     = 1;
    if (_ref.mult  == 0) _ref.mult  = 1;
}

// --- низкоуровневые helpers ---

void Lmx2594::csLow()
{
    HAL_GPIO_WritePin(_pins.cs_port, _pins.cs_pin, GPIO_PIN_RESET);
}

void Lmx2594::csHigh()
{
    HAL_GPIO_WritePin(_pins.cs_port, _pins.cs_pin, GPIO_PIN_SET);
}

void Lmx2594::powerDownHard()
{
    HAL_GPIO_WritePin(_pins.ce_port, _pins.ce_pin, GPIO_PIN_RESET);
}

void Lmx2594::powerUpHard()
{
    HAL_GPIO_WritePin(_pins.ce_port, _pins.ce_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef Lmx2594::writeReg(uint8_t addr, uint16_t value)
{
    // Формат: [R/W + A6..A0][D15..D8][D7..D0], всегда R/W=0 (запись)
    uint8_t buf[3];
    buf[0] = addr & 0x7F; // R/W = 0
    buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>( value       & 0xFF);

    csLow();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(_spi, buf, sizeof(buf), HAL_MAX_DELAY);
    csHigh();

    return st;
}

HAL_StatusTypeDef Lmx2594::writeRegs(const Reg* table, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        HAL_StatusTypeDef st = writeReg(table[i].addr, table[i].value);
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

// --- базовая инициализация, без установки частоты ---

HAL_StatusTypeDef Lmx2594::init()
{
    // Поднять CE – включить микросхему
    powerUpHard();
    HAL_Delay(1);

    // Программный reset (R0.RESET = 1, потом 0).
    // Здесь не заморачиваемся резервными битами – ставим только RESET.
    uint16_t r0 = 0;
    r0 |= (1u << 1);    // RESET=1
    HAL_StatusTypeDef st = writeReg(0, r0);
    if (st != HAL_OK) return st;
    HAL_Delay(1);

    r0 &= ~(1u << 1);   // RESET=0
    st = writeReg(0, r0);
    if (st != HAL_OK) return st;




    // --- после RESET=1→0 ---

    // R1 – CAL_CLK_DIV: для fOSC=100 МГц нужно 0 (fOSC ≤ 200 МГц)
    {
        uint16_t r1 = 0;
        // CAL_CLK_DIV = 0 (бит[2:0]) – остальные 0
        HAL_StatusTypeDef st = writeReg(1, r1);
        if (st != HAL_OK) return st;
    }

    // R58 – INPIN_IGNORE уже по reset =1, можно явно продублировать
    {
        uint16_t r58 = 0;
        r58 |= (1u << 15);   // INPIN_IGNORE = 1
        HAL_StatusTypeDef st = writeReg(58, r58);
        if (st != HAL_OK) return st;
    }

    // R59 – LD_TYPE: 1 = по Vtune (Lock Detect по напряжению настройки VCO)
    {
        uint16_t r59 = 0;
        r59 |= 1u;           // LD_TYPE = 1
        HAL_StatusTypeDef st = writeReg(59, r59);
        if (st != HAL_OK) return st;
    }

    // R60 – LD_DLY: задержка перед утверждением LD (цикл стейт‑машины)
    {
        uint16_t r60 = 1000; // любое разумное значение, TI часто ставит около 1000
        HAL_StatusTypeDef st = writeReg(60, r60);
        if (st != HAL_OK) return st;
    }









    // Настройка входного тракта (OSCin-путь)
    // R9 – OSC_2X
    uint16_t r9 = 0;
    if (_ref.osc_doubler) {
        // R9[12] = OSC_2X
        r9 |= (1u << 12);
    }
    st = writeReg(9, r9);
    if (st != HAL_OK) return st;

    // R10 – MULT[11:7]
    uint16_t r10 = 0;
    {
        uint8_t mult = _ref.mult;
        if (mult < 1) mult = 1;
        // MULT=1 — bypass
        r10 |= (static_cast<uint16_t>(mult) & 0x1F) << 7;
    }
    st = writeReg(10, r10);
    if (st != HAL_OK) return st;

    // R11 – PLL_R[11:4]
    uint16_t r11 = 0;
    r11 |= (static_cast<uint16_t>(_ref.r) & 0xFF) << 4;
    st = writeReg(11, r11);
    if (st != HAL_OK) return st;

    // R12 – PLL_R_PRE[11:0]
    uint16_t r12 = 0;
    r12 |= static_cast<uint16_t>(_ref.r_pre) & 0x0FFF;
    st = writeReg(12, r12);
    if (st != HAL_OK) return st;

    // R14 – ток зарядового насоса, CPG=7 (15 мА) по даташиту
    uint16_t r14 = 0;
    r14 |= (7u << 4); // CPG[6:4]
    st = writeReg(14, r14);
    if (st != HAL_OK) return st;

    // R31 – пока CHDIV_DIV2=0, сам CHDIV настроим позже
    uint16_t r31 = 0;
    st = writeReg(31, r31);
    if (st != HAL_OK) return st;

    // R44 – включаем OUTA, выключаем OUTB, MASH пока 0-го порядка (integer)
    uint16_t r44 = 0;
    r44 |= (31u << 8);  // OUTA_PWR = 31
    r44 |= (1u << 7);   // OUTB_PD = 1 (выкл)
    // OUTA_PD=0
    r44 |= (1u << 5);   // MASH_RESET_N = 1
    // MASH_ORDER=0 (integer)
    st = writeReg(44, r44);
    if (st != HAL_OK) return st;

    // R45 – OUTA_MUX = VCO, OUTB_PWR не используем
    uint16_t r45 = 0;
    r45 |= (1u << 11);  // OUTA_MUX[12:11] = 01 => VCO
    st = writeReg(45, r45);
    if (st != HAL_OK) return st;

    // R46 – OUTB_MUX = Hi-Z (3)
    uint16_t r46 = 0;
    r46 |= 3u;          // OUTB_MUX[1:0] = 3
    st = writeReg(46, r46);
    if (st != HAL_OK) return st;

    // Посчитаем fPD для дальнейших расчётов
    double fosc = _ref.f_ref_Hz;
    double mult_factor = (_ref.osc_doubler ? 2.0 : 1.0) *
                         (_ref.mult ? _ref.mult : 1);
    _f_pd_Hz = fosc * mult_factor /
               (static_cast<double>(_ref.r_pre) * _ref.r);

    return HAL_OK;
}

// Инициализация + сразу выставить частоту
HAL_StatusTypeDef Lmx2594::init(double fout_Hz)
{
    HAL_StatusTypeDef st = init();
    if (st != HAL_OK) return st;
    return setFrequency(fout_Hz);
}

// --- расчёт частотного плана (VCO, N, NUM/DEN, CHDIV) ---

bool Lmx2594::computeFreqPlan(double fout_Hz, FreqPlan& plan)
{
    if (fout_Hz <= 0.0) return false;

    // Пересчёт fPD на всякий случай
    double fosc = _ref.f_ref_Hz;
    double mult_factor = (_ref.osc_doubler ? 2.0 : 1.0) *
                         (_ref.mult ? _ref.mult : 1);
    _f_pd_Hz = fosc * mult_factor /
               (static_cast<double>(_ref.r_pre) * _ref.r);

    if (_f_pd_Hz <= 0.0) return false;

    // Для frac-N по даташиту: 5 МГц ≤ fPD ≤ 300 МГц :contentReference[oaicite:1]{index=1}
    if (_f_pd_Hz < 5e6 || _f_pd_Hz > 300e6) {
        return false;
    }

    // Возможные делители канала (CHDIV – см. таблицу 8/32) :contentReference[oaicite:2]{index=2}
    static const uint16_t chdiv_values[] = {
        1, // особый случай – bypass (OUTA_MUX=VCO)
        2, 4, 6, 8, 12, 16, 24, 32,
        48, 64, 72, 96, 128, 192,
        256, 384, 512, 768
    };

    constexpr double vco_min = 7.5e9;
    constexpr double vco_max = 15.0e9;
    constexpr double max_err_hz = 50.0; // допустимая ошибка по частоте

    for (uint16_t chdiv : chdiv_values) {
        double f_vco = fout_Hz * chdiv;
        if (chdiv == 1) {
            // bypass делителя
            f_vco = fout_Hz;
        }

        if (f_vco < vco_min || f_vco > vco_max) continue;

        // При fVCO > 10 ГГц – максимальный CHDIV = 6 :contentReference[oaicite:3]{index=3}
        if (f_vco > 10e9 && chdiv > 6) continue;

        double n = f_vco / _f_pd_Hz;
        if (n < 1.0) continue;

        uint32_t pll_n = static_cast<uint32_t>(std::floor(n));
        double frac    = n - static_cast<double>(pll_n);

        uint32_t pll_den = _pll_den;
        uint32_t pll_num = static_cast<uint32_t>(
                std::llround(frac * static_cast<double>(pll_den)));

        bool integer_mode = false;
        if (pll_num == 0u || pll_num == pll_den) {
            integer_mode = true;
            pll_num = 0u;
            if (pll_den == pll_num) {
                ++pll_n;
            }
        }

        uint8_t mash_order;
        uint8_t pfd_dly_sel;

        // Ограничения на минимальный N и PFD_DLY_SEL – таблица 2 :contentReference[oaicite:4]{index=4}
        if (integer_mode) {
            mash_order = 0;
            if (f_vco <= 12.5e9) {
                if (pll_n < 28) continue;
                pfd_dly_sel = 1;
            } else {
                if (pll_n < 32) continue;
                pfd_dly_sel = 2;
            }
        } else {
            mash_order = 3; // обычно хватает MASH 3-го порядка
            if (f_vco <= 10e9) {
                if (pll_n < 36) continue;
                pfd_dly_sel = 3;
            } else {
                if (pll_n < 40) continue;
                pfd_dly_sel = 4;
            }
        }

        // Реальная частота с учётом квантования
        double n_real = static_cast<double>(pll_n) +
                        static_cast<double>(pll_num) / pll_den;
        double f_out_real = n_real * _f_pd_Hz /
                            static_cast<double>(chdiv);

        if (std::fabs(f_out_real - fout_Hz) > max_err_hz) {
            continue;
        }

        plan.f_vco        = f_vco;
        plan.f_out        = f_out_real;
        plan.pll_n        = pll_n;
        plan.pll_num      = pll_num;
        plan.pll_den      = pll_den;
        plan.mash_order   = mash_order;
        plan.pfd_dly_sel  = pfd_dly_sel;
        plan.integer_mode = integer_mode;

        if (chdiv == 1) {
            plan.use_chdiv  = false;
            plan.chdiv_code = 0;
        } else {
            plan.use_chdiv  = true;
            switch (chdiv) {
            case 2:   plan.chdiv_code = 0;  break;
            case 4:   plan.chdiv_code = 1;  break;
            case 6:   plan.chdiv_code = 2;  break;
            case 8:   plan.chdiv_code = 3;  break;
            case 12:  plan.chdiv_code = 4;  break;
            case 16:  plan.chdiv_code = 5;  break;
            case 24:  plan.chdiv_code = 6;  break;
            case 32:  plan.chdiv_code = 7;  break;
            case 48:  plan.chdiv_code = 8;  break;
            case 64:  plan.chdiv_code = 9;  break;
            case 72:  plan.chdiv_code = 10; break;
            case 96:  plan.chdiv_code = 11; break;
            case 128: plan.chdiv_code = 12; break;
            case 192: plan.chdiv_code = 13; break;
            case 256: plan.chdiv_code = 14; break;
            case 384: plan.chdiv_code = 15; break;
            case 512: plan.chdiv_code = 16; break;
            case 768: plan.chdiv_code = 17; break;
            default:  continue;
            }
        }

        return true;
    }

    return false; // ничего подходящего не нашли
}

// --- выставление частоты ---

HAL_StatusTypeDef Lmx2594::setFrequency(double fout_Hz)
{
    FreqPlan plan{};
    if (!computeFreqPlan(fout_Hz, plan)) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef st = applyFreqPlan(plan);
    if (st != HAL_OK) return st;

    st = runFcal(plan);
    if (st != HAL_OK) return st;

    _f_vco_Hz = plan.f_vco;
    _f_out_Hz = plan.f_out;

    return HAL_OK;
}

// Запись регистров под частотный план (без FCAL_EN)

HAL_StatusTypeDef Lmx2594::applyFreqPlan(const FreqPlan& plan)
{
    HAL_StatusTypeDef st;

    // PLL_N – R34 (старшие 3 бита) + R36 (младшие 16)
    uint16_t r34 = 0;
    r34 |= static_cast<uint16_t>((plan.pll_n >> 16) & 0x7u); // PLL_N[18:16]
    st = writeReg(34, r34);
    if (st != HAL_OK) return st;

    uint16_t r36 = static_cast<uint16_t>(plan.pll_n & 0xFFFFu);
    st = writeReg(36, r36);
    if (st != HAL_OK) return st;

    // PLL_DEN – R38..R39
    uint16_t r38 = static_cast<uint16_t>((plan.pll_den >> 16) & 0xFFFFu);
    uint16_t r39 = static_cast<uint16_t>( plan.pll_den        & 0xFFFFu);
    st = writeReg(38, r38);
    if (st != HAL_OK) return st;
    st = writeReg(39, r39);
    if (st != HAL_OK) return st;

    // PLL_NUM – R42..R43
    uint16_t r42 = static_cast<uint16_t>((plan.pll_num >> 16) & 0xFFFFu);
    uint16_t r43 = static_cast<uint16_t>( plan.pll_num        & 0xFFFFu);
    st = writeReg(42, r42);
    if (st != HAL_OK) return st;
    st = writeReg(43, r43);
    if (st != HAL_OK) return st;

    // PFD_DLY_SEL – R37[13:8]
    uint16_t r37 = 0;
    r37 |= static_cast<uint16_t>(plan.pfd_dly_sel & 0x3Fu) << 8;
    st = writeReg(37, r37);
    if (st != HAL_OK) return st;

    // MASH_ORDER, OUTA/OUTB_PD, OUTA_PWR – R44
    uint16_t r44 = 0;
    r44 |= (31u << 8);                  // OUTA_PWR
    r44 |= (1u << 7);                   // OUTB_PD=1
    r44 |= (1u << 5);                   // MASH_RESET_N=1
    r44 |= (plan.mash_order & 0x7u);    // MASH_ORDER[2:0]
    st = writeReg(44, r44);
    if (st != HAL_OK) return st;

    // OUTA_MUX, OUT_ISET, OUTB_PWR – R45
    uint16_t r45 = 0;
    if (!plan.use_chdiv) {
        // VCO напрямую на выход: OUTA_MUX=1
        r45 |= (1u << 11);
    } else {
        // OUTA_MUX=0 – канал с делителем
    }
    st = writeReg(45, r45);
    if (st != HAL_OK) return st;

    // OUTB_MUX – R46 (оставляем Hi-Z)
    uint16_t r46 = 0;
    r46 |= 3u; // Hi-Z
    st = writeReg(46, r46);
    if (st != HAL_OK) return st;

    // CHDIV – R31 (CHDIV_DIV2) и R75 (CHDIV код)
    if (plan.use_chdiv) {
        uint16_t r31 = 0;
        // CHDIV_DIV2=1 для всех делителей > 2
        uint16_t chdiv_val;
        switch (plan.chdiv_code) {
        case 0:  chdiv_val = 2;   break;
        case 1:  chdiv_val = 4;   break;
        case 2:  chdiv_val = 6;   break;
        case 3:  chdiv_val = 8;   break;
        case 4:  chdiv_val = 12;  break;
        case 5:  chdiv_val = 16;  break;
        case 6:  chdiv_val = 24;  break;
        case 7:  chdiv_val = 32;  break;
        case 8:  chdiv_val = 48;  break;
        case 9:  chdiv_val = 64;  break;
        case 10: chdiv_val = 72;  break;
        case 11: chdiv_val = 96;  break;
        case 12: chdiv_val = 128; break;
        case 13: chdiv_val = 192; break;
        case 14: chdiv_val = 256; break;
        case 15: chdiv_val = 384; break;
        case 16: chdiv_val = 512; break;
        case 17: chdiv_val = 768; break;
        default: chdiv_val = 2;   break;
        }
        if (chdiv_val > 2) {
            r31 |= (1u << 14); // CHDIV_DIV2
        }
        st = writeReg(31, r31);
        if (st != HAL_OK) return st;

        uint16_t r75 = 0;
        r75 |= static_cast<uint16_t>(plan.chdiv_code & 0x1Fu) << 6; // CHDIV[10:6]
        st = writeReg(75, r75);
        if (st != HAL_OK) return st;
    } else {
        // Делитель не используем
        uint16_t r31 = 0;
        st = writeReg(31, r31);
        if (st != HAL_OK) return st;
        uint16_t r75 = 0;
        st = writeReg(75, r75);
        if (st != HAL_OK) return st;
    }

    return HAL_OK;
}

// --- запуск VCO calibration (FCAL_EN) ---

HAL_StatusTypeDef Lmx2594::runFcal(const FreqPlan& /*plan*/)
{
    // Подбор FCAL_HPFD_ADJ / FCAL_LPFD_ADJ по fPD (см. описание R0) :contentReference[oaicite:5]{index=5}

    uint8_t hpf = 0;
    if      (_f_pd_Hz <= 100e6) hpf = 0;
    else if (_f_pd_Hz <= 150e6) hpf = 1;
    else if (_f_pd_Hz <= 200e6) hpf = 2;
    else                        hpf = 3;

    uint8_t lpf = 0;
    if      (_f_pd_Hz >= 10e6)  lpf = 0;
    else if (_f_pd_Hz >= 5e6)   lpf = 1;
    else if (_f_pd_Hz >= 2.5e6) lpf = 2;
    else                        lpf = 3;

    uint16_t r0 = 0;
    r0 |= (1u << 13);
    r0 |= (1u << 10);
    r0 |= (1u << 4);

    r0 |= (static_cast<uint16_t>(hpf & 0x3u) << 7);
    r0 |= (static_cast<uint16_t>(lpf & 0x3u) << 5);

    r0 |= (1u << 3); // FCAL_EN=1
    r0 |= (1u << 2); // MUXOUT_LD_SEL=1 (LD на MUXout)
    // RESET=0, POWERDOWN=0

    HAL_StatusTypeDef st = writeReg(0, r0);
    if (st != HAL_OK) return st;

    HAL_Delay(1);
    return HAL_OK;
}

HAL_StatusTypeDef Lmx2594::initFromTics()
{
    powerUpHard();          // EN_LO = 1
    HAL_Delay(2);

    for (size_t i = 0; i < 113; ++i) {
        uint32_t w = lmx_5200_regs[i];
        uint8_t buf[3] = {
            static_cast<uint8_t>((w >> 16) & 0xFF),
            static_cast<uint8_t>((w >>  8) & 0xFF),
            static_cast<uint8_t>( w        & 0xFF)
        };

        csLow();
        HAL_StatusTypeDef st = HAL_SPI_Transmit(_spi, buf, 3, HAL_MAX_DELAY);
        csHigh();
        if (st != HAL_OK) return st;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Lmx2594::initFromTics_5210()
{
    powerUpHard();          // EN_LO = 1
    HAL_Delay(2);

    for (size_t i = 0; i < 113; ++i) {
        uint32_t w = lmx_5210_regs[i];
        uint8_t buf[3] = {
            static_cast<uint8_t>((w >> 16) & 0xFF),
            static_cast<uint8_t>((w >>  8) & 0xFF),
            static_cast<uint8_t>( w        & 0xFF)
        };

        csLow();
        HAL_StatusTypeDef st = HAL_SPI_Transmit(_spi, buf, 3, HAL_MAX_DELAY);
        csHigh();
        if (st != HAL_OK) return st;
    }

    return HAL_OK;
}

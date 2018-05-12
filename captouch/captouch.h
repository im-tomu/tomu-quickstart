#include <stdint.h>

/** Capsense initialization structure. */
typedef struct
{
  /** Full bias current. See the ACMP chapter about bias and response time in
   *  the reference manual for details. */
  bool                          fullBias;

#if defined(_ACMP_CTRL_HALFBIAS_MASK)
  /** Half bias current. See the ACMP chapter about bias and response time in
   *  the reference manual for details. */
  bool                          halfBias;
#endif

  /** Bias current. See the ACMP chapter about bias and response time in the
   *  reference manual for details. */
  uint32_t                      biasProg;

#if defined(_ACMP_CTRL_WARMTIME_MASK)
  /** Warmup time. This is measured in HFPERCLK cycles and should be
   *  about 10us in wall clock time. */
  ACMP_WarmTime_TypeDef         warmTime;
#endif

#if defined(_ACMP_CTRL_HYSTSEL_MASK)
  /** Hysteresis level */
  ACMP_HysteresisLevel_TypeDef  hysteresisLevel;
#else
  /** Hysteresis level when ACMP output is 0 */
  ACMP_HysteresisLevel_TypeDef  hysteresisLevel_0;

  /** Hysteresis level when ACMP output is 1 */
  ACMP_HysteresisLevel_TypeDef  hysteresisLevel_1;
#endif

  /** Resistor used in the capacative sensing circuit. For values see
   *  your device datasheet. */
  ACMP_CapsenseResistor_TypeDef resistor;

#if defined(_ACMP_INPUTSEL_LPREF_MASK)
  /** Low power reference enabled. This setting, if enabled, reduces the
   *  power used by the VDD and bandgap references. */
  bool                          lowPowerReferenceEnabled;
#endif

#if defined(_ACMP_INPUTSEL_VDDLEVEL_MASK)
  /** Vdd reference value. VDD_SCALED = (Vdd * VDDLEVEL) / 63.
   *  Valid values are in the range 0-63. */
  uint32_t                      vddLevel;
#else
  /**
   * This value configures the upper voltage threshold of the capsense
   * oscillation rail.
   *
   * The voltage threshold is calculated as
   *   Vdd * (vddLevelHigh + 1) / 64
   */
  uint32_t                      vddLevelHigh;

  /**
   * This value configures the lower voltage threshold of the capsense
   * oscillation rail.
   *
   * The voltage threshold is calculated as
   *   Vdd * (vddLevelLow + 1) / 64
   */
  uint32_t                      vddLevelLow;
#endif

  /** If true, ACMP is being enabled after configuration. */
  bool                          enable;
} ACMP_CapsenseInit_TypeDef;
#include <stdint.h>

/* Bit fields for ACMP CTRL */
#define _ACMP_CTRL_RESETVALUE              0x47000000UL                         /**< Default value for ACMP_CTRL */
#define _ACMP_CTRL_MASK                    0xCF03077FUL                         /**< Mask for ACMP_CTRL */

#define _ACMP_CTRL_EN_SHIFT                0                                    /**< Shift value for ACMP_EN */
#define _ACMP_CTRL_EN_MASK                 0x1UL                                /**< Bit mask for ACMP_EN */
#define _ACMP_CTRL_EN_DEFAULT              0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */

#define _ACMP_CTRL_MUXEN_SHIFT             1                                    /**< Shift value for ACMP_MUXEN */
#define _ACMP_CTRL_MUXEN_MASK              0x2UL                                /**< Bit mask for ACMP_MUXEN */
#define _ACMP_CTRL_MUXEN_DEFAULT           0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */

#define _ACMP_CTRL_INACTVAL_SHIFT          2                                    /**< Shift value for ACMP_INACTVAL */
#define _ACMP_CTRL_INACTVAL_MASK           0x4UL                                /**< Bit mask for ACMP_INACTVAL */
#define _ACMP_CTRL_INACTVAL_DEFAULT        0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */
#define _ACMP_CTRL_INACTVAL_LOW            0x00000000UL                         /**< Mode LOW for ACMP_CTRL */
#define _ACMP_CTRL_INACTVAL_HIGH           0x00000001UL                         /**< Mode HIGH for ACMP_CTRL */

#define _ACMP_CTRL_GPIOINV_SHIFT           3                                    /**< Shift value for ACMP_GPIOINV */
#define _ACMP_CTRL_GPIOINV_MASK            0x8UL                                /**< Bit mask for ACMP_GPIOINV */
#define _ACMP_CTRL_GPIOINV_DEFAULT         0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */
#define _ACMP_CTRL_GPIOINV_NOTINV          0x00000000UL                         /**< Mode NOTINV for ACMP_CTRL */
#define _ACMP_CTRL_GPIOINV_INV             0x00000001UL                         /**< Mode INV for ACMP_CTRL */

#define _ACMP_CTRL_HYSTSEL_SHIFT           4                                    /**< Shift value for ACMP_HYSTSEL */
#define _ACMP_CTRL_HYSTSEL_MASK            0x70UL                               /**< Bit mask for ACMP_HYSTSEL */
#define _ACMP_CTRL_HYSTSEL_DEFAULT         0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST0           0x00000000UL                         /**< Mode HYST0 for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST1           0x00000001UL                         /**< Mode HYST1 for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST2           0x00000002UL                         /**< Mode HYST2 for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST3           0x00000003UL                         /**< Mode HYST3 for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST4           0x00000004UL                         /**< Mode HYST4 for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST5           0x00000005UL                         /**< Mode HYST5 for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST6           0x00000006UL                         /**< Mode HYST6 for ACMP_CTRL */
#define _ACMP_CTRL_HYSTSEL_HYST7           0x00000007UL                         /**< Mode HYST7 for ACMP_CTRL */

#define _ACMP_CTRL_WARMTIME_SHIFT          8                                    /**< Shift value for ACMP_WARMTIME */
#define _ACMP_CTRL_WARMTIME_MASK           0x700UL                              /**< Bit mask for ACMP_WARMTIME */
#define _ACMP_CTRL_WARMTIME_DEFAULT        0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_4CYCLES        0x00000000UL                         /**< Mode 4CYCLES for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_8CYCLES        0x00000001UL                         /**< Mode 8CYCLES for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_16CYCLES       0x00000002UL                         /**< Mode 16CYCLES for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_32CYCLES       0x00000003UL                         /**< Mode 32CYCLES for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_64CYCLES       0x00000004UL                         /**< Mode 64CYCLES for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_128CYCLES      0x00000005UL                         /**< Mode 128CYCLES for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_256CYCLES      0x00000006UL                         /**< Mode 256CYCLES for ACMP_CTRL */
#define _ACMP_CTRL_WARMTIME_512CYCLES      0x00000007UL                         /**< Mode 512CYCLES for ACMP_CTRL */

#define _ACMP_CTRL_IRISE_SHIFT             16                                   /**< Shift value for ACMP_IRISE */
#define _ACMP_CTRL_IRISE_MASK              0x10000UL                            /**< Bit mask for ACMP_IRISE */
#define _ACMP_CTRL_IRISE_DEFAULT           0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */
#define _ACMP_CTRL_IRISE_DISABLED          0x00000000UL                         /**< Mode DISABLED for ACMP_CTRL */
#define _ACMP_CTRL_IRISE_ENABLED           0x00000001UL                         /**< Mode ENABLED for ACMP_CTRL */

#define _ACMP_CTRL_IFALL_SHIFT             17                                   /**< Shift value for ACMP_IFALL */
#define _ACMP_CTRL_IFALL_MASK              0x20000UL                            /**< Bit mask for ACMP_IFALL */
#define _ACMP_CTRL_IFALL_DEFAULT           0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */
#define _ACMP_CTRL_IFALL_DISABLED          0x00000000UL                         /**< Mode DISABLED for ACMP_CTRL */
#define _ACMP_CTRL_IFALL_ENABLED           0x00000001UL                         /**< Mode ENABLED for ACMP_CTRL */

#define _ACMP_CTRL_BIASPROG_SHIFT          24                                   /**< Shift value for ACMP_BIASPROG */
#define _ACMP_CTRL_BIASPROG_MASK           0xF000000UL                          /**< Bit mask for ACMP_BIASPROG */
#define _ACMP_CTRL_BIASPROG_DEFAULT        0x00000007UL                         /**< Mode DEFAULT for ACMP_CTRL */

#define _ACMP_CTRL_HALFBIAS_SHIFT          30                                   /**< Shift value for ACMP_HALFBIAS */
#define _ACMP_CTRL_HALFBIAS_MASK           0x40000000UL                         /**< Bit mask for ACMP_HALFBIAS */
#define _ACMP_CTRL_HALFBIAS_DEFAULT        0x00000001UL                         /**< Mode DEFAULT for ACMP_CTRL */

#define _ACMP_CTRL_FULLBIAS_SHIFT          31                                   /**< Shift value for ACMP_FULLBIAS */
#define _ACMP_CTRL_FULLBIAS_MASK           0x80000000UL                         /**< Bit mask for ACMP_FULLBIAS */
#define _ACMP_CTRL_FULLBIAS_DEFAULT        0x00000000UL                         /**< Mode DEFAULT for ACMP_CTRL */



/* Bit fields for ACMP INPUTSEL */
#define _ACMP_INPUTSEL_RESETVALUE          0x00010080UL                            /**< Default value for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_MASK                0x31013FF7UL                            /**< Mask for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_SHIFT        0                                       /**< Shift value for ACMP_POSSEL */
#define _ACMP_INPUTSEL_POSSEL_MASK         0x7UL                                   /**< Bit mask for ACMP_POSSEL */
#define _ACMP_INPUTSEL_POSSEL_DEFAULT      0x00000000UL                            /**< Mode DEFAULT for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH0          0x00000000UL                            /**< Mode CH0 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH1          0x00000001UL                            /**< Mode CH1 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH2          0x00000002UL                            /**< Mode CH2 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH3          0x00000003UL                            /**< Mode CH3 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH4          0x00000004UL                            /**< Mode CH4 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH5          0x00000005UL                            /**< Mode CH5 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH6          0x00000006UL                            /**< Mode CH6 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_POSSEL_CH7          0x00000007UL                            /**< Mode CH7 for ACMP_INPUTSEL */

#define _ACMP_INPUTSEL_NEGSEL_SHIFT        4                                       /**< Shift value for ACMP_NEGSEL */
#define _ACMP_INPUTSEL_NEGSEL_MASK         0xF0UL                                  /**< Bit mask for ACMP_NEGSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH0          0x00000000UL                            /**< Mode CH0 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH1          0x00000001UL                            /**< Mode CH1 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH2          0x00000002UL                            /**< Mode CH2 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH3          0x00000003UL                            /**< Mode CH3 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH4          0x00000004UL                            /**< Mode CH4 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH5          0x00000005UL                            /**< Mode CH5 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH6          0x00000006UL                            /**< Mode CH6 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CH7          0x00000007UL                            /**< Mode CH7 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_DEFAULT      0x00000008UL                            /**< Mode DEFAULT for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_1V25         0x00000008UL                            /**< Mode 1V25 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_2V5          0x00000009UL                            /**< Mode 2V5 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_VDD          0x0000000AUL                            /**< Mode VDD for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_NEGSEL_CAPSENSE     0x0000000BUL                            /**< Mode CAPSENSE for ACMP_INPUTSEL */

#define _ACMP_INPUTSEL_VDDLEVEL_SHIFT      8                                       /**< Shift value for ACMP_VDDLEVEL */
#define _ACMP_INPUTSEL_VDDLEVEL_MASK       0x3F00UL                                /**< Bit mask for ACMP_VDDLEVEL */
#define _ACMP_INPUTSEL_VDDLEVEL_DEFAULT    0x00000000UL                            /**< Mode DEFAULT for ACMP_INPUTSEL */

#define _ACMP_INPUTSEL_LPREF_SHIFT         16                                      /**< Shift value for ACMP_LPREF */
#define _ACMP_INPUTSEL_LPREF_MASK          0x10000UL                               /**< Bit mask for ACMP_LPREF */
#define _ACMP_INPUTSEL_LPREF_DEFAULT       0x00000001UL                            /**< Mode DEFAULT for ACMP_INPUTSEL */

#define _ACMP_INPUTSEL_CSRESEN_SHIFT       24                                      /**< Shift value for ACMP_CSRESEN */
#define _ACMP_INPUTSEL_CSRESEN_MASK        0x1000000UL                             /**< Bit mask for ACMP_CSRESEN */
#define _ACMP_INPUTSEL_CSRESEN_DEFAULT     0x00000000UL                            /**< Mode DEFAULT for ACMP_INPUTSEL */

#define _ACMP_INPUTSEL_CSRESSEL_SHIFT      28                                      /**< Shift value for ACMP_CSRESSEL */
#define _ACMP_INPUTSEL_CSRESSEL_MASK       0x30000000UL                            /**< Bit mask for ACMP_CSRESSEL */
#define _ACMP_INPUTSEL_CSRESSEL_DEFAULT    0x00000000UL                            /**< Mode DEFAULT for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_CSRESSEL_RES0       0x00000000UL                            /**< Mode RES0 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_CSRESSEL_RES1       0x00000001UL                            /**< Mode RES1 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_CSRESSEL_RES2       0x00000002UL                            /**< Mode RES2 for ACMP_INPUTSEL */
#define _ACMP_INPUTSEL_CSRESSEL_RES3       0x00000003UL                            /**< Mode RES3 for ACMP_INPUTSEL */

/** ACMP inputs. Note that scaled VDD and bandgap references can only be used
 *  as negative inputs. */
typedef enum
{
  /** Channel 0 */
  acmpChannel0    = _ACMP_INPUTSEL_NEGSEL_CH0,
  /** Channel 1 */
  acmpChannel1    = _ACMP_INPUTSEL_NEGSEL_CH1,
  /** Channel 2 */
  acmpChannel2    = _ACMP_INPUTSEL_NEGSEL_CH2,
  /** Channel 3 */
  acmpChannel3    = _ACMP_INPUTSEL_NEGSEL_CH3,
  /** Channel 4 */
  acmpChannel4    = _ACMP_INPUTSEL_NEGSEL_CH4,
  /** Channel 5 */
  acmpChannel5    = _ACMP_INPUTSEL_NEGSEL_CH5,
  /** Channel 6 */
  acmpChannel6    = _ACMP_INPUTSEL_NEGSEL_CH6,
  /** Channel 7 */
  acmpChannel7    = _ACMP_INPUTSEL_NEGSEL_CH7,
  /** 1.25V internal reference */
  acmpChannel1V25 = _ACMP_INPUTSEL_NEGSEL_1V25,
  /** 2.5V internal reference */
  acmpChannel2V5  = _ACMP_INPUTSEL_NEGSEL_2V5,
  /** Scaled VDD reference */
  acmpChannelVDD  = _ACMP_INPUTSEL_NEGSEL_VDD,

  /** Capacitive sense mode */
  acmpChannelCapSense = _ACMP_INPUTSEL_NEGSEL_CAPSENSE,
} ACMP_Channel_TypeDef;

typedef enum
{
  /** 4 HFPERCLK cycles warmup */
  acmpWarmTime4   = _ACMP_CTRL_WARMTIME_4CYCLES,
  /** 8 HFPERCLK cycles warmup */
  acmpWarmTime8   = _ACMP_CTRL_WARMTIME_8CYCLES,
  /** 16 HFPERCLK cycles warmup */
  acmpWarmTime16  = _ACMP_CTRL_WARMTIME_16CYCLES,
  /** 32 HFPERCLK cycles warmup */
  acmpWarmTime32  = _ACMP_CTRL_WARMTIME_32CYCLES,
  /** 64 HFPERCLK cycles warmup */
  acmpWarmTime64  = _ACMP_CTRL_WARMTIME_64CYCLES,
  /** 128 HFPERCLK cycles warmup */
  acmpWarmTime128 = _ACMP_CTRL_WARMTIME_128CYCLES,
  /** 256 HFPERCLK cycles warmup */
  acmpWarmTime256 = _ACMP_CTRL_WARMTIME_256CYCLES,
  /** 512 HFPERCLK cycles warmup */
  acmpWarmTime512 = _ACMP_CTRL_WARMTIME_512CYCLES
} ACMP_WarmTime_TypeDef;


/** Hysteresis level. See datasheet for your device for details on each
 *  level. */
typedef enum
{
  acmpHysteresisLevel0 = _ACMP_CTRL_HYSTSEL_HYST0,       /**< Hysteresis level 0 */
  acmpHysteresisLevel1 = _ACMP_CTRL_HYSTSEL_HYST1,       /**< Hysteresis level 1 */
  acmpHysteresisLevel2 = _ACMP_CTRL_HYSTSEL_HYST2,       /**< Hysteresis level 2 */
  acmpHysteresisLevel3 = _ACMP_CTRL_HYSTSEL_HYST3,       /**< Hysteresis level 3 */
  acmpHysteresisLevel4 = _ACMP_CTRL_HYSTSEL_HYST4,       /**< Hysteresis level 4 */
  acmpHysteresisLevel5 = _ACMP_CTRL_HYSTSEL_HYST5,       /**< Hysteresis level 5 */
  acmpHysteresisLevel6 = _ACMP_CTRL_HYSTSEL_HYST6,       /**< Hysteresis level 6 */
  acmpHysteresisLevel7 = _ACMP_CTRL_HYSTSEL_HYST7        /**< Hysteresis level 7 */
} ACMP_HysteresisLevel_TypeDef;


/** Resistor values used for the internal capacative sense resistor. See the
 *  datasheet for your device for details on each resistor value. */
typedef enum
{
  acmpResistor0 = _ACMP_INPUTSEL_CSRESSEL_RES0,   /**< Resistor value 0 */
  acmpResistor1 = _ACMP_INPUTSEL_CSRESSEL_RES1,   /**< Resistor value 1 */
  acmpResistor2 = _ACMP_INPUTSEL_CSRESSEL_RES2,   /**< Resistor value 2 */
  acmpResistor3 = _ACMP_INPUTSEL_CSRESSEL_RES3,   /**< Resistor value 3 */
} ACMP_CapsenseResistor_TypeDef;

/** Capsense initialization structure. */
struct acmp_capsense_init
{
  /** Full bias current. See the ACMP chapter about bias and response time in
   *  the reference manual for details. */
  bool                          fullBias;

  /** Half bias current. See the ACMP chapter about bias and response time in
   *  the reference manual for details. */
  bool                          halfBias;

  /** Bias current. See the ACMP chapter about bias and response time in the
   *  reference manual for details. */
  uint32_t                      biasProg;

  /** Warmup time. This is measured in HFPERCLK cycles and should be
   *  about 10us in wall clock time. */
  ACMP_WarmTime_TypeDef         warmTime;

  /** Hysteresis level */
  ACMP_HysteresisLevel_TypeDef  hysteresisLevel;

  /** Resistor used in the capacative sensing circuit. For values see
   *  your device datasheet. */
  ACMP_CapsenseResistor_TypeDef resistor;

  /** Low power reference enabled. This setting, if enabled, reduces the
   *  power used by the VDD and bandgap references. */
  bool                          lowPowerReferenceEnabled;

  /** Vdd reference value. VDD_SCALED = (Vdd * VDDLEVEL) / 63.
   *  Valid values are in the range 0-63. */
  uint32_t                      vddLevel;

  /** If true, ACMP is being enabled after configuration. */
  bool                          enable;
};

#define ACMP_CAPSENSE_INIT_DEFAULT                                            \
{                                                                             \
    .fullBias = true,                      /* fullBias */                    \
    .halfBias = false,                      /* halfBias */                    \
    .biasProg = 0x7,                        /* biasProg */                    \
    .warmTime = acmpWarmTime512,            /* 512 cycle warmup to be safe */ \
    .hysteresisLevel = acmpHysteresisLevel1,                                  \
    .resistor = acmpResistor0,                                                \
    .lowPowerReferenceEnabled = false,      /* low power reference */         \
    .vddLevel = 0x3D,                       /* VDD level */                   \
    .enable = true                          /* Enable after init. */          \
}

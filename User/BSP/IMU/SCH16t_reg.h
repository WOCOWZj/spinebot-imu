#ifndef SCH16T_REG_H
#define SCH16T_REG_H

/**
 * SCH1 Filter Configuration
 */
#define SCH1_FILTER_RATE_68HZ        0x00
#define SCH1_FILTER_RATE_30HZ        0x01
#define SCH1_FILTER_RATE_13HZ        0x02
#define SCH1_FILTER_RATE_280HZ       0x03
#define SCH1_FILTER_RATE_370HZ       0x04
#define SCH1_FILTER_RATE_235HZ       0x05
#define SCH1_FILTER_RATE_RESERVED    0x06
#define SCH1_FILTER_RATE_BYPASS      0x07

#define SCH1_FILTER_ACC_68HZ       0x00
#define SCH1_FILTER_ACC_30HZ       0x01
#define SCH1_FILTER_ACC_13HZ       0x02
#define SCH1_FILTER_ACC_240HZ      0x03
#define SCH1_FILTER_ACC_290HZ      0x04
#define SCH1_FILTER_ACC_210HZ      0x05
#define SCH1_FILTER_ACC_RESERVED   0x06
#define SCH1_FILTER_ACC_500HZ      0x07

#define SCH1_RANGE_RATE_DYN_5200DPS   0x01
#define SCH1_RANGE_RATE_DYN_2600DPS   0x03
#define SCH1_RANGE_RATE_DYN_1300DPS   0x04

#define SCH1_LSB_PER_DPS_RATE_5200DPS   (100.0f)
#define SCH1_LSB_PER_DPS_RATE_2600DPS   (200.0f)
#define SCH1_LSB_PER_DPS_RATE_1300DPS   (400.0f)

#define SCH1_RANGE_ACC_12_DYN_16G   0x01
#define SCH1_RANGE_ACC_12_DYN_8G    0x02
#define SCH1_RANGE_ACC_12_DYN_4G    0x03
#define SCH1_RANGE_ACC_12_DYN_2G    0x04

#define SCH1_LSB_PER_MS2_ACC_12_16G   (3200.0f)
#define SCH1_LSB_PER_MS2_ACC_12_8G    (6400.0f)
#define SCH1_LSB_PER_MS2_ACC_12_4G    (12800.0f)
#define SCH1_LSB_PER_MS2_ACC_12_2G    (25600.0f)

#define SCH1_RANGE_ACC_3_DYN_26G    0x00
#define SCH1_RANGE_ACC_3_DYN_16G    0x01
#define SCH1_RANGE_ACC_3_DYN_8G     0x02
#define SCH1_RANGE_ACC_3_DYN_4G     0x03
#define SCH1_RANGE_ACC_3_DYN_2G     0x04

#define SCH1_LSB_PER_MS2_ACC_3_26G   (1600.0f)
#define SCH1_LSB_PER_MS2_ACC_3_16G   (3200.0f)
#define SCH1_LSB_PER_MS2_ACC_3_8G    (6400.0f)
#define SCH1_LSB_PER_MS2_ACC_3_4G    (12800.0f)
#define SCH1_LSB_PER_MS2_ACC_3_2G    (25600.0f)

#define SCH1_DECIMATION_1     0x00
#define SCH1_DECIMATION_2     0x01
#define SCH1_DECIMATION_4     0x02
#define SCH1_DECIMATION_8     0x03
#define SCH1_DECIMATION_16    0x04

#define SCH1_LSB_PER_DEGREE_C   (100.0f)

/**
 * SCH1 Decimation Configuration
 */
#define SCH1_DECIMATION_1       0x00
#define SCH1_DECIMATION_2       0x01
#define SCH1_DECIMATION_4       0x02
#define SCH1_DECIMATION_8       0x03
#define SCH1_DECIMATION_16      0x04
#define SCH1_DECIMATION_32      0x05

/**
 * SCH1 Standard requests
 */
// Rate and acceleration
#define REQ_READ_RATE_X1            0x0048000000AC
#define REQ_READ_RATE_Y1            0x00880000009A
#define REQ_READ_RATE_Z1            0x00C80000006D
#define REQ_READ_ACC_X1             0x0108000000F6
#define REQ_READ_ACC_Y1             0x014800000001
#define REQ_READ_ACC_Z1             0x018800000037
#define REQ_READ_ACC_X3             0x01C8000000C0
#define REQ_READ_ACC_Y3             0x02080000002E
#define REQ_READ_ACC_Z3             0x0248000000D9
#define REQ_READ_RATE_X2            0x0288000000EF
#define REQ_READ_RATE_Y2            0x02C800000018
#define REQ_READ_RATE_Z2            0x030800000083
#define REQ_READ_ACC_X2             0x034800000074
#define REQ_READ_ACC_Y2             0x038800000042
#define REQ_READ_ACC_Z2             0x03C8000000B5

// Status
#define REQ_READ_STAT_SUM           0x05080000001C
#define REQ_READ_STAT_SUM_SAT       0x0548000000EB
#define REQ_READ_STAT_COM           0x0588000000DD
#define REQ_READ_STAT_RATE_COM      0x05C80000002A
#define REQ_READ_STAT_RATE_X        0x0608000000C4
#define REQ_READ_STAT_RATE_Y        0x064800000033
#define REQ_READ_STAT_RATE_Z        0x068800000005
#define REQ_READ_STAT_ACC_X         0x06C8000000F2
#define REQ_READ_STAT_ACC_Y         0x070800000069
#define REQ_READ_STAT_ACC_Z         0x07480000009E

// Temperature and traceability
#define REQ_READ_TEMP               0x0408000000B1
#define REQ_READ_SN_ID1             0x0F4800000065
#define REQ_READ_SN_ID2             0x0F8800000053
#define REQ_READ_SN_ID3             0x0FC8000000A4
#define REQ_READ_COMP_ID            0x0F0800000092

// Filters
#define REQ_READ_FILT_RATE          0x0948000000FA
#define REQ_READ_FILT_ACC12         0x0988000000CC
#define REQ_READ_FILT_ACC3          0x09C80000003B
#define REQ_READ_RATE_CTRL          0x0A08000000D5
#define REQ_READ_ACC12_CTRL         0x0A4800000022
#define REQ_READ_ACC3_CTRL          0x0A8800000014
#define REQ_READ_MODE_CTRL          0x0D4800000010
#define REQ_SET_FILT_RATE           0x0968000000    // For building Rate_XYZ1/2 filter setting frame.
#define REQ_SET_FILT_ACC12          0x09A8000000    // For building Acc_XYZ1/2 filter setting frame.
#define REQ_SET_FILT_ACC3           0x09E8000000    // For building Acc_XYZ3 filter setting frame.

// Sensitivity and decimation
#define REQ_SET_RATE_CTRL           0x0A28000000    // For building Rate_XYZ1/2 sensitivity and
                                                    //     Rate_XYZ2 decimation setting frame.
#define REQ_SET_ACC12_CTRL          0x0A68000000    // For building Acc_XYZ1/2 sensitivity and
                                                    //     Acc_XYZ2 decimation setting frame.
#define REQ_SET_ACC3_CTRL           0x0AA8000000    // For building Acc_XYZ3 sensitivity setting frame.
#define REQ_SET_MODE_CTRL           0x0D68000000    // For building MODE-register setting frame.

// DRY/SYNC configuration
#define REQ_READ_USER_IF_CTRL       0x0CC80000007C
#define REQ_SET_USER_IF_CTRL        0x0CE8000000    // For building USER_IF_CTRL -register setting frame.

// Other
#define REQ_SOFTRESET               0x0DA800000AC3  // SPI soft reset command.

/**
 * Frame field masks
 */
#define TA_FIELD_MASK               0xFFC000000000
#define SA_FIELD_MASK               0x7FE000000000
#define DATA_FIELD_MASK             0x00000FFFFF00
#define CRC_FIELD_MASK              0x0000000000FF
#define ERROR_FIELD_MASK            0x001E00000000


/**
 * Status Summary Register bit definitions
 */
#define S_SUM_CMN               0x0080
#define S_SUM_RATE_X            0x0040
#define S_SUM_RATE_Y            0x0020
#define S_SUM_RATE_Z            0x0010
#define S_SUM_ACC_X             0x0008
#define S_SUM_ACC_Y             0x0004
#define S_SUM_ACC_Z             0x0002
#define S_SUM_INIT_RDY          0x0001

/**
 * Saturation Status Summary Register bit definitions
 */
#define S_SUM_SAT_RATE_X1       0x4000
#define S_SUM_SAT_RATE_Y1       0x2000
#define S_SUM_SAT_RATE_Z1       0x1000
#define S_SUM_SAT_ACC_X1        0x0800
#define S_SUM_SAT_ACC_Y1        0x0400
#define S_SUM_SAT_ACC_Z1        0x0200
#define S_SUM_SAT_ACC_X3        0x0100
#define S_SUM_SAT_ACC_Y3        0x0080
#define S_SUM_SAT_ACC_Z3        0x0040
#define S_SUM_SAT_RATE_X2       0x0020
#define S_SUM_SAT_RATE_Y2       0x0010
#define S_SUM_SAT_RATE_Z2       0x0008
#define S_SUM_SAT_ACC_X2        0x0004
#define S_SUM_SAT_ACC_Y2        0x0002
#define S_SUM_SAT_ACC_Z2        0x0001

/**
 * Common Status Register bit definitions
 */
#define S_COM_MCLK              0x0400
#define S_COM_DUAL_CLOCK        0x0200
#define S_COM_DSP               0x0100
#define S_COM_SVM               0x0080
#define S_COM_HV_CP             0x0040
#define S_COM_SUPPLY            0x0020
#define S_COM_TEMP              0x0010
#define S_COM_NMODE             0x0008
#define S_COM_NVM_STS           0x0004
#define S_COM_CMN_STS           0x0002
#define S_COM_CMN_STS_RDY       0x0001

/**
 * Rate Common Status Register bit definitions
 */
#define S_RATE_COM_PRI_AGC      0x0080
#define S_RATE_COM_PRI          0x0040
#define S_RATE_COM_PRI_START    0x0020
#define S_RATE_COM_HV           0x0010
#define S_RATE_COM_SD_STS       0x0004
#define S_RATE_COM_BOND_STS     0x0002
#define S_RATE_COM_STS_RDY      0x0001

/**
 * Rate Status X Register bit definitions
 */
#define S_RATE_X_DEC_SAT        0x0200
#define S_RATE_X_INTP_SAT       0x0100
#define S_RATE_X_STC_DIG        0x0040
#define S_RATE_X_STC_ANA        0x0020
#define S_RATE_X_QC             0x0010

/**
 * Rate Status Y Register bit definitions
 */
#define S_RATE_Y_DEC_SAT        0x0200
#define S_RATE_Y_INTP_SAT       0x0100
#define S_RATE_Y_STC_DIG        0x0040
#define S_RATE_Y_STC_ANA        0x0020
#define S_RATE_Y_QC             0x0010

/**
 * Rate Status Z Register bit definitions
 */
#define S_RATE_Z_DEC_SAT        0x0200
#define S_RATE_Z_INTP_SAT       0x0100
#define S_RATE_Z_STC_DIG        0x0040
#define S_RATE_Z_STC_ANA        0x0020
#define S_RATE_Z_QC             0x0010

/**
 * ACC Status X Register bit definitions
 */
#define S_ACC_X_SAT             0x0400
#define S_ACC_X_DEC_SAT         0x0200
#define S_ACC_X_INTP_SAT        0x0100
#define S_ACC_X_STC_DIG         0x0080
#define S_ACC_X_STC_TCAP        0x0040
#define S_ACC_X_STC_SDD         0x0020
#define S_ACC_X_STC_N           0x0010
#define S_ACC_X_SD_STS          0x0004
#define S_ACC_X_STS             0x0002
#define S_ACC_X_STS_RDY         0x0001

/**
 * ACC Status Y Register bit definitions
 */
#define S_ACC_Y_SAT             0x0400
#define S_ACC_Y_DEC_SAT         0x0200
#define S_ACC_Y_INTP_SAT        0x0100
#define S_ACC_Y_STC_DIG         0x0080
#define S_ACC_Y_STC_TCAP        0x0040
#define S_ACC_Y_STC_SDD         0x0020
#define S_ACC_Y_STC_N           0x0010
#define S_ACC_Y_SD_STS          0x0004
#define S_ACC_Y_STS             0x0002
#define S_ACC_Y_STS_RDY         0x0001

/**
 * ACC Status Z Register bit definitions
 */
#define S_ACC_Z_SAT             0x0400
#define S_ACC_Z_DEC_SAT         0x0200
#define S_ACC_Z_INTP_SAT        0x0100
#define S_ACC_Z_STC_DIG         0x0080
#define S_ACC_Z_STC_TCAP        0x0040
#define S_ACC_Z_STC_SDD         0x0020
#define S_ACC_Z_STC_N           0x0010
#define S_ACC_Z_SD_STS          0x0004
#define S_ACC_Z_STS             0x0002
#define S_ACC_Z_STS_RDY         0x0001


#endif


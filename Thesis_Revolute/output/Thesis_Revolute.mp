  ROM 78% full. 12606 bytes used (does not include absolute areas).
  RAM 67% full. 520 bytes used (does not include stack usage).

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   0190   0500 =   1280. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0190  LoadConfigTBL_hello_Bank0
       01B5  LoadConfigTBL_hello_Bank1
       01E6  UnloadConfigTBL_hello_Bank0
       01F3  UnloadConfigTBL_hello_Bank1
       0225  LoadConfigTBL_initial_Bank0
       0238  LoadConfigTBL_initial_Bank1
       0251  UnloadConfigTBL_initial_Bank0
       0258  UnloadConfigTBL_initial_Bank1
       0272  LoadConfigTBL_my_response_Bank0
       028B  LoadConfigTBL_my_response_Bank1
       02AC  UnloadConfigTBL_my_response_Bank0
       02B5  UnloadConfigTBL_my_response_Bank1
       02D7  LoadConfigTBL_response1_Bank0
       02EA  LoadConfigTBL_response1_Bank1
       0303  UnloadConfigTBL_response1_Bank0
       030A  UnloadConfigTBL_response1_Bank1
       0324  LoadConfigTBL_response2_Bank0
       0337  LoadConfigTBL_response2_Bank1
       0350  UnloadConfigTBL_response2_Bank0
       0357  UnloadConfigTBL_response2_Bank1
       0371  LoadConfigTBL_response3_Bank0
       0384  LoadConfigTBL_response3_Bank1
       039D  UnloadConfigTBL_response3_Bank0
       03A4  UnloadConfigTBL_response3_Bank1
       03BE  LoadConfigTBL_response4_Bank0
       03D1  LoadConfigTBL_response4_Bank1
       03EA  UnloadConfigTBL_response4_Bank0
       03F1  UnloadConfigTBL_response4_Bank1
       040B  LoadConfigTBL_servo_transmit_Bank0
       0412  LoadConfigTBL_servo_transmit_Bank1
       041B  UnloadConfigTBL_servo_transmit_Bank0
       041E  UnloadConfigTBL_servo_transmit_Bank1
       0428  LoadConfigTBL_waiting_Bank0
       0497  LoadConfigTBL_waiting_Bank1
       0530  LoadConfigTBL_waiting_Ordered
       05D9  ReloadConfigTBL_waiting_Bank0
       05E6  ReloadConfigTBL_waiting_Bank1
       05F7  UnloadConfigTBL_waiting_Bank0
       05FC  UnloadConfigTBL_waiting_Bank1
       060E  UnloadConfigTBL_Total_Bank0
       062F  UnloadConfigTBL_Total_Bank1
       0690  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                        func_lit   0690   0002 =      2. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0690  __func_lit_start
       0692  __func_lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     psoc_config   0692   05BC =   1468. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0692  _LoadConfigInit
       0692  LoadConfigInit
       06A2  _LoadConfig_waiting
       06A2  LoadConfig_waiting
       06C5  ReloadConfig_waiting
       06C5  _ReloadConfig_waiting
       06E6  UnloadConfig_waiting
       06E6  _UnloadConfig_waiting
       0709  LoadConfig_hello
       0709  _LoadConfig_hello
       0755  UnloadConfig_hello
       0755  _UnloadConfig_hello
       07AC  _LoadConfig_initial
       07AC  LoadConfig_initial
       07F5  _UnloadConfig_initial
       07F5  UnloadConfig_initial
       0846  _LoadConfig_my_response
       0846  LoadConfig_my_response
       087A  _UnloadConfig_my_response
       087A  UnloadConfig_my_response
       08AD  LoadConfig_response1
       08AD  _LoadConfig_response1
       08FC  UnloadConfig_response1
       08FC  _UnloadConfig_response1
       0950  LoadConfig_response2
       0950  _LoadConfig_response2
       099F  _UnloadConfig_response2
       099F  UnloadConfig_response2
       09F3  LoadConfig_response3
       09F3  _LoadConfig_response3
       0A42  UnloadConfig_response3
       0A42  _UnloadConfig_response3
       0A96  LoadConfig_response4
       0A96  _LoadConfig_response4
       0AEB  UnloadConfig_response4
       0AEB  _UnloadConfig_response4
       0B42  LoadConfig_servo_transmit
       0B42  _LoadConfig_servo_transmit
       0B91  UnloadConfig_servo_transmit
       0B91  _UnloadConfig_servo_transmit
       0BE5  _UnloadConfig_Total
       0BE5  UnloadConfig_Total
       0C4E  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   0C4E   13FE =   5118. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0C4E  _WAIT_RECV_ISR
       0CC2  WAIT_RECV_EnableInt
       0CC2  _WAIT_RECV_EnableInt
       0CC6  _WAIT_RECV_DisableInt
       0CC6  WAIT_RECV_DisableInt
       0CCA  WAIT_RECV_Start
       0CCA  _WAIT_RECV_Start
       0CCF  WAIT_RECV_Stop
       0CCF  _WAIT_RECV_Stop
       0CD3  WAIT_RECV_bReadRxData
       0CD3  _WAIT_RECV_bReadRxData
       0CD3  _bWAIT_RECV_ReadRxData
       0CD3  bWAIT_RECV_ReadRxData
       0CD6  _bWAIT_RECV_ReadRxStatus
       0CD6  _WAIT_RECV_bReadRxStatus
       0CD6  bWAIT_RECV_ReadRxStatus
       0CD6  WAIT_RECV_bReadRxStatus
       0CD9  _WAIT_RECV_cGetChar
       0CD9  WAIT_RECV_cGetChar
       0CE1  _WAIT_RECV_cReadChar
       0CE1  WAIT_RECV_cReadChar
       0CF8  WAIT_RECV_iReadChar
       0CF8  _WAIT_RECV_iReadChar
       0D0E  WAIT_RECV_CmdReset
       0D0E  _WAIT_RECV_CmdReset
       0D24  _WAIT_RECV_bCmdCheck
       0D24  WAIT_RECV_bCmdCheck
       0D2C  WAIT_RECV_bErrCheck
       0D2C  _WAIT_RECV_bErrCheck
       0D37  WAIT_RECV_bCmdLength
       0D37  _WAIT_RECV_bCmdLength
       0D3D  _WAIT_RECV_szGetParam
       0D3D  WAIT_RECV_szGetParam
       0D9C  WAIT_RECV_szGetRestOfParams
       0D9C  _WAIT_RECV_szGetRestOfParams
       0DA7  _TX_23_ISR
       0DA8  _TX_23_EnableInt
       0DA8  TX_23_EnableInt
       0DAC  TX_23_DisableInt
       0DAC  _TX_23_DisableInt
       0DB0  _TX_23_SetTxIntMode
       0DB0  TX_23_SetTxIntMode
       0DC2  _TX_23_Start
       0DC2  TX_23_Start
       0DC7  _TX_23_Stop
       0DC7  TX_23_Stop
       0DCB  TX_23_SendData
       0DCB  _TX_23_SendData
       0DCE  bTX_23_ReadTxStatus
       0DCE  _bTX_23_ReadTxStatus
       0DCE  _TX_23_bReadTxStatus
       0DCE  TX_23_bReadTxStatus
       0DE1  _TX_23_PutSHexByte
       0DE1  TX_23_PutSHexByte
       0DF4  TX_23_PutSHexInt
       0DF4  _TX_23_PutSHexInt
       0DFB  _TX_23_PutChar
       0DFB  TX_23_PutChar
       0E03  _TX_23_PutString
       0E03  TX_23_PutString
       0E15  _TX_23_Write
       0E15  TX_23_Write
       0E3D  _TX_23_CWrite
       0E3D  TX_23_CWrite
       0E65  TX_23_CPutString
       0E65  _TX_23_CPutString
       0E77  TX_23_PutCRLF
       0E77  _TX_23_PutCRLF
       0E80  _TX_014_ISR
       0E81  _TX_014_EnableInt
       0E81  TX_014_EnableInt
       0E85  _TX_014_DisableInt
       0E85  TX_014_DisableInt
       0E89  _TX_014_SetTxIntMode
       0E89  TX_014_SetTxIntMode
       0E9B  TX_014_Start
       0E9B  _TX_014_Start
       0EA0  TX_014_Stop
       0EA0  _TX_014_Stop
       0EA4  TX_014_SendData
       0EA4  _TX_014_SendData
       0EA7  _bTX_014_ReadTxStatus
       0EA7  _TX_014_bReadTxStatus
       0EA7  bTX_014_ReadTxStatus
       0EA7  TX_014_bReadTxStatus
       0EBA  _TX_014_PutSHexByte
       0EBA  TX_014_PutSHexByte
       0ECD  _TX_014_PutSHexInt
       0ECD  TX_014_PutSHexInt
       0ED4  TX_014_PutChar
       0ED4  _TX_014_PutChar
       0EDC  TX_014_PutString
       0EDC  _TX_014_PutString
       0EEE  _TX_014_Write
       0EEE  TX_014_Write
       0F16  _TX_014_CWrite
       0F16  TX_014_CWrite
       0F3E  TX_014_CPutString
       0F3E  _TX_014_CPutString
       0F50  _TX_014_PutCRLF
       0F50  TX_014_PutCRLF
       0F59  TX_01234_TIMEOUT_EnableInt
       0F59  _TX_01234_TIMEOUT_EnableInt
       0F5D  _TX_01234_TIMEOUT_DisableInt
       0F5D  TX_01234_TIMEOUT_DisableInt
       0F61  _TX_01234_TIMEOUT_Start
       0F61  TX_01234_TIMEOUT_Start
       0F65  _TX_01234_TIMEOUT_Stop
       0F65  TX_01234_TIMEOUT_Stop
       0F69  _TX_01234_TIMEOUT_WritePeriod
       0F69  TX_01234_TIMEOUT_WritePeriod
       0F6F  TX_01234_TIMEOUT_WriteCompareValue
       0F6F  _TX_01234_TIMEOUT_WriteCompareValue
       0F75  TX_01234_TIMEOUT_wReadCompareValue
       0F75  _TX_01234_TIMEOUT_wReadCompareValue
       0F75  _wTX_01234_TIMEOUT_ReadCompareValue
       0F75  wTX_01234_TIMEOUT_ReadCompareValue
       0F7B  _TX_01234_TIMEOUT_wReadTimerSaveCV
       0F7B  _wTX_01234_TIMEOUT_ReadTimerSaveCV
       0F7B  _wTX_01234_TIMEOUT_ReadCounter
       0F7B  wTX_01234_TIMEOUT_ReadTimerSaveCV
       0F7B  TX_01234_TIMEOUT_wReadTimerSaveCV
       0F7B  wTX_01234_TIMEOUT_ReadCounter
       0FB1  _wTX_01234_TIMEOUT_CaptureCounter
       0FB1  _wTX_01234_TIMEOUT_ReadTimer
       0FB1  _TX_01234_TIMEOUT_wReadTimer
       0FB1  wTX_01234_TIMEOUT_CaptureCounter
       0FB1  wTX_01234_TIMEOUT_ReadTimer
       0FB1  TX_01234_TIMEOUT_wReadTimer
       0FB9  _SERVO_TX_ISR
       0FBA  SERVO_TX_EnableInt
       0FBA  _SERVO_TX_EnableInt
       0FBE  _SERVO_TX_DisableInt
       0FBE  SERVO_TX_DisableInt
       0FC2  SERVO_TX_SetTxIntMode
       0FC2  _SERVO_TX_SetTxIntMode
       0FD4  SERVO_TX_Start
       0FD4  _SERVO_TX_Start
       0FD9  _SERVO_TX_Stop
       0FD9  SERVO_TX_Stop
       0FDD  _SERVO_TX_SendData
       0FDD  SERVO_TX_SendData
       0FE0  _bSERVO_TX_ReadTxStatus
       0FE0  _SERVO_TX_bReadTxStatus
       0FE0  SERVO_TX_bReadTxStatus
       0FE0  bSERVO_TX_ReadTxStatus
       0FF3  _SERVO_TX_PutSHexByte
       0FF3  SERVO_TX_PutSHexByte
       1006  _SERVO_TX_PutSHexInt
       1006  SERVO_TX_PutSHexInt
       100D  SERVO_TX_PutChar
       100D  _SERVO_TX_PutChar
       1015  _SERVO_TX_PutString
       1015  SERVO_TX_PutString
       1027  SERVO_TX_Write
       1027  _SERVO_TX_Write
       104F  SERVO_TX_CWrite
       104F  _SERVO_TX_CWrite
       1077  _SERVO_TX_CPutString
       1077  SERVO_TX_CPutString
       1089  _SERVO_TX_PutCRLF
       1089  SERVO_TX_PutCRLF
       1092  _RX8_2_ISR
       1106  _RX8_2_EnableInt
       1106  RX8_2_EnableInt
       110A  RX8_2_DisableInt
       110A  _RX8_2_DisableInt
       110E  RX8_2_Start
       110E  _RX8_2_Start
       1113  _RX8_2_Stop
       1113  RX8_2_Stop
       1117  RX8_2_bReadRxData
       1117  bRX8_2_ReadRxData
       1117  _RX8_2_bReadRxData
       1117  _bRX8_2_ReadRxData
       111A  _bRX8_2_ReadRxStatus
       111A  RX8_2_bReadRxStatus
       111A  _RX8_2_bReadRxStatus
       111A  bRX8_2_ReadRxStatus
       111D  _RX8_2_cGetChar
       111D  RX8_2_cGetChar
       1125  RX8_2_cReadChar
       1125  _RX8_2_cReadChar
       113C  RX8_2_iReadChar
       113C  _RX8_2_iReadChar
       1152  _RX8_2_CmdReset
       1152  RX8_2_CmdReset
       1168  _RX8_2_bCmdCheck
       1168  RX8_2_bCmdCheck
       1170  _RX8_2_bErrCheck
       1170  RX8_2_bErrCheck
       117B  _RX8_2_bCmdLength
       117B  RX8_2_bCmdLength
       1181  _RX8_2_szGetParam
       1181  RX8_2_szGetParam
       11E0  RX8_2_szGetRestOfParams
       11E0  _RX8_2_szGetRestOfParams
       11EB  _INIT_TIMEOUT_EnableInt
       11EB  INIT_TIMEOUT_EnableInt
       11EF  _INIT_TIMEOUT_DisableInt
       11EF  INIT_TIMEOUT_DisableInt
       11F3  INIT_TIMEOUT_Start
       11F3  _INIT_TIMEOUT_Start
       11F7  _INIT_TIMEOUT_Stop
       11F7  INIT_TIMEOUT_Stop
       11FB  INIT_TIMEOUT_WritePeriod
       11FB  _INIT_TIMEOUT_WritePeriod
       1201  _INIT_TIMEOUT_WriteCompareValue
       1201  INIT_TIMEOUT_WriteCompareValue
       1207  _wINIT_TIMEOUT_ReadCompareValue
       1207  _INIT_TIMEOUT_wReadCompareValue
       1207  wINIT_TIMEOUT_ReadCompareValue
       1207  INIT_TIMEOUT_wReadCompareValue
       120D  _wINIT_TIMEOUT_ReadCounter
       120D  wINIT_TIMEOUT_ReadCounter
       120D  INIT_TIMEOUT_wReadTimerSaveCV
       120D  wINIT_TIMEOUT_ReadTimerSaveCV
       120D  _INIT_TIMEOUT_wReadTimerSaveCV
       120D  _wINIT_TIMEOUT_ReadTimerSaveCV
       1243  wINIT_TIMEOUT_ReadTimer
       1243  _wINIT_TIMEOUT_ReadTimer
       1243  _wINIT_TIMEOUT_CaptureCounter
       1243  _INIT_TIMEOUT_wReadTimer
       1243  INIT_TIMEOUT_wReadTimer
       1243  wINIT_TIMEOUT_CaptureCounter
       124B  _INIT_RX_ISR
       12BF  INIT_RX_EnableInt
       12BF  _INIT_RX_EnableInt
       12C3  _INIT_RX_DisableInt
       12C3  INIT_RX_DisableInt
       12C7  INIT_RX_Start
       12C7  _INIT_RX_Start
       12CC  _INIT_RX_Stop
       12CC  INIT_RX_Stop
       12D0  INIT_RX_bReadRxData
       12D0  _bINIT_RX_ReadRxData
       12D0  _INIT_RX_bReadRxData
       12D0  bINIT_RX_ReadRxData
       12D3  _INIT_RX_bReadRxStatus
       12D3  _bINIT_RX_ReadRxStatus
       12D3  bINIT_RX_ReadRxStatus
       12D3  INIT_RX_bReadRxStatus
       12D6  INIT_RX_cGetChar
       12D6  _INIT_RX_cGetChar
       12DE  _INIT_RX_cReadChar
       12DE  INIT_RX_cReadChar
       12F5  _INIT_RX_iReadChar
       12F5  INIT_RX_iReadChar
       130B  _INIT_RX_CmdReset
       130B  INIT_RX_CmdReset
       1321  INIT_RX_bCmdCheck
       1321  _INIT_RX_bCmdCheck
       1329  INIT_RX_bErrCheck
       1329  _INIT_RX_bErrCheck
       1334  INIT_RX_bCmdLength
       1334  _INIT_RX_bCmdLength
       133A  INIT_RX_szGetParam
       133A  _INIT_RX_szGetParam
       1399  _INIT_RX_szGetRestOfParams
       1399  INIT_RX_szGetRestOfParams
       13A4  HELLO_TIMEOUT_EnableInt
       13A4  _HELLO_TIMEOUT_EnableInt
       13A8  HELLO_TIMEOUT_DisableInt
       13A8  _HELLO_TIMEOUT_DisableInt
       13AC  _HELLO_TIMEOUT_Start
       13AC  HELLO_TIMEOUT_Start
       13B0  _HELLO_TIMEOUT_Stop
       13B0  HELLO_TIMEOUT_Stop
       13B4  _HELLO_TIMEOUT_WritePeriod
       13B4  HELLO_TIMEOUT_WritePeriod
       13BA  _HELLO_TIMEOUT_WriteCompareValue
       13BA  HELLO_TIMEOUT_WriteCompareValue
       13C0  HELLO_TIMEOUT_wReadCompareValue
       13C0  _HELLO_TIMEOUT_wReadCompareValue
       13C0  _wHELLO_TIMEOUT_ReadCompareValue
       13C0  wHELLO_TIMEOUT_ReadCompareValue
       13C6  _HELLO_TIMEOUT_wReadTimerSaveCV
       13C6  _wHELLO_TIMEOUT_ReadTimerSaveCV
       13C6  HELLO_TIMEOUT_wReadTimerSaveCV
       13C6  wHELLO_TIMEOUT_ReadTimerSaveCV
       13C6  _wHELLO_TIMEOUT_ReadCounter
       13C6  wHELLO_TIMEOUT_ReadCounter
       13FC  _wHELLO_TIMEOUT_CaptureCounter
       13FC  HELLO_TIMEOUT_wReadTimer
       13FC  wHELLO_TIMEOUT_CaptureCounter
       13FC  wHELLO_TIMEOUT_ReadTimer
       13FC  _wHELLO_TIMEOUT_ReadTimer
       13FC  _HELLO_TIMEOUT_wReadTimer
       1404  _HELLO_4_ISR
       1478  _HELLO_4_EnableInt
       1478  HELLO_4_EnableInt
       147C  _HELLO_4_DisableInt
       147C  HELLO_4_DisableInt
       1480  _HELLO_4_Start
       1480  HELLO_4_Start
       1485  HELLO_4_Stop
       1485  _HELLO_4_Stop
       1489  _HELLO_4_bReadRxData
       1489  HELLO_4_bReadRxData
       1489  _bHELLO_4_ReadRxData
       1489  bHELLO_4_ReadRxData
       148C  _HELLO_4_bReadRxStatus
       148C  HELLO_4_bReadRxStatus
       148C  bHELLO_4_ReadRxStatus
       148C  _bHELLO_4_ReadRxStatus
       148F  HELLO_4_cGetChar
       148F  _HELLO_4_cGetChar
       1497  HELLO_4_cReadChar
       1497  _HELLO_4_cReadChar
       14AE  HELLO_4_iReadChar
       14AE  _HELLO_4_iReadChar
       14C4  _HELLO_4_CmdReset
       14C4  HELLO_4_CmdReset
       14DA  _HELLO_4_bCmdCheck
       14DA  HELLO_4_bCmdCheck
       14E2  _HELLO_4_bErrCheck
       14E2  HELLO_4_bErrCheck
       14ED  HELLO_4_bCmdLength
       14ED  _HELLO_4_bCmdLength
       14F3  _HELLO_4_szGetParam
       14F3  HELLO_4_szGetParam
       1552  _HELLO_4_szGetRestOfParams
       1552  HELLO_4_szGetRestOfParams
       155D  _HELLO_3_ISR
       15D1  HELLO_3_EnableInt
       15D1  _HELLO_3_EnableInt
       15D5  HELLO_3_DisableInt
       15D5  _HELLO_3_DisableInt
       15D9  _HELLO_3_Start
       15D9  HELLO_3_Start
       15DE  _HELLO_3_Stop
       15DE  HELLO_3_Stop
       15E2  bHELLO_3_ReadRxData
       15E2  HELLO_3_bReadRxData
       15E2  _bHELLO_3_ReadRxData
       15E2  _HELLO_3_bReadRxData
       15E5  HELLO_3_bReadRxStatus
       15E5  _HELLO_3_bReadRxStatus
       15E5  bHELLO_3_ReadRxStatus
       15E5  _bHELLO_3_ReadRxStatus
       15E8  _HELLO_3_cGetChar
       15E8  HELLO_3_cGetChar
       15F0  HELLO_3_cReadChar
       15F0  _HELLO_3_cReadChar
       1607  _HELLO_3_iReadChar
       1607  HELLO_3_iReadChar
       161D  _HELLO_3_CmdReset
       161D  HELLO_3_CmdReset
       1633  HELLO_3_bCmdCheck
       1633  _HELLO_3_bCmdCheck
       163B  HELLO_3_bErrCheck
       163B  _HELLO_3_bErrCheck
       1646  HELLO_3_bCmdLength
       1646  _HELLO_3_bCmdLength
       164C  HELLO_3_szGetParam
       164C  _HELLO_3_szGetParam
       16AB  _HELLO_3_szGetRestOfParams
       16AB  HELLO_3_szGetRestOfParams
       16B6  _HELLO_2_ISR
       172A  _HELLO_2_EnableInt
       172A  HELLO_2_EnableInt
       172E  _HELLO_2_DisableInt
       172E  HELLO_2_DisableInt
       1732  _HELLO_2_Start
       1732  HELLO_2_Start
       1737  HELLO_2_Stop
       1737  _HELLO_2_Stop
       173B  bHELLO_2_ReadRxData
       173B  _bHELLO_2_ReadRxData
       173B  _HELLO_2_bReadRxData
       173B  HELLO_2_bReadRxData
       173E  _HELLO_2_bReadRxStatus
       173E  _bHELLO_2_ReadRxStatus
       173E  HELLO_2_bReadRxStatus
       173E  bHELLO_2_ReadRxStatus
       1741  HELLO_2_cGetChar
       1741  _HELLO_2_cGetChar
       1749  HELLO_2_cReadChar
       1749  _HELLO_2_cReadChar
       1760  HELLO_2_iReadChar
       1760  _HELLO_2_iReadChar
       1776  _HELLO_2_CmdReset
       1776  HELLO_2_CmdReset
       178C  _HELLO_2_bCmdCheck
       178C  HELLO_2_bCmdCheck
       1794  _HELLO_2_bErrCheck
       1794  HELLO_2_bErrCheck
       179F  _HELLO_2_bCmdLength
       179F  HELLO_2_bCmdLength
       17A5  HELLO_2_szGetParam
       17A5  _HELLO_2_szGetParam
       1804  _HELLO_2_szGetRestOfParams
       1804  HELLO_2_szGetRestOfParams
       180F  _HELLO_1_ISR
       1883  HELLO_1_EnableInt
       1883  _HELLO_1_EnableInt
       1887  HELLO_1_DisableInt
       1887  _HELLO_1_DisableInt
       188B  HELLO_1_Start
       188B  _HELLO_1_Start
       1890  HELLO_1_Stop
       1890  _HELLO_1_Stop
       1894  bHELLO_1_ReadRxData
       1894  HELLO_1_bReadRxData
       1894  _HELLO_1_bReadRxData
       1894  _bHELLO_1_ReadRxData
       1897  HELLO_1_bReadRxStatus
       1897  bHELLO_1_ReadRxStatus
       1897  _bHELLO_1_ReadRxStatus
       1897  _HELLO_1_bReadRxStatus
       189A  HELLO_1_cGetChar
       189A  _HELLO_1_cGetChar
       18A2  _HELLO_1_cReadChar
       18A2  HELLO_1_cReadChar
       18B9  HELLO_1_iReadChar
       18B9  _HELLO_1_iReadChar
       18CF  _HELLO_1_CmdReset
       18CF  HELLO_1_CmdReset
       18E5  HELLO_1_bCmdCheck
       18E5  _HELLO_1_bCmdCheck
       18ED  HELLO_1_bErrCheck
       18ED  _HELLO_1_bErrCheck
       18F8  HELLO_1_bCmdLength
       18F8  _HELLO_1_bCmdLength
       18FE  HELLO_1_szGetParam
       18FE  _HELLO_1_szGetParam
       195D  HELLO_1_szGetRestOfParams
       195D  _HELLO_1_szGetRestOfParams
       1968  _CHILD_4_ISR
       19DC  CHILD_4_TIMEOUT_EnableInt
       19DC  _CHILD_4_TIMEOUT_EnableInt
       19E0  CHILD_4_TIMEOUT_DisableInt
       19E0  _CHILD_4_TIMEOUT_DisableInt
       19E4  _CHILD_4_TIMEOUT_Start
       19E4  CHILD_4_TIMEOUT_Start
       19E8  _CHILD_4_TIMEOUT_Stop
       19E8  CHILD_4_TIMEOUT_Stop
       19EC  CHILD_4_TIMEOUT_WritePeriod
       19EC  _CHILD_4_TIMEOUT_WritePeriod
       19F2  CHILD_4_TIMEOUT_WriteCompareValue
       19F2  _CHILD_4_TIMEOUT_WriteCompareValue
       19F8  _wCHILD_4_TIMEOUT_ReadCompareValue
       19F8  CHILD_4_TIMEOUT_wReadCompareValue
       19F8  _CHILD_4_TIMEOUT_wReadCompareValue
       19F8  wCHILD_4_TIMEOUT_ReadCompareValue
       19FE  wCHILD_4_TIMEOUT_ReadCounter
       19FE  _wCHILD_4_TIMEOUT_ReadCounter
       19FE  CHILD_4_TIMEOUT_wReadTimerSaveCV
       19FE  wCHILD_4_TIMEOUT_ReadTimerSaveCV
       19FE  _CHILD_4_TIMEOUT_wReadTimerSaveCV
       19FE  _wCHILD_4_TIMEOUT_ReadTimerSaveCV
       1A34  _wCHILD_4_TIMEOUT_ReadTimer
       1A34  wCHILD_4_TIMEOUT_CaptureCounter
       1A34  _wCHILD_4_TIMEOUT_CaptureCounter
       1A34  wCHILD_4_TIMEOUT_ReadTimer
       1A34  CHILD_4_TIMEOUT_wReadTimer
       1A34  _CHILD_4_TIMEOUT_wReadTimer
       1A3C  _CHILD_4_EnableInt
       1A3C  CHILD_4_EnableInt
       1A40  CHILD_4_DisableInt
       1A40  _CHILD_4_DisableInt
       1A44  _CHILD_4_Start
       1A44  CHILD_4_Start
       1A49  CHILD_4_Stop
       1A49  _CHILD_4_Stop
       1A4D  _CHILD_4_bReadRxData
       1A4D  CHILD_4_bReadRxData
       1A4D  bCHILD_4_ReadRxData
       1A4D  _bCHILD_4_ReadRxData
       1A50  _CHILD_4_bReadRxStatus
       1A50  _bCHILD_4_ReadRxStatus
       1A50  CHILD_4_bReadRxStatus
       1A50  bCHILD_4_ReadRxStatus
       1A53  _CHILD_4_cGetChar
       1A53  CHILD_4_cGetChar
       1A5B  CHILD_4_cReadChar
       1A5B  _CHILD_4_cReadChar
       1A72  CHILD_4_iReadChar
       1A72  _CHILD_4_iReadChar
       1A88  _CHILD_4_CmdReset
       1A88  CHILD_4_CmdReset
       1A9E  _CHILD_4_bCmdCheck
       1A9E  CHILD_4_bCmdCheck
       1AA6  _CHILD_4_bErrCheck
       1AA6  CHILD_4_bErrCheck
       1AB1  CHILD_4_bCmdLength
       1AB1  _CHILD_4_bCmdLength
       1AB7  _CHILD_4_szGetParam
       1AB7  CHILD_4_szGetParam
       1B16  _CHILD_4_szGetRestOfParams
       1B16  CHILD_4_szGetRestOfParams
       1B21  _CHILD_3_ISR
       1B95  _CHILD_3_TIMEOUT_EnableInt
       1B95  CHILD_3_TIMEOUT_EnableInt
       1B99  _CHILD_3_TIMEOUT_DisableInt
       1B99  CHILD_3_TIMEOUT_DisableInt
       1B9D  CHILD_3_TIMEOUT_Start
       1B9D  _CHILD_3_TIMEOUT_Start
       1BA1  CHILD_3_TIMEOUT_Stop
       1BA1  _CHILD_3_TIMEOUT_Stop
       1BA5  _CHILD_3_TIMEOUT_WritePeriod
       1BA5  CHILD_3_TIMEOUT_WritePeriod
       1BAB  _CHILD_3_TIMEOUT_WriteCompareValue
       1BAB  CHILD_3_TIMEOUT_WriteCompareValue
       1BB1  wCHILD_3_TIMEOUT_ReadCompareValue
       1BB1  CHILD_3_TIMEOUT_wReadCompareValue
       1BB1  _CHILD_3_TIMEOUT_wReadCompareValue
       1BB1  _wCHILD_3_TIMEOUT_ReadCompareValue
       1BB7  _wCHILD_3_TIMEOUT_ReadTimerSaveCV
       1BB7  CHILD_3_TIMEOUT_wReadTimerSaveCV
       1BB7  wCHILD_3_TIMEOUT_ReadCounter
       1BB7  _CHILD_3_TIMEOUT_wReadTimerSaveCV
       1BB7  wCHILD_3_TIMEOUT_ReadTimerSaveCV
       1BB7  _wCHILD_3_TIMEOUT_ReadCounter
       1BED  wCHILD_3_TIMEOUT_CaptureCounter
       1BED  _wCHILD_3_TIMEOUT_CaptureCounter
       1BED  _wCHILD_3_TIMEOUT_ReadTimer
       1BED  _CHILD_3_TIMEOUT_wReadTimer
       1BED  wCHILD_3_TIMEOUT_ReadTimer
       1BED  CHILD_3_TIMEOUT_wReadTimer
       1BF5  CHILD_3_EnableInt
       1BF5  _CHILD_3_EnableInt
       1BF9  _CHILD_3_DisableInt
       1BF9  CHILD_3_DisableInt
       1BFD  _CHILD_3_Start
       1BFD  CHILD_3_Start
       1C02  CHILD_3_Stop
       1C02  _CHILD_3_Stop
       1C06  _CHILD_3_bReadRxData
       1C06  _bCHILD_3_ReadRxData
       1C06  bCHILD_3_ReadRxData
       1C06  CHILD_3_bReadRxData
       1C09  CHILD_3_bReadRxStatus
       1C09  bCHILD_3_ReadRxStatus
       1C09  _bCHILD_3_ReadRxStatus
       1C09  _CHILD_3_bReadRxStatus
       1C0C  CHILD_3_cGetChar
       1C0C  _CHILD_3_cGetChar
       1C14  _CHILD_3_cReadChar
       1C14  CHILD_3_cReadChar
       1C2B  CHILD_3_iReadChar
       1C2B  _CHILD_3_iReadChar
       1C41  CHILD_3_CmdReset
       1C41  _CHILD_3_CmdReset
       1C57  CHILD_3_bCmdCheck
       1C57  _CHILD_3_bCmdCheck
       1C5F  _CHILD_3_bErrCheck
       1C5F  CHILD_3_bErrCheck
       1C6A  CHILD_3_bCmdLength
       1C6A  _CHILD_3_bCmdLength
       1C70  CHILD_3_szGetParam
       1C70  _CHILD_3_szGetParam
       1CCF  CHILD_3_szGetRestOfParams
       1CCF  _CHILD_3_szGetRestOfParams
       1CDA  _CHILD_2_ISR
       1D4E  CHILD_2_TIMEOUT_EnableInt
       1D4E  _CHILD_2_TIMEOUT_EnableInt
       1D52  CHILD_2_TIMEOUT_DisableInt
       1D52  _CHILD_2_TIMEOUT_DisableInt
       1D56  CHILD_2_TIMEOUT_Start
       1D56  _CHILD_2_TIMEOUT_Start
       1D5A  CHILD_2_TIMEOUT_Stop
       1D5A  _CHILD_2_TIMEOUT_Stop
       1D5E  _CHILD_2_TIMEOUT_WritePeriod
       1D5E  CHILD_2_TIMEOUT_WritePeriod
       1D64  _CHILD_2_TIMEOUT_WriteCompareValue
       1D64  CHILD_2_TIMEOUT_WriteCompareValue
       1D6A  CHILD_2_TIMEOUT_wReadCompareValue
       1D6A  wCHILD_2_TIMEOUT_ReadCompareValue
       1D6A  _CHILD_2_TIMEOUT_wReadCompareValue
       1D6A  _wCHILD_2_TIMEOUT_ReadCompareValue
       1D70  _wCHILD_2_TIMEOUT_ReadCounter
       1D70  wCHILD_2_TIMEOUT_ReadCounter
       1D70  wCHILD_2_TIMEOUT_ReadTimerSaveCV
       1D70  _wCHILD_2_TIMEOUT_ReadTimerSaveCV
       1D70  _CHILD_2_TIMEOUT_wReadTimerSaveCV
       1D70  CHILD_2_TIMEOUT_wReadTimerSaveCV
       1DA6  wCHILD_2_TIMEOUT_ReadTimer
       1DA6  CHILD_2_TIMEOUT_wReadTimer
       1DA6  _wCHILD_2_TIMEOUT_CaptureCounter
       1DA6  _wCHILD_2_TIMEOUT_ReadTimer
       1DA6  _CHILD_2_TIMEOUT_wReadTimer
       1DA6  wCHILD_2_TIMEOUT_CaptureCounter
       1DAE  _CHILD_2_EnableInt
       1DAE  CHILD_2_EnableInt
       1DB2  _CHILD_2_DisableInt
       1DB2  CHILD_2_DisableInt
       1DB6  _CHILD_2_Start
       1DB6  CHILD_2_Start
       1DBB  CHILD_2_Stop
       1DBB  _CHILD_2_Stop
       1DBF  _bCHILD_2_ReadRxData
       1DBF  _CHILD_2_bReadRxData
       1DBF  bCHILD_2_ReadRxData
       1DBF  CHILD_2_bReadRxData
       1DC2  _CHILD_2_bReadRxStatus
       1DC2  _bCHILD_2_ReadRxStatus
       1DC2  CHILD_2_bReadRxStatus
       1DC2  bCHILD_2_ReadRxStatus
       1DC5  CHILD_2_cGetChar
       1DC5  _CHILD_2_cGetChar
       1DCD  _CHILD_2_cReadChar
       1DCD  CHILD_2_cReadChar
       1DE4  _CHILD_2_iReadChar
       1DE4  CHILD_2_iReadChar
       1DFA  _CHILD_2_CmdReset
       1DFA  CHILD_2_CmdReset
       1E10  CHILD_2_bCmdCheck
       1E10  _CHILD_2_bCmdCheck
       1E18  CHILD_2_bErrCheck
       1E18  _CHILD_2_bErrCheck
       1E23  CHILD_2_bCmdLength
       1E23  _CHILD_2_bCmdLength
       1E29  _CHILD_2_szGetParam
       1E29  CHILD_2_szGetParam
       1E88  _CHILD_2_szGetRestOfParams
       1E88  CHILD_2_szGetRestOfParams
       1E93  _CHILD_1_ISR
       1F07  _CHILD_1_TIMEOUT_EnableInt
       1F07  CHILD_1_TIMEOUT_EnableInt
       1F0B  _CHILD_1_TIMEOUT_DisableInt
       1F0B  CHILD_1_TIMEOUT_DisableInt
       1F0F  _CHILD_1_TIMEOUT_Start
       1F0F  CHILD_1_TIMEOUT_Start
       1F13  _CHILD_1_TIMEOUT_Stop
       1F13  CHILD_1_TIMEOUT_Stop
       1F17  CHILD_1_TIMEOUT_WritePeriod
       1F17  _CHILD_1_TIMEOUT_WritePeriod
       1F1D  _CHILD_1_TIMEOUT_WriteCompareValue
       1F1D  CHILD_1_TIMEOUT_WriteCompareValue
       1F23  wCHILD_1_TIMEOUT_ReadCompareValue
       1F23  CHILD_1_TIMEOUT_wReadCompareValue
       1F23  _CHILD_1_TIMEOUT_wReadCompareValue
       1F23  _wCHILD_1_TIMEOUT_ReadCompareValue
       1F29  wCHILD_1_TIMEOUT_ReadCounter
       1F29  _wCHILD_1_TIMEOUT_ReadCounter
       1F29  _CHILD_1_TIMEOUT_wReadTimerSaveCV
       1F29  _wCHILD_1_TIMEOUT_ReadTimerSaveCV
       1F29  CHILD_1_TIMEOUT_wReadTimerSaveCV
       1F29  wCHILD_1_TIMEOUT_ReadTimerSaveCV
       1F5F  wCHILD_1_TIMEOUT_ReadTimer
       1F5F  _CHILD_1_TIMEOUT_wReadTimer
       1F5F  CHILD_1_TIMEOUT_wReadTimer
       1F5F  wCHILD_1_TIMEOUT_CaptureCounter
       1F5F  _wCHILD_1_TIMEOUT_CaptureCounter
       1F5F  _wCHILD_1_TIMEOUT_ReadTimer
       1F67  _CHILD_1_EnableInt
       1F67  CHILD_1_EnableInt
       1F6B  CHILD_1_DisableInt
       1F6B  _CHILD_1_DisableInt
       1F6F  _CHILD_1_Start
       1F6F  CHILD_1_Start
       1F74  CHILD_1_Stop
       1F74  _CHILD_1_Stop
       1F78  _CHILD_1_bReadRxData
       1F78  _bCHILD_1_ReadRxData
       1F78  bCHILD_1_ReadRxData
       1F78  CHILD_1_bReadRxData
       1F7B  bCHILD_1_ReadRxStatus
       1F7B  CHILD_1_bReadRxStatus
       1F7B  _bCHILD_1_ReadRxStatus
       1F7B  _CHILD_1_bReadRxStatus
       1F7E  CHILD_1_cGetChar
       1F7E  _CHILD_1_cGetChar
       1F86  CHILD_1_cReadChar
       1F86  _CHILD_1_cReadChar
       1F9D  CHILD_1_iReadChar
       1F9D  _CHILD_1_iReadChar
       1FB3  CHILD_1_CmdReset
       1FB3  _CHILD_1_CmdReset
       1FC9  _CHILD_1_bCmdCheck
       1FC9  CHILD_1_bCmdCheck
       1FD1  _CHILD_1_bErrCheck
       1FD1  CHILD_1_bErrCheck
       1FDC  _CHILD_1_bCmdLength
       1FDC  CHILD_1_bCmdLength
       1FE2  CHILD_1_szGetParam
       1FE2  _CHILD_1_szGetParam
       2041  _CHILD_1_szGetRestOfParams
       2041  CHILD_1_szGetRestOfParams
       204C  __UserModules_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   204C   1282 =   4738. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       204C  _main
       204C  __text_start
       209B  _sayHello
       2100  _configToggle
       230A  _commandReady
       2845  _takeAction
       29F7  _pingResponse
       2AB8  _unloadAllConfigs
       2AD4  _unloadConfig
       2B5D  _assignedID
       2BFC  _childListen
       2C54  _childResponse
       2D3C  _servoFinder
       2F0A  _servoInstruction
       2FE8  _xmitWait
       3007  _servoBootWait
       305B  _servoConfigWait
       30BA  _TX_01234_TIMEOUT_ISR
       30D1  _HELLO_TIMEOUT_ISR
       30E8  _CHILD_1_TIMEOUT_ISR
       30FF  _CHILD_2_TIMEOUT_ISR
       3116  _CHILD_3_TIMEOUT_ISR
       312D  _CHILD_4_TIMEOUT_ISR
       3144  _INIT_TIMEOUT_ISR
       315B  Dispatch_INTERRUPT_10
       31C7  Dispatch_INTERRUPT_11
       31F1  Dispatch_INTERRUPT_9
       3247  __divmod_16X16_16
       3295  __i_div16_block_util
       32CE  __text_end
       32E1  __xidata_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             TOP   0080   014E =    334. bytes (abs,ovr,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0080  __Start
       014B  bGetPowerSetting
       014B  _bGetPowerSetting

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                    InterruptRAM   0000   0018 =     24. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  _WAIT_RECV_fStatus
       0000  WAIT_RECV_fStatus
       0001  _WAIT_RECV_bRxCnt
       0001  WAIT_RECV_bRxCnt
       0002  _RX8_2_fStatus
       0002  RX8_2_fStatus
       0003  RX8_2_bRxCnt
       0003  _RX8_2_bRxCnt
       0004  ACTIVE_CONFIG_STATUS
       0004  NO_SHADOW
       0004  _NO_SHADOW
       0006  INIT_RX_fStatus
       0006  _INIT_RX_fStatus
       0007  INIT_RX_bRxCnt
       0007  _INIT_RX_bRxCnt
       0008  _HELLO_4_fStatus
       0008  HELLO_4_fStatus
       0009  HELLO_4_bRxCnt
       0009  _HELLO_4_bRxCnt
       000A  HELLO_3_fStatus
       000A  _HELLO_3_fStatus
       000B  _HELLO_3_bRxCnt
       000B  HELLO_3_bRxCnt
       000C  HELLO_2_fStatus
       000C  _HELLO_2_fStatus
       000D  HELLO_2_bRxCnt
       000D  _HELLO_2_bRxCnt
       000E  _HELLO_1_fStatus
       000E  HELLO_1_fStatus
       000F  HELLO_1_bRxCnt
       000F  _HELLO_1_bRxCnt
       0010  CHILD_4_fStatus
       0010  _CHILD_4_fStatus
       0011  _CHILD_4_bRxCnt
       0011  CHILD_4_bRxCnt
       0012  _CHILD_3_fStatus
       0012  CHILD_3_fStatus
       0013  _CHILD_3_bRxCnt
       0013  CHILD_3_bRxCnt
       0014  _CHILD_2_fStatus
       0014  CHILD_2_fStatus
       0015  _CHILD_2_bRxCnt
       0015  CHILD_2_bRxCnt
       0016  CHILD_1_fStatus
       0016  _CHILD_1_fStatus
       0017  _CHILD_1_bRxCnt
       0017  CHILD_1_bRxCnt

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     CHILD_1_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0180  _CHILD_1_aRxBuffer
       0180  CHILD_1_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     CHILD_2_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0140  _CHILD_2_aRxBuffer
       0140  CHILD_2_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     CHILD_3_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0058  _CHILD_3_aRxBuffer
       0058  CHILD_3_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     CHILD_4_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  _CHILD_4_aRxBuffer
       0100  CHILD_4_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     HELLO_1_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       01D0  _HELLO_1_aRxBuffer
       01D0  HELLO_1_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     HELLO_2_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       01E0  _HELLO_2_aRxBuffer
       01E0  HELLO_2_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     HELLO_3_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00D8  _HELLO_3_aRxBuffer
       00D8  HELLO_3_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     HELLO_4_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00E8  _HELLO_4_aRxBuffer
       00E8  HELLO_4_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     INIT_RX_RAM   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       01C0  INIT_RX_aRxBuffer
       01C0  _INIT_RX_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                       RX8_2_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0018  RX8_2_aRxBuffer
       0018  _RX8_2_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                   WAIT_RECV_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0098  WAIT_RECV_aRxBuffer
       0098  _WAIT_RECV_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
               virtual_registers   00F8   0006 =      6. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00F8  __rZ
       00F9  __rY
       00FA  __rX
       00FB  __r3
       00FC  __r1
       00FD  __r0

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0000   000F =     15. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  __data_start
       00FE  _CONFIGURED
       01F0  _STATE
       01F2  _TIMEOUT
       01F4  _CHILD
       01FF  _COMMAND_ERROR
       0200  _COMMAND_LENGTH
       0201  _COMMAND_PARAM
       0202  _COMMAND_TYPE
       0203  _COMMAND_DESTINATION
       0204  _COMMAND_SOURCE
       0205  _ID
       0207  _SERVO_ID

Files Linked      [ module(s) ]

./obj/boot.o	[ boot.asm ]
./obj/main.o	[ main.c ]
<library>	[ wait_recvint.asm, wait_recv.asm, tx_23int.asm, tx_23.asm, tx_014int.asm, tx_014.asm, tx_01234_timeout.asm ]
[ servo_txint.asm, servo_tx.asm, rx8_2int.asm, rx8_2.asm, psocdynamicint.asm, psocconfigtbl.asm ]
[ psocconfig.asm, init_timeout.asm, init_rxint.asm, init_rx.asm, hello_timeout.asm, hello_4int.asm ]
[ hello_4.asm, hello_3int.asm, hello_3.asm, hello_2int.asm, hello_2.asm, hello_1int.asm, hello_1.asm ]
[ child_4int.asm, child_4_timeout.asm, child_4.asm, child_3int.asm, child_3_timeout.asm, child_3.asm ]
[ child_2int.asm, child_2_timeout.asm, child_2.asm, child_1int.asm, child_1_timeout.asm, child_1.asm ]
[ rZ.asm, rY.asm, rX.asm, r3.asm, r1.asm, r0.asm, divmod_16x16_16_util.asm, i_div16_block_util.asm ]


User Base Address Definitions

lit = 0x190
data = 0x0
SSCParmBlk:0x00F7.0x00FF
InterruptRAM = 0x0
virtual_registers = 0xf8
waiting_RAM
hello_RAM = 0x208
initial_RAM = 0x208
my_response_RAM = 0x208
response1_RAM = 0x208
response2_RAM = 0x208
response3_RAM = 0x208
response4_RAM = 0x208
servo_transmit_RAM


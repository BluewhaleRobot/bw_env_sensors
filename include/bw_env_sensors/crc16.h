#ifndef CRC16_H
#define CRC16_H

namespace bw_env_sensors
{
  void CRC16CheckSum(uint8_t *pDataIn, int len, uint8_t *sum);
  void CRC16CheckSum(uint8_t *pDataIn, int len, uint16_t *sum);
} //namespace bw_env_sensors
#endif // CRC16_H

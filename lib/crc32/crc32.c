#include "crc32.h"

WORD32 crc32 (WORD32 crc, const BYTE *buf, WORD32 len)
{

    crc = crc ^ 0xffffffffL;
    while (len >= 8)
    {
      DO8(buf);
      len -= 8;
    }
    if (len) do {
      DO1(buf);
    } while (--len);
    return crc ^ 0xffffffffL;
}

#include "unpack.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>


#define checksumByte(i) data[i] << (8 * ((i + 1) % 2))

extern const uint8_t LDS_UNIVERSAL_KEY[16];

/**
 *  Check for universal key, return 1 on error
 */
int check_universal_key(unsigned short *checksum, size_t *i, unsigned char *data)
{
  for (size_t k = 0; k < 16; k++)
  {
    if (LDS_UNIVERSAL_KEY[k] != data[*i])
      return WRONG_UNIVERSAL_KEY;
    *checksum += checksumByte(*i);
    (*i)++;
  }

  return OK;
}

/**
 *  Retrieve LDS length, its encoding relies on short/long BER encoding.
 */
size_t packet_length(unsigned short *checksum, size_t *i, unsigned char *data)
{
  *checksum += checksumByte(*i);
  unsigned char first_byte = data[(*i)++];

  // Short BER
  if (first_byte <= 127)
    return first_byte;
  // Long BER
  else
  {
    size_t size = 0;
    unsigned char nb_subsequent_bytes = first_byte & ~(1 << 7);

    for (int b = 0; b < nb_subsequent_bytes; b++)
    {
      size = size << 8;
      *checksum += checksumByte(*i);
      size += (unsigned char)data[(*i)++];
    }

    return size;
  }
}

/**
 *  - Check Universal Key.
 *  - Get length.
 *  - Decode and add each KLVs into `klvmap`.
 *  - Check if UNIX Timestamp, Checksum and ULS version KLV are presents.
 */
int unpack_misb_raw(unsigned char* data, size_t size, struct KLVRawMap* klvmap) {
    char contain_timestamp_klv = 0;
    char contain_lds_version_klv = 0;
    unsigned short packet_checksum_klv = 0;

    unsigned short expected_checksum = 0;

    size_t i = 0;

    if (check_universal_key(&expected_checksum, &i, data)) {
        return WRONG_UNIVERSAL_KEY;
    }

    packet_length(&expected_checksum, &i, data);

    while (i < size) {
        struct KLVRaw* klv = malloc(sizeof(struct KLVRaw));

        expected_checksum += checksumByte(i);
        enum Tags klv_tag = data[i++];

        expected_checksum += checksumByte(i);
        unsigned char klv_size = data[i++];

        klv->tag = klv_tag;
        klv->rawSize = klv_size;
        klv->rawBytes = malloc(klv_size);

        memcpy(klv->rawBytes, &data[i], klv_size);

        for (size_t y = 0; y < klv_size; y++) {
            if (klv_tag != CHECKSUM) {
                expected_checksum += checksumByte(i);
            }
            else {
                packet_checksum_klv = (packet_checksum_klv << 8) + data[i];
            }
            i++;
        }

        klvmap->KLVs[klv_tag] = klv;

        if (klv_tag == UNIX_TIME_STAMP) contain_timestamp_klv = 1;
        if (klv_tag == UAS_LDS_VERSION_NUMBER) contain_lds_version_klv = 1;
    }

    if (!contain_timestamp_klv) return NO_TIMESTAMP;
    if (!contain_lds_version_klv) return NO_LDS_VERSION;
    if (!packet_checksum_klv) return NO_CHECKSUM;
    if (packet_checksum_klv != expected_checksum) return WRONG_CHECKSUM;

    return OK;
}


struct KLVRaw* get_klv_by_tag(const struct KLVRawMap* klvmap, int tag) {
    if (!klvmap || tag < 0 || tag >= 94) return NULL;
    return klvmap->KLVs[tag];
}


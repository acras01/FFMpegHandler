#include "unpack.h"
#include "utils.h"

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

struct GenericValue decode_value(const struct Field field, unsigned char *value, uint8_t size)
{
  union Offset {
    float foffset;
    double doffset;
  } offset;

  struct GenericValue result;
  result.type = field.value_format;

  // Conversion if needed
  if (field.value_format != field.encoded_format)
  {
    switch (field.encoded_format)
    {
    case F_UINT16:
      offset.foffset = (field.range.min.float_value < 0) ? -field.range.min.float_value : 0;
      result.float_value = int16_to_unsigned_dec((unsigned short)*value,
                                                  fabs(field.range.min.float_value) +
                                                  fabs(field.range.max.float_value),
                                                  offset.foffset);
      return result;

    case F_UINT32:
      offset.doffset = (field.range.min.double_value < 0) ? -field.range.min.double_value : 0;
      result.double_value = int32_to_unsigned_dec((int)*value,
                                                  fabs(field.range.min.double_value) +
                                                  fabs(field.range.max.double_value),
                                                  offset.doffset);
      return result;

    case F_INT16:
      result.float_value = int16_to_signed_dec((unsigned)*value,
                                                fabs(field.range.min.float_value) +
                                                fabs(field.range.max.float_value));
      return result;

    case F_INT32:
      result.double_value = int32_to_signed_dec((int)*value,
                                                fabs(field.range.min.double_value) +
                                                fabs(field.range.max.double_value));
      return result;

    default:
      // TODO implement missing conversions
      return result;
    }
  }
  else
  {
    switch (field.value_format)
    {
    case F_UINT8:
      result.uint8_value = (uint8_t)*value;
      return result;
    case F_UINT16:
      result.uint8_value = (uint16_t)*value;
      return result;
    case F_UINT32:
      result.uint8_value = (uint32_t)*value;
      return result;
    case F_UINT64:
      result.uint8_value = (uint64_t)*value;
      return result;
    case F_INT8:
      result.uint8_value = (int8_t)*value;
      return result;
    case F_INT16:
      result.uint8_value = (int16_t)*value;
      return result;
    case F_INT32:
      result.uint8_value = (int32_t)*value;
      return result;
    case F_INT64:
      result.int64_value = (int64_t)*value;
      return result;
    case F_CHAR_P:
      result.charp_value = malloc((size + 1) * sizeof(char));
      memcpy(result.charp_value, (char *)value, size);
      result.charp_value[size] = '\0';
      return result;
    default :
      // TODO Handle properly error
      return result;
    }
  }
}


/**
 *  - Check Universal Key.
 *  - Get length.
 *  - Decode and add each KLVs into `klvmap`.
 *  - Check if UNIX Timestamp, Checksum and ULS version KLV are presents.
 */
int unpack_misb(unsigned char *data, size_t size, struct KLVMap *klvmap)
{
  // Mandatory KLVs
  char contain_timestamp_klv = 0;
  char contain_lds_version_klv = 0;
  unsigned short packet_checksum_klv = 0;
 
  unsigned short expected_checksum = 0;

  size_t i = 0;

  // Check Universal Key
  if (check_universal_key(&expected_checksum, &i, data))
    return WRONG_UNIVERSAL_KEY;

  // Retrieve packet length
  packet_length(&expected_checksum, &i, data);

  // KLVs
  while (i < size)
  {
    struct KLV *klv = malloc(sizeof(struct KLV));

    // Max tag value is 93, so it is encoded on 1 byte.
    expected_checksum += checksumByte(i);
    enum Tags klv_tag = data[i++];

    // Max length value is lower than 255 bytes so it is also encoded in 1 byte.
    expected_checksum += checksumByte(i);
    unsigned char klv_size = data[i++];

    klv->tag = klv_tag;
    klv->size = klv_size;
    struct GenericValue value = decode_value(FieldMap[klv_tag], data + i, klv_size);
    klv->value = value;

    // Calculate checksum for the content of the KLV.
    for (size_t y = 0; y < klv_size; y++)
    {
      if (klv_tag != CHECKSUM)
        expected_checksum += checksumByte(i);
      else
      {
        packet_checksum_klv = packet_checksum_klv << 8;
        packet_checksum_klv += data[i];
      }
      i++;
    }

    if (klv_tag == UNIX_TIME_STAMP) contain_timestamp_klv = 1;
    if (klv_tag == UAS_LDS_VERSION_NUMBER) contain_lds_version_klv = 1;

    klvmap->KLVs[klv_tag] = klv;
  }

  if (!contain_timestamp_klv)
    return NO_TIMESTAMP;
  if (!contain_lds_version_klv)
    return NO_LDS_VERSION;
  if (!packet_checksum_klv)
    return NO_CHECKSUM;
  if (packet_checksum_klv != expected_checksum)
    return WRONG_CHECKSUM;

  return OK;
}

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


struct KLV* get_klv_by_tag(const struct KLVMap* klvmap, int tag) {
    if (!klvmap || tag < 0 || tag >= 94) return NULL;
    return klvmap->KLVs[tag];
}

void free_klvmap(struct KLVMap* klvmap) {
    if (!klvmap) return;

    for (int i = 0; i < 94; i++) {
        if (klvmap->KLVs[i]) {
            free(klvmap->KLVs[i]);
            klvmap->KLVs[i] = NULL;
        }
    }
    free(klvmap);
}


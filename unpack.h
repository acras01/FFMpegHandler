#ifndef UNPACK_H
#define UNPACK_H

#include "keys.h"

#include <stddef.h>

#define OK                  0
#define WRONG_UNIVERSAL_KEY 1
#define NO_TIMESTAMP        2
#define NO_LDS_VERSION      3
#define NO_CHECKSUM         4
#define WRONG_CHECKSUM      5

/**
 *  KLV structure
 */
struct KLVRaw {
    enum Tags tag;
    uint8_t* rawBytes;
    size_t rawSize;
};

/**
 *  Map structure to retrieve KLV
 */
struct KLVRawMap {
    struct KLVRaw* KLVs[94];
};

/**
 *  Unpack the given data and fill the `klvmap` in parameter.
 *  Return 0 on success and positive value on error.
 *  Errors :
 *    - 1 : Universal Key is wrong.
 *    - 2 : No UNIX Timestamp KLV found.
 *    - 3 : No ULS Version KLV found.
 *    - 4 : No Checksum KLV found.
 *    - 5 : Checksum KLV is wrong.
 *
 *  The `klvmap` should be filled with NULL values.
 *  KLV `data` field inside KLVMap are pointer toward the `data` memory.
 *  Thus, freeing `data` also free `data` inside KLVs.
 *  Despite this fact, you need to free the KLV struct inside the map.
 */
int unpack_misb_raw(unsigned char *data, size_t size, struct KLVRawMap *klvmap);

struct KLVRaw* get_klv_by_tag(const struct KLVRawMap* klvmap, int tag);

#endif /* !UNPACK_H */

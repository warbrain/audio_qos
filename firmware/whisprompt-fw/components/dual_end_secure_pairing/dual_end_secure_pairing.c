#include "dual_end_secure_pairing.h"

#include <string.h>

#include "nvs.h"

#define PAIRING_LEGACY_MAGIC 0x50414952u
#define PAIRING_ACTIVE_MAGIC 0x32414950u
#define PAIRING_PENDING_MAGIC 0x32444e50u
#define PAIRING_LEGACY_VERSION 1u
#define PAIRING_STORE_VERSION 2u
#define PAIRING_FLAG_ENCRYPTED 0x01u

#define PAIRING_NVS_NAMESPACE "pairing"
#define PAIRING_NVS_LEGACY_KEY "peer"
#define PAIRING_NVS_ACTIVE_KEY "active"
#define PAIRING_NVS_PENDING_KEY "pending"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t peer_mac[DUAL_END_PAIRING_MAC_BYTES];
    uint8_t reserved;
} pairing_legacy_record_t;

_Static_assert(sizeof(pairing_legacy_record_t) == 12,
               "legacy pairing/peer layout changed");
_Static_assert(sizeof(dual_end_pairing_active_record_t) == 60,
               "pairing/active layout changed");
_Static_assert(sizeof(dual_end_pairing_pending_record_t) == 132,
               "pairing/pending layout changed");

static bool peer_mac_is_valid(const uint8_t mac[DUAL_END_PAIRING_MAC_BYTES])
{
    if (!mac || (mac[0] & 0x01u) != 0) {
        return false;
    }
    bool all_zero = true;
    bool all_ff = true;
    for (size_t i = 0; i < DUAL_END_PAIRING_MAC_BYTES; i++) {
        all_zero = all_zero && mac[i] == 0;
        all_ff = all_ff && mac[i] == 0xff;
    }
    return !all_zero && !all_ff;
}

bool dual_end_pairing_secret_is_valid(const uint8_t *value, size_t length)
{
    if (!value || length == 0) {
        return false;
    }
    bool all_zero = true;
    bool all_ff = true;
    for (size_t i = 0; i < length; i++) {
        all_zero = all_zero && value[i] == 0;
        all_ff = all_ff && value[i] == 0xff;
    }
    return !all_zero && !all_ff;
}

bool dual_end_pairing_active_is_valid(const dual_end_pairing_active_record_t *record)
{
    if (!record || record->magic != PAIRING_ACTIVE_MAGIC ||
        record->version != PAIRING_STORE_VERSION ||
        (record->flags & ~PAIRING_FLAG_ENCRYPTED) != 0 ||
        !peer_mac_is_valid(record->peer_mac)) {
        return false;
    }
    if ((record->flags & PAIRING_FLAG_ENCRYPTED) == 0) {
        return true;
    }
    return record->generation != 0 && record->transaction_id != 0 &&
           dual_end_pairing_secret_is_valid(record->lmk, sizeof(record->lmk)) &&
           dual_end_pairing_secret_is_valid(record->pmk, sizeof(record->pmk)) &&
           dual_end_pairing_secret_is_valid(record->key_id, sizeof(record->key_id));
}

dual_end_pairing_security_t dual_end_pairing_active_security(
    const dual_end_pairing_active_record_t *record)
{
    if (!dual_end_pairing_active_is_valid(record)) {
        return DUAL_END_PAIRING_SECURITY_NONE;
    }
    return (record->flags & PAIRING_FLAG_ENCRYPTED) != 0 ?
           DUAL_END_PAIRING_SECURITY_ENCRYPTED :
           DUAL_END_PAIRING_SECURITY_LEGACY_PLAIN;
}

bool dual_end_pairing_pending_is_valid(const dual_end_pairing_pending_record_t *record)
{
    return record && record->magic == PAIRING_PENDING_MAGIC &&
           record->version == PAIRING_STORE_VERSION &&
           (record->state == DUAL_END_PAIRING_PENDING_PREPARED ||
            record->state == DUAL_END_PAIRING_PENDING_COMMITTED) &&
           record->transaction_id != 0 &&
           record->transaction_id == record->candidate.transaction_id &&
           dual_end_pairing_active_is_valid(&record->candidate) &&
           (!record->previous_valid ||
            dual_end_pairing_active_is_valid(&record->previous));
}

bool dual_end_pairing_records_equal(const dual_end_pairing_active_record_t *left,
                                    const dual_end_pairing_active_record_t *right)
{
    return left && right && memcmp(left, right, sizeof(*left)) == 0;
}

void dual_end_pairing_make_legacy(dual_end_pairing_active_record_t *record,
                                  const uint8_t peer_mac[DUAL_END_PAIRING_MAC_BYTES])
{
    if (!record) {
        return;
    }
    memset(record, 0, sizeof(*record));
    record->magic = PAIRING_ACTIVE_MAGIC;
    record->version = PAIRING_STORE_VERSION;
    if (peer_mac) {
        memcpy(record->peer_mac, peer_mac, sizeof(record->peer_mac));
    }
}

void dual_end_pairing_make_secure(
    dual_end_pairing_active_record_t *record,
    const uint8_t peer_mac[DUAL_END_PAIRING_MAC_BYTES],
    const uint8_t lmk[DUAL_END_PAIRING_KEY_BYTES],
    const uint8_t pmk[DUAL_END_PAIRING_KEY_BYTES],
    uint32_t generation,
    uint32_t transaction_id,
    const uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES])
{
    if (!record) {
        return;
    }
    memset(record, 0, sizeof(*record));
    record->magic = PAIRING_ACTIVE_MAGIC;
    record->version = PAIRING_STORE_VERSION;
    record->flags = PAIRING_FLAG_ENCRYPTED;
    record->generation = generation;
    record->transaction_id = transaction_id;
    if (peer_mac) {
        memcpy(record->peer_mac, peer_mac, sizeof(record->peer_mac));
    }
    if (lmk) {
        memcpy(record->lmk, lmk, sizeof(record->lmk));
    }
    if (pmk) {
        memcpy(record->pmk, pmk, sizeof(record->pmk));
    }
    if (key_id) {
        memcpy(record->key_id, key_id, sizeof(record->key_id));
    }
}

void dual_end_pairing_make_pending(
    dual_end_pairing_pending_record_t *record,
    const dual_end_pairing_active_record_t *candidate,
    const dual_end_pairing_active_record_t *previous)
{
    if (!record) {
        return;
    }
    memset(record, 0, sizeof(*record));
    record->magic = PAIRING_PENDING_MAGIC;
    record->version = PAIRING_STORE_VERSION;
    record->state = DUAL_END_PAIRING_PENDING_PREPARED;
    if (candidate) {
        record->transaction_id = candidate->transaction_id;
        record->candidate = *candidate;
    }
    if (previous && dual_end_pairing_active_is_valid(previous)) {
        record->previous_valid = 1;
        record->previous = *previous;
    }
}

static esp_err_t save_blob(const char *key, const void *blob, size_t length)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(PAIRING_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_blob(handle, key, blob, length);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static esp_err_t erase_key(const char *key)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(PAIRING_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_erase_key(handle, key);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

esp_err_t dual_end_pairing_store_save_active(
    const dual_end_pairing_active_record_t *record)
{
    if (!dual_end_pairing_active_is_valid(record)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(PAIRING_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_blob(handle, PAIRING_NVS_ACTIVE_KEY,
                       record, sizeof(*record));
    if (ret == ESP_OK) {
        /* Never leave the old plaintext peer record as a downgrade fallback
         * after an active v2 record has been committed. */
        esp_err_t erase_ret = nvs_erase_key(handle, PAIRING_NVS_LEGACY_KEY);
        if (erase_ret != ESP_OK && erase_ret != ESP_ERR_NVS_NOT_FOUND) {
            ret = erase_ret;
        }
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

esp_err_t dual_end_pairing_store_erase_active(void)
{
    return erase_key(PAIRING_NVS_ACTIVE_KEY);
}

bool dual_end_pairing_store_load_active(dual_end_pairing_active_record_t *record,
                                        bool *migrated_legacy)
{
    if (migrated_legacy) {
        *migrated_legacy = false;
    }
    if (!record) {
        return false;
    }
    nvs_handle_t handle;
    if (nvs_open(PAIRING_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t length = sizeof(*record);
    esp_err_t ret = nvs_get_blob(handle, PAIRING_NVS_ACTIVE_KEY,
                                 record, &length);
    if (ret == ESP_OK) {
        bool valid = length == sizeof(*record) &&
                     dual_end_pairing_active_is_valid(record);
        nvs_close(handle);
        if (!valid) {
            /* An invalid v2 record is corruption, not permission to fall
             * back to the legacy plaintext key. Fail closed. */
            memset(record, 0, sizeof(*record));
        }
        return valid;
    }
    if (ret != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        memset(record, 0, sizeof(*record));
        return false;
    }

    pairing_legacy_record_t legacy = {0};
    length = sizeof(legacy);
    ret = nvs_get_blob(handle, PAIRING_NVS_LEGACY_KEY, &legacy, &length);
    nvs_close(handle);
    if (ret != ESP_OK || length != sizeof(legacy) ||
        legacy.magic != PAIRING_LEGACY_MAGIC ||
        legacy.version != PAIRING_LEGACY_VERSION ||
        !peer_mac_is_valid(legacy.peer_mac)) {
        return false;
    }

    dual_end_pairing_make_legacy(record, legacy.peer_mac);
    if (dual_end_pairing_store_save_active(record) == ESP_OK) {
        (void)erase_key(PAIRING_NVS_LEGACY_KEY);
        if (migrated_legacy) {
            *migrated_legacy = true;
        }
    }
    return true;
}

esp_err_t dual_end_pairing_store_save_pending(
    const dual_end_pairing_pending_record_t *record)
{
    if (!dual_end_pairing_pending_is_valid(record)) {
        return ESP_ERR_INVALID_ARG;
    }
    return save_blob(PAIRING_NVS_PENDING_KEY, record, sizeof(*record));
}

bool dual_end_pairing_store_load_pending(dual_end_pairing_pending_record_t *record)
{
    if (!record) {
        return false;
    }
    nvs_handle_t handle;
    if (nvs_open(PAIRING_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t length = sizeof(*record);
    esp_err_t ret = nvs_get_blob(handle, PAIRING_NVS_PENDING_KEY,
                                 record, &length);
    nvs_close(handle);
    return ret == ESP_OK && length == sizeof(*record) &&
           dual_end_pairing_pending_is_valid(record);
}

esp_err_t dual_end_pairing_store_erase_pending(void)
{
    return erase_key(PAIRING_NVS_PENDING_KEY);
}

esp_err_t dual_end_pairing_store_clear_all(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(PAIRING_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    const char *keys[] = {
        PAIRING_NVS_ACTIVE_KEY,
        PAIRING_NVS_PENDING_KEY,
        PAIRING_NVS_LEGACY_KEY,
    };
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        esp_err_t erase_ret = nvs_erase_key(handle, keys[i]);
        if (erase_ret != ESP_OK && erase_ret != ESP_ERR_NVS_NOT_FOUND) {
            ret = erase_ret;
            break;
        }
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DUAL_END_PAIRING_KEY_BYTES 16u
#define DUAL_END_PAIRING_KEY_ID_BYTES 8u
#define DUAL_END_PAIRING_MAC_BYTES 6u

typedef enum {
    DUAL_END_PAIRING_SECURITY_NONE = 0,
    DUAL_END_PAIRING_SECURITY_LEGACY_PLAIN = 1,
    DUAL_END_PAIRING_SECURITY_ENCRYPTED = 2,
} dual_end_pairing_security_t;

typedef enum {
    DUAL_END_PAIRING_PENDING_NONE = 0,
    DUAL_END_PAIRING_PENDING_PREPARED = 1,
    DUAL_END_PAIRING_PENDING_COMMITTED = 2,
} dual_end_pairing_pending_state_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t flags;
    uint8_t peer_mac[DUAL_END_PAIRING_MAC_BYTES];
    uint8_t lmk[DUAL_END_PAIRING_KEY_BYTES];
    uint8_t pmk[DUAL_END_PAIRING_KEY_BYTES];
    uint32_t generation;
    uint32_t transaction_id;
    uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES];
} dual_end_pairing_active_record_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t state;
    uint8_t previous_valid;
    uint8_t reserved;
    uint32_t transaction_id;
    dual_end_pairing_active_record_t candidate;
    dual_end_pairing_active_record_t previous;
} dual_end_pairing_pending_record_t;

typedef struct {
    bool paired;
    bool from_nvs;
    dual_end_pairing_security_t security;
    uint8_t peer_mac[DUAL_END_PAIRING_MAC_BYTES];
    uint32_t generation;
    uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES];
    dual_end_pairing_pending_state_t pending_state;
    uint32_t pending_transaction_id;
} dual_end_pairing_status_t;

bool dual_end_pairing_secret_is_valid(const uint8_t *value, size_t length);
bool dual_end_pairing_active_is_valid(const dual_end_pairing_active_record_t *record);
dual_end_pairing_security_t dual_end_pairing_active_security(
    const dual_end_pairing_active_record_t *record);
bool dual_end_pairing_pending_is_valid(const dual_end_pairing_pending_record_t *record);
bool dual_end_pairing_records_equal(const dual_end_pairing_active_record_t *left,
                                    const dual_end_pairing_active_record_t *right);

void dual_end_pairing_make_legacy(dual_end_pairing_active_record_t *record,
                                  const uint8_t peer_mac[DUAL_END_PAIRING_MAC_BYTES]);
void dual_end_pairing_make_secure(
    dual_end_pairing_active_record_t *record,
    const uint8_t peer_mac[DUAL_END_PAIRING_MAC_BYTES],
    const uint8_t lmk[DUAL_END_PAIRING_KEY_BYTES],
    const uint8_t pmk[DUAL_END_PAIRING_KEY_BYTES],
    uint32_t generation,
    uint32_t transaction_id,
    const uint8_t key_id[DUAL_END_PAIRING_KEY_ID_BYTES]);
void dual_end_pairing_make_pending(
    dual_end_pairing_pending_record_t *record,
    const dual_end_pairing_active_record_t *candidate,
    const dual_end_pairing_active_record_t *previous);

bool dual_end_pairing_store_load_active(dual_end_pairing_active_record_t *record,
                                        bool *migrated_legacy);
esp_err_t dual_end_pairing_store_save_active(
    const dual_end_pairing_active_record_t *record);
esp_err_t dual_end_pairing_store_erase_active(void);
bool dual_end_pairing_store_load_pending(dual_end_pairing_pending_record_t *record);
esp_err_t dual_end_pairing_store_save_pending(
    const dual_end_pairing_pending_record_t *record);
esp_err_t dual_end_pairing_store_erase_pending(void);
esp_err_t dual_end_pairing_store_clear_all(void);

#ifdef __cplusplus
}
#endif

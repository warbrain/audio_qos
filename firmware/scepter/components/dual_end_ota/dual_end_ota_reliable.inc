/* Reliable application OTA backport from firmware-0.4.1. This file is kept
 * identical in both projects; no board, animation or provisioning dependency. */

static esp_err_t pending_result_write(const ota_pending_result_t *record)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open("ota_result", NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;
    ret = record ? nvs_set_blob(handle, "pending", record, sizeof(*record)) :
                   nvs_erase_key(handle, "pending");
    if (ret == ESP_ERR_NVS_NOT_FOUND) ret = ESP_OK;
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

static esp_err_t persist_wireless_install_result(void)
{
    if (!s_wireless_session) return ESP_OK;
    esp_app_desc_t desc = {0};
    ESP_RETURN_ON_ERROR(esp_ota_get_partition_description(s_update_partition, &desc),
                        TAG, "read OTA result description");
    ota_pending_result_t record = {
        .magic = 0x4f544152,
        .session_id = s_status.session_id,
        .image_size = s_status.image_size,
        .image_crc32 = s_status.image_crc32,
    };
    snprintf(record.partition, sizeof(record.partition), "%.15s", s_update_partition->label);
    snprintf(record.app_version, sizeof(record.app_version), "%s", desc.version);
    record.crc32 = dual_end_ota_crc32_update(0, (const uint8_t *)&record,
                                            offsetof(ota_pending_result_t, crc32));
    return pending_result_write(&record);
}

esp_err_t dual_end_ota_confirm_running_image(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = ESP_OK;
#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    if (running->subtype != ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        esp_ota_img_states_t state;
        ret = esp_ota_get_state_partition(running, &state);
        if (ret == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
            ret = esp_ota_mark_app_valid_cancel_rollback();
        }
    }
#endif
    s_boot_confirmation_error = ret;
    s_boot_confirmed = true;
    return ret;
}

esp_err_t dual_end_ota_boot_confirmation_error(void)
{
    return s_boot_confirmation_error;
}

bool dual_end_ota_relay_is_active(void)
{
    return s_relay_active;
}

static void cancel_scepter_ack_wait(void)
{
    portENTER_CRITICAL(&s_ack_lock);
    s_waiting_for_ack = false;
    portEXIT_CRITICAL(&s_ack_lock);
}

static esp_err_t send_packet_with_ack(dual_end_ota_packet_t *packet,
                                      uint32_t timeout_ms)
{
    if (!s_ack_sem) return ESP_ERR_INVALID_STATE;
    uint32_t payload_crc = dual_end_ota_crc32_update(0, packet->payload, packet->payload_len);
    esp_err_t ret = ESP_ERR_TIMEOUT;
    for (unsigned attempt = 0; attempt < 6; ++attempt) {
        while (xSemaphoreTake(s_ack_sem, 0) == pdTRUE) {}
        portENTER_CRITICAL(&s_ack_lock);
        s_expected_ack = *packet;
        s_expected_payload_crc = payload_crc;
        s_waiting_for_ack = true;
        portEXIT_CRITICAL(&s_ack_lock);
        ret = send_packet(packet);
        /* A failed MAC callback can still mean the receiver got the frame.
         * Wait for its application ACK before retrying that same frame. */
        bool got_ack = xSemaphoreTake(s_ack_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
        portENTER_CRITICAL(&s_ack_lock);
        bool got_boot_result = packet->msg_type == DUAL_END_OTA_MSG_END && s_result_received;
        portEXIT_CRITICAL(&s_ack_lock);
        if (got_boot_result) {
            cancel_scepter_ack_wait();
            return ESP_OK; /* Caller consumes the result, including a boot failure. */
        }
        if (got_ack) {
            dual_end_ota_ack_payload_t ack;
            portENTER_CRITICAL(&s_ack_lock);
            ack = s_last_ack;
            s_waiting_for_ack = false;
            portEXIT_CRITICAL(&s_ack_lock);
            if (ack.status == DUAL_END_OTA_ACK_ACCEPTED) return ESP_OK;
            ret = ack.error_code != ESP_OK ? ack.error_code : ESP_ERR_INVALID_RESPONSE;
            if (ack.status == DUAL_END_OTA_ACK_REJECTED) return ret;
        } else if (ret == ESP_OK) {
            ret = ESP_ERR_TIMEOUT;
        }
        cancel_scepter_ack_wait();
    }
    return ret;
}

esp_err_t dual_end_ota_prepare_scepter_result(uint32_t session_id)
{
    if (!s_result_sem || !session_id) return ESP_ERR_INVALID_STATE;
    while (xSemaphoreTake(s_result_sem, 0) == pdTRUE) {}
    portENTER_CRITICAL(&s_ack_lock);
    s_result_received = false;
    s_expected_result_session = session_id;
    s_waiting_for_result = true;
    portEXIT_CRITICAL(&s_ack_lock);
    return ESP_OK;
}

void dual_end_ota_cancel_scepter_result_wait(void)
{
    portENTER_CRITICAL(&s_ack_lock);
    s_waiting_for_result = false;
    portEXIT_CRITICAL(&s_ack_lock);
    s_relay_active = false;
    notify_ota_activity(false);
}

esp_err_t dual_end_ota_wait_scepter_result(uint32_t timeout_ms, dual_end_ota_result_t *result)
{
    if (!result || !s_result_sem) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_result_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        dual_end_ota_cancel_scepter_result_wait();
        return ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&s_ack_lock);
    *result = s_last_result;
    s_waiting_for_result = false;
    portEXIT_CRITICAL(&s_ack_lock);
    s_relay_active = false;
    notify_ota_activity(false);
    return result->payload.status == DUAL_END_OTA_RESULT_INSTALL_CONFIRMED ?
        ESP_OK : (result->payload.error_code != ESP_OK ? result->payload.error_code : ESP_FAIL);
}

esp_err_t dual_end_ota_send_scepter_abort(uint32_t session_id)
{
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_ABORT,
        .target = DUAL_END_OTA_TARGET_SCEPTER,
        .session_id = session_id,
    };
    esp_err_t ret = send_packet(&packet);
    dual_end_ota_cancel_scepter_result_wait();
    return ret;
}

static esp_err_t send_wireless_ack(const dual_end_ota_packet_t *received,
                                   esp_err_t error, bool duplicate)
{
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC,
        .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_ACK,
        .target = DUAL_END_OTA_TARGET_DONGLE,
        .reserved = received->target,
        .session_id = received->session_id,
        .sequence = received->sequence,
        .offset = received->offset,
        .image_size = received->image_size,
        .image_crc32 = received->image_crc32,
        .payload_len = sizeof(dual_end_ota_ack_payload_t),
    };
    dual_end_ota_ack_payload_t ack = {
        .version = DUAL_END_OTA_ACK_VERSION,
        .acked_msg_type = received->msg_type,
        .status = error == ESP_OK ? DUAL_END_OTA_ACK_ACCEPTED : DUAL_END_OTA_ACK_REJECTED,
        .duplicate = duplicate,
        .error_code = error,
        .received = s_status.bytes_written,
        .rolling_crc32 = s_running_crc,
        .payload_crc32 = dual_end_ota_crc32_update(0, received->payload, received->payload_len),
    };
    memcpy(packet.payload, &ack, sizeof(ack));
    return send_packet(&packet);
}

static bool packet_matches_session(const dual_end_ota_packet_t *packet)
{
    return s_status.active && s_wireless_session &&
        packet->session_id == s_status.session_id &&
        packet->target == s_status.target &&
        packet->image_size == s_status.image_size &&
        packet->image_crc32 == s_status.image_crc32;
}

static void process_wireless_packet(const dual_end_ota_packet_t *packet)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    bool duplicate = false;
    bool restart = false;
    switch (packet->msg_type) {
    case DUAL_END_OTA_MSG_BEGIN:
        if (packet->payload_len || packet->reserved > 2) {
            ret = ESP_ERR_INVALID_ARG;
        } else if (s_status.active) {
            duplicate = packet_matches_session(packet) &&
                (packet->reserved == 0 || (s_update_partition &&
                 strcmp(s_update_partition->label, partition_code_to_label(packet->reserved)) == 0));
            ret = duplicate ? ESP_OK : ESP_ERR_INVALID_STATE;
        } else {
            /* Stop microphone/HID before erasing or writing flash. */
            notify_ota_activity(true);
            ret = dual_end_ota_self_begin_partition(DUAL_END_OTA_TARGET_SCEPTER,
                packet->session_id, packet->image_size, packet->image_crc32,
                partition_code_to_label(packet->reserved));
            if (ret == ESP_OK) {
                s_wireless_session = true;
                s_scepter_expected_sequence = 0;
                s_have_last_data = false;
            } else {
                notify_ota_activity(false);
            }
        }
        break;
    case DUAL_END_OTA_MSG_DATA:
        if (packet_matches_session(packet) && packet->payload_len > 0) {
            uint32_t crc = dual_end_ota_crc32_update(0, packet->payload, packet->payload_len);
            duplicate = s_have_last_data && packet->sequence == s_last_data.sequence &&
                packet->offset == s_last_data.offset && packet->payload_len == s_last_data.payload_len &&
                crc == s_last_data_crc;
            if (duplicate) {
                ret = ESP_OK;
            } else if (packet->sequence == s_scepter_expected_sequence &&
                       packet->offset == s_status.bytes_written) {
                ret = dual_end_ota_self_write(packet->payload, packet->payload_len);
                if (ret == ESP_OK) {
                    s_last_data = *packet;
                    s_last_data_crc = crc;
                    s_have_last_data = true;
                    ++s_scepter_expected_sequence;
                } else {
                    dual_end_ota_self_abort();
                }
            } else {
                ret = ESP_ERR_INVALID_RESPONSE;
            }
        }
        break;
    case DUAL_END_OTA_MSG_END:
        if (packet_matches_session(packet) && packet->payload_len == 0 &&
            packet->sequence == s_scepter_expected_sequence &&
            packet->offset == s_status.bytes_written) {
            ret = dual_end_ota_self_finish(false);
            restart = ret == ESP_OK;
        }
        break;
    case DUAL_END_OTA_MSG_ABORT:
        if (s_wireless_session && packet->session_id == s_status.session_id) {
            dual_end_ota_self_abort();
            ret = ESP_OK;
        }
        break;
    case DUAL_END_OTA_MSG_ERASE:
        ret = packet->reserved == 1 || packet->reserved == 2 ?
            dual_end_ota_erase_partition(partition_code_to_label(packet->reserved)) : ESP_ERR_INVALID_ARG;
        break;
    case DUAL_END_OTA_MSG_BOOT:
        ret = packet->reserved >= 1 && packet->reserved <= 3 ?
            dual_end_ota_set_boot_partition_label(partition_code_to_label(packet->reserved)) : ESP_ERR_INVALID_ARG;
        break;
    case DUAL_END_OTA_MSG_REBOOT:
        ret = s_status.active ? ESP_ERR_INVALID_STATE : ESP_OK;
        restart = ret == ESP_OK;
        break;
    default:
        return;
    }
    if (ret == ESP_OK && s_wireless_session) s_wireless_activity_us = esp_timer_get_time();
    if (ret == ESP_OK && packet->msg_type == DUAL_END_OTA_MSG_DATA) {
        /* Keep the sender waiting while Wi-Fi and microphone work can settle.
         * Pace accepted duplicates too; they must not bypass this idle window. */
        vTaskDelay(pdMS_TO_TICKS(2) > 0 ? pdMS_TO_TICKS(2) : 1);
    }
    (void)send_wireless_ack(packet, ret, duplicate);
    if (restart) {
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }
}

static bool send_boot_result(const ota_pending_result_t *record)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_app_desc_t *desc = esp_app_get_description();
    bool installed = s_boot_confirmation_error == ESP_OK && running && desc &&
        strcmp(running->label, record->partition) == 0 &&
        strcmp(desc->version, record->app_version) == 0 &&
        strcmp(desc->project_name, "cytoscepter_scepter") == 0;
    dual_end_ota_packet_t packet = {
        .magic = DUAL_END_OTA_MAGIC, .version = DUAL_END_OTA_VERSION,
        .msg_type = DUAL_END_OTA_MSG_RESULT, .target = DUAL_END_OTA_TARGET_DONGLE,
        .session_id = record->session_id, .image_size = record->image_size,
        .image_crc32 = record->image_crc32, .payload_len = sizeof(dual_end_ota_result_payload_t),
    };
    dual_end_ota_result_payload_t result = {
        .version = DUAL_END_OTA_RESULT_VERSION,
        .status = installed ? DUAL_END_OTA_RESULT_INSTALL_CONFIRMED : DUAL_END_OTA_RESULT_INSTALL_FAILED,
        .stage = DUAL_END_OTA_RESULT_STAGE_BOOT_CONFIRM,
        .error_code = installed ? ESP_OK : ESP_ERR_INVALID_STATE,
    };
    snprintf(result.app_version, sizeof(result.app_version), "%s", desc ? desc->version : "unknown");
    snprintf(result.partition, sizeof(result.partition), "%.15s", running ? running->label : "unknown");
    memcpy(packet.payload, &result, sizeof(result));
    unsigned sent = 0;
    for (unsigned i = 0; i < 20; ++i) {
        if (send_packet(&packet) == ESP_OK) ++sent;
        if (sent >= 6) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (sent) (void)pending_result_write(NULL);
    return sent != 0;
}

static void wireless_ota_task(void *arg)
{
    (void)arg;
    /* NVS is initialized by link startup, before application boot confirmation. */
    while (!s_boot_confirmed) vTaskDelay(pdMS_TO_TICKS(50));
    ota_pending_result_t pending = {0};
    nvs_handle_t handle;
    bool have_pending = false;
    if (nvs_open("ota_result", NVS_READONLY, &handle) == ESP_OK) {
        size_t len = sizeof(pending);
        have_pending = nvs_get_blob(handle, "pending", &pending, &len) == ESP_OK &&
            len == sizeof(pending) && pending.magic == 0x4f544152 &&
            pending.crc32 == dual_end_ota_crc32_update(0, (const uint8_t *)&pending,
                offsetof(ota_pending_result_t, crc32));
        nvs_close(handle);
    }
    dual_end_ota_packet_t packet;
    while (true) {
        if (have_pending && s_boot_confirmed && dual_end_espnow_link_is_connected()) {
            have_pending = !send_boot_result(&pending);
        }
        if (xQueueReceive(s_wireless_queue, &packet, pdMS_TO_TICKS(500)) == pdTRUE) {
            process_wireless_packet(&packet);
        }
        if (s_status.active && s_wireless_session &&
            esp_timer_get_time() - s_wireless_activity_us > 60000000LL) {
            ESP_LOGW(TAG, "wireless OTA receive timed out; releasing paused audio");
            dual_end_ota_self_abort();
        }
    }
}

bool dual_end_ota_handle_scepter_espnow_packet(const uint8_t *data, int len)
{
    const dual_end_ota_packet_t *packet = (const dual_end_ota_packet_t *)data;
    if (len <= 0 || !dual_end_ota_packet_valid(packet, (size_t)len)) return false;
    if (s_service_role == SCEPTER_ROLE_DONGLE) {
        if (packet->target != DUAL_END_OTA_TARGET_DONGLE) return true;
        if (packet->msg_type == DUAL_END_OTA_MSG_ACK &&
            packet->payload_len == sizeof(dual_end_ota_ack_payload_t)) {
            dual_end_ota_ack_payload_t ack;
            memcpy(&ack, packet->payload, sizeof(ack));
            bool accepted = false;
            portENTER_CRITICAL(&s_ack_lock);
            if (s_waiting_for_ack && ack.version == DUAL_END_OTA_ACK_VERSION &&
                packet->session_id == s_expected_ack.session_id &&
                packet->sequence == s_expected_ack.sequence && packet->offset == s_expected_ack.offset &&
                packet->image_size == s_expected_ack.image_size && packet->image_crc32 == s_expected_ack.image_crc32 &&
                packet->reserved == s_expected_ack.target && ack.acked_msg_type == s_expected_ack.msg_type &&
                ack.payload_crc32 == s_expected_payload_crc) {
                uint32_t expected_received = packet->offset + s_expected_ack.payload_len;
                if (ack.status != DUAL_END_OTA_ACK_ACCEPTED ||
                    s_expected_ack.msg_type != DUAL_END_OTA_MSG_DATA || ack.received == expected_received) {
                    s_last_ack = ack;
                    accepted = true;
                }
            }
            portEXIT_CRITICAL(&s_ack_lock);
            if (accepted) xSemaphoreGive(s_ack_sem);
        } else if (packet->msg_type == DUAL_END_OTA_MSG_RESULT &&
                   packet->payload_len == sizeof(dual_end_ota_result_payload_t)) {
            dual_end_ota_result_payload_t result;
            memcpy(&result, packet->payload, sizeof(result));
            result.app_version[sizeof(result.app_version) - 1] = '\0';
            result.partition[sizeof(result.partition) - 1] = '\0';
            bool accepted = false;
            portENTER_CRITICAL(&s_ack_lock);
            if (s_waiting_for_result && packet->session_id == s_expected_result_session &&
                packet->image_size == s_relay_image_size && packet->image_crc32 == s_relay_image_crc &&
                result.version == DUAL_END_OTA_RESULT_VERSION &&
                result.stage == DUAL_END_OTA_RESULT_STAGE_BOOT_CONFIRM) {
                s_result_received = true;
                s_last_result.session_id = packet->session_id;
                s_last_result.payload = result;
                accepted = true;
            }
            portEXIT_CRITICAL(&s_ack_lock);
            if (accepted) {
                xSemaphoreGive(s_result_sem);
                /* Boot RESULT is stronger evidence than a lost pre-reboot END ACK. */
                xSemaphoreGive(s_ack_sem);
            }
        }
        return true;
    }
    if (packet->target != DUAL_END_OTA_TARGET_SCEPTER) return true;
    if (s_wireless_queue) {
        dual_end_ota_packet_t copy = {0};
        memcpy(&copy, packet, sizeof(*packet) - DUAL_END_OTA_MAX_PAYLOAD + packet->payload_len);
        /* Never block the Wi-Fi callback. Lost queue entries are retransmitted. */
        (void)xQueueSend(s_wireless_queue, &copy, 0);
    }
    return true;
}

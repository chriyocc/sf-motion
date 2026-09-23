#include "sf_motion_usb_com.h"
#include "string.h"

void sfm_usb_com_init(sfm_usb_com_t *usb, int (*send_data)(uint8_t*, uint16_t)) {
    usb->send_data = send_data;
}

int8_t sfm_usb_com_start_send_data(sfm_usb_com_t *usb, uint8_t *data, uint16_t len) {
    if (len > SFM_USB_COM_DATA_TX_MAX - 3) return -1;
    const uint16_t frame_header = SFM_USB_COM_FRAME_HEADER;
    uint16_t data_offset = 0;
    memcpy(usb->data_tx, &frame_header, sizeof(frame_header));
    data_offset += sizeof(frame_header);
    uint8_t data_len = (uint8_t)len;
    memcpy(usb->data_tx + data_offset, &data_len, sizeof(data_len));
    data_offset += sizeof(data_len);
    memcpy(usb->data_tx + data_offset, data, data_len);
    data_offset += data_len;
    if (usb->send_data(usb->data_tx, data_offset) == 0) return 0;
    return -1;
}

int8_t sfm_usb_com_receive_data(sfm_usb_com_t *usb, uint8_t *data_out, uint16_t *data_out_len) {
    if (usb->data_rx_len < 3) return -1;
    uint16_t frame_header;
    memcpy(&frame_header, usb->data_rx, sizeof(frame_header));
    if (frame_header == SFM_USB_COM_FRAME_HEADER) {
        uint8_t len = usb->data_rx[2];
        if (len > 0) {
            memcpy(data_out, usb->data_rx + 3, len);
        }
        else return -1;
        *data_out_len = (uint16_t)len;
    }
    return 0;
}
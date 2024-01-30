#include "multi_channel_camera_util.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <memory.h>

#ifdef HDMI_CAPTURE_UTIL_DEBUG_ENABLE
#define debug_printf(format, arg...) printf(format, ##arg);
#else
#define debug_printf(format, arg...)                                           \
    {}
#endif

#define HDMI_FRAME_HEAD_CONTENT (0x5a5b)
#define HDMI_FRAME_TAIL_CONTENT (0x5b5a)
#define HDMI_FRAME_HEADER_PKT_SIZE (sizeof(struct frame_header_t))

#pragma pack(1)
struct frame_header_t {
    uint16_t head;      /*帧头*/
    uint8_t ch_num;     /*通道号*/
    uint32_t frame_num; /*帧号*/
    uint8_t type;       /*帧类型*/
    uint8_t release[2]; /*保留*/
    uint8_t sum;        /*校验和*/
    uint16_t tail;      /*帧尾部*/
};
#pragma pack()

static int device_memory_fd = -1;
const uint8_t decode_table[] = {0, 1, 2,  3,  4,  4,  5,  6,  7,
                                8, 9, 10, 10, 11, 12, 13, 14, 15};

int decode_frame_header(const void *frame, size_t frame_size,
                        struct frame_info_t *info) {

    int ret = 0;
    struct frame_header_t header;
    uint8_t *p_frame = (uint8_t *)frame;

    if ((p_frame == NULL) || (info == NULL)) {
        debug_printf(
            "[decode_frame_header:]Param is invalid, pointer is null .\n");
        return -1;
    }

    if (frame_size < HDMI_FRAME_HEADER_PKT_SIZE * 3) {
        debug_printf("[decode_frame_header:]Param is invalid, frame_size[%d] < "
                     "frame_header_t size [%d]\n",
                     frame_size, HDMI_FRAME_HEADER_PKT_SIZE);
        return -1;
    }
        memset(&header,0xFF,sizeof(header));
    {
        uint8_t *pheader = (uint8_t *)&header;
        uint8_t data_low = 0, data_high = 0;
        for (size_t i = 0; i < HDMI_FRAME_HEADER_PKT_SIZE; i++) {

            if ((p_frame[i * 6] < 0x82) ||
                (p_frame[i * 6] >= 0x82 + sizeof(decode_table)) ||
                (p_frame[i * 6 + 3] < 0x82) ||
                (p_frame[i * 6 + 3] >= 0x82 + sizeof(decode_table))) {
                /*数据编码错误，不能解码*/
                debug_printf("[decode_frame_header:]decode_frame_header "
                             "data invalide high[%02x] low[%02x].\n",
                             p_frame[i * 6], p_frame[i * 6 + 3]);
                return -1;
            }

            data_high = decode_table[(p_frame[i * 6] - 0x82)];
            data_low = decode_table[p_frame[i * 6 + 3] - 0x82];

            pheader[i] = ((data_high << 4) & 0xF0) | (data_low & 0x0F);
        }
#if 0
        printf("HDMI PKT:");
        for (size_t i = 0; i < sizeof(struct frame_header_t); i++) {
            printf("%02x ", pheader[i]);
        }
        printf("\n");
#endif
    }

    if ((header.head != HDMI_FRAME_HEAD_CONTENT) ||
        ((header.tail != HDMI_FRAME_TAIL_CONTENT))) {
        debug_printf("[decode_frame_header:]decode_frame_header head and "
                     "tail is failed.[head:0x%04x][tail:0x%04x]\n",
                     header.head, header.tail);
        return -3;
    }
    if (header.type >= VIDEO_FRAME_MAX) {
        debug_printf("[decode_frame_header:]decode_frame_header frame type is "
                     "failed.[0x%02x]\n",
                     header.type);
        return -4;
    }

    info->ch_num = header.ch_num;
    info->frame_num = header.frame_num;
    info->type = (enum VIDEO_FRAME_TYPE)header.type;
    return 0;
}

int init_device_memory(void) {
    const char *device_path = "/dev/fpgadrv0";
    device_memory_fd = open(device_path, O_RDWR);
    if (device_memory_fd < 0) {
        debug_printf("[hdmi_capture_util] open /dev/fpgadrv0 failed.\n");
        return -1;
    }
    return 0;
}

void deint_device_memory(void) {
    if (device_memory_fd >= 0) {
        close(device_memory_fd);
    }
}

void *malloc_device_memory(size_t size) {

    void *ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, device_memory_fd, 0);

    if ((void *)-1 == (void *)ptr) {
        debug_printf("[hdmi_capture_util] mmap64 error, %ld.\n", size);
        return NULL;
    }
    return ptr;
}

struct MemoryCacheArgs {
    void *pointer;
    size_t size;
};

int invalidate_device_memeory(void *addr, size_t size) {
#define IOCTL_FPGA_MAGIC (('F' + 'P' + 'G' + 'A') / 4)
#define IOCTL_MEMCACHE_INVAL _IOW(IOCTL_FPGA_MAGIC, 12, struct MemoryCacheArgs)
    struct MemoryCacheArgs args = {NULL, 0};
    args.pointer = addr;
    args.size = size;
    return ioctl(device_memory_fd, IOCTL_MEMCACHE_INVAL, &args);
}

int release_device_memory(void *addr, size_t size) {
    return munmap(addr, size);
}

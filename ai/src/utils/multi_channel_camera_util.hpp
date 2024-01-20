
#ifndef HDMI_CAPTURE_UTIL_H__H
#define HDMI_CAPTURE_UTIL_H__H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************************
 * 
 * 提供：大块连续内存管理接口
 * 使用场景：V4L2 中使用的缓存可以配置为 User 空间传递的内存
 * 但是系统本身无法分配到 1920*1080*3 的连续内存空间。  
 * 
***************************************************************************/
/*
 * Function:初始化
 *
 * return:  0 为成功，其他失败
 */
int init_device_memory(void);

void deint_device_memory(void);

/*
 * Function: 分配大块连续内存
 * return:  NULL： 失败
 */
void *malloc_device_memory(size_t size);

/*
 * Function: 释放内存
 * return:  NULL： 失败
 */
int release_device_memory(void *addr, size_t size);

/*
 * Function: 拷贝内存前调用：保证将cache中的数据写回内存
 * return:  NULL： 失败
 */
int invalidate_device_memeory(void *addr, size_t size);

/***************************************************************************
 * 
 * 视频帧中存储了 通道号与帧号，通过一下接口获取 这些信息。 
 * 
***************************************************************************/

enum VIDEO_FRAME_TYPE{
    VIDEO_FRAME_NORMAL = 0, /*正常的摄像头数据帧*/
    VIDEO_FRAME_NULL, /*由于网络等因素，没有获取到网络摄像头的数据，传递空白图片*/
    VIDEO_FRAME_MAX
};

struct frame_info_t {
    uint8_t ch_num;     /*通道号*/
    uint32_t frame_num; /*帧号*/
    enum VIDEO_FRAME_TYPE type;
};

/*
 * Function:    从视频帧中解码出 当前帧的信息
 *              并保存到info指向的内存中
 * 
 * frame :      视频帧其实地址
 * frame_size:  视频帧大小
 * header：     当前帧的信息并保存到info中
 *
 * return:  0 为成功，其他失败
 *          -1：参数错误
 *          -2：数据编码错误
 *          -3：数据头 尾校验错误
 *          -4：帧类型错误
 */
int decode_frame_header(const void *frame, size_t frame_size,
                        struct frame_info_t *info);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
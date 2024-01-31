/**
 * @brief 共享内存和信号量方法的声明，在已经编译好的libshared_mem.os中实现
 * @note  目的让control进程和ai进程共享数据：图像IMAGE_ID和结果RESULT_ID
 * @note  注意访问需互斥
*/
#ifndef _COMM_H__
#define _COMM_H__
 
#define IMAGE_ID 0x123456
#define RESULT_ID 0x123457
#define IMAGE_SHM 0x654322
#define RESULT_SHM 0x654321
#define SPEED_SHM 0x123123

#include <string>
#include <opencv2/opencv.hpp>
#include <sys/shm.h>
 
int CreateShm(int id, int size);
int DestroyShm(int shmid);
int GetShm(int id, int size);

//char* CreateShmPtr();

int GetSem(int id);
int InitSem(int semid, int val=1);
int DestroySem(int semid);
int semP(int semid, int num=0);
int semV(int semid, int num=0);

void matWrite(const std::string& filename, const cv::Mat& mat);
void matRead(const std::string& filename, cv::Mat& dst);

void matWrite(const int fd, const cv::Mat& mat);
void matRead(const int fd, cv::Mat& dst);

void matWrite(void* addr, const cv::Mat& mat);
void matRead(void* addr, cv::Mat& dst);

#endif

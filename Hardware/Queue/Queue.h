#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern  "C" {
#endif
  
#define MAX_SQUEUE_SIZE (128)

//定义数据类型
typedef  float  DataType;

typedef struct
{
    //队列的存储空间
    //队列的最大长度:120个，循环队列，不使用1个元素 
    DataType cdata[MAX_SQUEUE_SIZE + 1];
    //队头，队尾指针
    int nFront;
    int nRear;
    // 用户定义待使用buffer的长度：单片机中，不考虑使用new动态分配
    unsigned int nSize;
}squeue,*squlink;

typedef struct
{
  	DataType cps;
    unsigned long time;
    unsigned long updownTimes;//突变次数
    //unsigned char up15;//15%连续变化次数
    //unsigned char up25;//25%连续变化次数
    //unsigned char up30;//30%连续变化次数
    //unsigned char down15;//15%连续变化次数
    //unsigned char down25;//25%连续变化次数
    unsigned char down30;//30%连续变化次数
    unsigned char startFlashFlag;//波动标志
    unsigned char FlashTimes;//波动出现次数
    DataType OlduSvh2;
    DataType OlduSvh3;
    DataType OlduSvh4;
    DataType OlduSvh5;
    DataType OlduSvh6;

}S_BENDI;


//初始化，置空队，最大长度为20个元素（根据需要变化最大长度MAX_SQUEUE_SIZE）
void InitQueue(squlink Q, unsigned int nSize);

// FIFO 接口
//进队
int EnterQueue(squlink Q,DataType e);
//出队
int DeQueue(squlink Q, DataType*  e);
//判断队列是否为空 
int IsEmptyQueue(squlink Q);
//判断队列是否满
int IsFullQueue(squlink Q);
// 长度
int Lenqueue(squlink Q);

//栈接口
//退栈
int DeStack(squlink Q, DataType*  e);


// 从头读取第N个数据:1,2...n
DataType GetElement(squlink Q, int nOrder);
//设置元素值
int SetElement(squlink Q, int nOrder, DataType value);
// 前N个数总和（不出队）
DataType SumFront(squlink Q, int nCnt);
// 后N个数总和（不出队）
DataType SumRear(squlink Q, int nCnt);

//设置队列长度，保留最新几个数据
void SetQLength(squlink Q, int cnt);
#ifdef __cplusplus
}
#endif
#endif  
//#include <msp430x21x2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Queue.h"

#if 0
//置空队
void InitQueue(squlink Q, unsigned int nSize)
{
    if (!Q)
    {
        return;
    }
    
    Q->nFront = 0;
    Q->nRear = 0;
    //队列的最大长度:循环队列，不使用1个元素 
    if (nSize > MAX_SQUEUE_SIZE)
    {
         nSize = MAX_SQUEUE_SIZE ;
    }
    Q->nSize = nSize+1;
}

//判断队列是否为空 
int IsEmptyQueue(squlink Q)
{
    int bRet = 0;
     
    if (!Q)
    {
        return 1;
    }
    
    if(Q->nFront == Q->nRear)
    {
        bRet = 1;
    }
    else
    {
        bRet = 0;
    }
    
    return bRet;
}

int IsFullQueue(squlink Q)
{
    int bRet = 0;
     
    if (!Q)
    {
        return 1;
    }
    
    if(((Q->nRear + 1) % Q->nSize) == Q->nFront)
    {
        bRet = 1;
    }
    else
    {
        bRet = 0;
    }
    
    return bRet;
}

//进队
int EnterQueue(squlink Q,DataType e)
{
    if (!Q)
    {
        return 0;
    }
    
    //队满,则将读指针后移一位
    if(((Q->nRear + 1) % Q->nSize) == Q->nFront)
    {
        Q->nFront = (Q->nFront + 1) % Q->nSize;
    }
    
    Q->data[Q->nRear] = e;
    Q->nRear = (Q->nRear + 1) % Q->nSize;
   
    return 1;
}

//出队
int DeQueue(squlink Q, DataType*  e)
{
    if (!Q)
    {
        return 0;
    }
    
    int bRet = 0;
    //队空
    if(IsEmptyQueue(Q))
    {
        bRet = 0;
    }
    else
    {
        *e = Q->data[Q->nFront];
        Q->nFront = (Q->nFront+1) % Q->nSize;
        bRet = 1;
    }
    
    return bRet;
}

//队Q中当前元素个数
int Lenqueue(squlink Q)
{
    int i = 0;
    if (!Q)
    {
        return 0;
    }
    
    i = (Q->nRear - Q->nFront + Q->nSize) % Q->nSize;
    
    return(i);
}


//栈接口
//退栈
int DeStack(squlink Q, DataType*  e)
{
    if (!Q)
    {
        return 0;
    }
    
    int bRet = 0;
    //队空
    if(IsEmptyQueue(Q))
    {
        bRet = 0;
    }
    else
    {
        Q->nRear = (Q->nRear - 1 + Q->nSize) % Q->nSize;
        *e = Q->data[Q->nRear];
        bRet = 1;
    }
    
    return bRet;
}


// 前N个数总和（不出队）
DataType SumFront(squlink Q, int nCnt)
{
    if (!Q || Lenqueue(Q) < nCnt)
    {
        return 0;
    }
    DataType nSum = 0;
    for(int i = 0; i < nCnt; i++)
    {
        nSum += Q->data[((Q->nFront+i) % Q->nSize)];
    }
    
    return nSum;
}

// 后N个数总和（不出队）
DataType SumRear(squlink Q, int nCnt)
{
    if (!Q || Lenqueue(Q) < nCnt)
    {
        return 0;
    }
    DataType nSum = 0;
    for(int i = 0; i < nCnt; i++)
    {
        nSum += Q->data[((Q->nRear - 1 - i + Q->nSize) % Q->nSize)];
    }
    
    return nSum;
}


// 从头读取第N个数据:1,2...n
DataType GetElement(squlink Q, int nOrder)
{
    if (!Q || Lenqueue(Q) <= 0)
    {
        return 0;
    }
    
    DataType nRet = 0;
    nRet = Q->data[((Q->nFront + nOrder - 1) % Q->nSize)];
    return nRet;
}
//设置队列长度
void SetQLength(squlink Q, int cnt)
{
  	Q->nFront = (Q->nSize + Q->nRear - cnt)% Q->nSize;
}

// 设置
int SetElement(squlink Q, int nOrder, DataType value)
{
    if (!Q)
    {
        return 0;
    }
    
    int index = (Q->nFront + nOrder - 1) % Q->nSize;
    if (index == Q->nRear)
    {
      return 0;
    }
    Q->data[index] = value;
    return 1;
}
#endif


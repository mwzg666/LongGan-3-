#ifndef CALCCPS_H
#define CALCCPS_H

#ifdef __cplusplus
extern  "C" {
#endif

void EmtyBendi(void);

// 低通道
//初始化  
void InitCalcLow(void);
void EmtyCalcLowQ(void);
// 每一秒被调用一次
float CalcLow(float parama, float paramb, float paramc, float RealCPS, float OlduSvh, float *NewuSvh);
// 每100ms调用一次
void FilterLow(unsigned long nNewCnt);

//High
//初始化  
void InitCalcHigh(void);
void EmtyCalcHighQ(void);

// 每一秒被调用一次
float CalcHigh(float parama, float paramb, float paramc, float RealCPS, float OlduSvh, float *NewuSvh);
// 每100ms调用一次
void FilterHigh(unsigned long nNewCnt);

#ifdef __cplusplus
}
#endif
#endif  
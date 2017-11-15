#include <string>
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>


#ifndef _FUNCTIONS_H_
#define  _FUNCTIONS_H_

#include <algorithm>    // std::sort

static tBool str_comp(cString a, cString b){
	string a_str (a);			
	string b_str (b);	
	return (a_str.compare(b_str)== 0) ? tTrue : tFalse;
}

#define GETTIME()   ((  _clock  != NULL ) ? _clock->GetTime() : cSystem::GetTime())
#define PRINT(A) if(m_filterProperties.debugOutput) {LOG_INFO(A);}; 
#define PRINT1(A, B) if(m_filterProperties.debugOutput) {LOG_INFO(cString::Format(A, B));}; 
#define PRINT2(A, B, C) if(m_filterProperties.debugOutput) {LOG_INFO(cString::Format(A, B, C));}; 
#define PRINT3(A, B, C, D) if(m_filterProperties.debugOutput) {LOG_INFO(cString::Format(A, B, C, D));}; 
#define PRINT4(A, B, C, D, E) if(m_filterProperties.debugOutput) {LOG_INFO(cString::Format(A, B, C, D, E));}; 
#define PRINT5(A, B, C, D, E, F) if(m_filterProperties.debugOutput) {LOG_INFO(cString::Format(A, B, C, D, E, F));}; 
#define PRINT6(A, B, C, D, E, F, G) if(m_filterProperties.debugOutput) {LOG_INFO(cString::Format(A, B, C, D, E, F, G));}; 
#define PRINT7(A, B, C, D, E, F, G, H) if(m_filterProperties.debugOutput) {LOG_INFO(cString::Format(A, B, C, D, E, F, G, H));}; 
template<typename Type>
tFloat32 GetQueueMedian(std::deque<Type> q)
{
	std::sort(q.begin() , q.end());

	int n = q.size();
	switch(n) {
	case 0:
		return 0.0f;
	case 1:
		return q[0];
	default:
		if(n % 2 == 0) {
			return (q[n / 2] + q[n / 2 - 1]) / 2;
		} else {
			return q[n / 2];
		}
	}
}





#endif

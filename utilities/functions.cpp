#include "functions.h" 

functions::functions()
{    
}

functions::~functions()
{
}

tBool str_comp(cString a, cString b){
	string a_str (a);			
	string b_str (b);	
	return (a_str.compare(b_str)== 0)? tTrue : tFalse;
}


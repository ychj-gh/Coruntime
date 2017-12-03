
#pragma once



#ifdef USE_ALL_TYPE

struct IValue{
	int8_t type;
	union{
		unsigned long value_u;
		signed long value_i;
		double value_d;
	}v;
};

#else

#define IValue	double

#endif


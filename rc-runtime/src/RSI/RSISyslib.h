
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

#include <memory.h>

#include <tinyxml.h>

#include "IValue.h"
#include "rc_exception.h"

#define MAX_SPOU_NAME_SIZE  50

#define RSI_DEBUG_PRINT

extern bool RSIStopFlag; /* THIS IS VERY IMPORTANT, WHICH CONTROL THE WHOLE LIFECYCLE OF RSI */

#define RSI_STOP_CHECK() do{if(RSIStopFlag == true) return 0;}while(0);


#ifdef RSI_DEBUG_PRINT
extern std::unordered_map<int, std::string> rdataIndexMap;   // index --> var
#endif

class EntityBase;

typedef struct {
    char name[MAX_SPOU_NAME_SIZE];
    int param_count;
    int (*pfun)(std::vector<int>&, EntityBase*, std::vector<IValue>&);
} RSILibEntry; /* System-level POU(Library) descriptor */

typedef enum{
	RSI_SUM = 0,
	RSI_SUB,
	RSI_MULTI,
	RSI_DIV,

	RSI_ABS,
	RSI_NORM,
	RSI_EXP,
	RSI_LOG,
	RSI_CEIL,
	RSI_FLOOR,
	RSI_ROUND,
	RSI_POW,

	RSI_SIN,
	RSI_COS,
	RSI_TAN,
	RSI_ASIN,
	RSI_ACOS,
	RSI_ATAN,
	RSI_ATAN2,

	RSI_INC,
	RSI_DEC,

	RSI_PRINT,

	RSI_EQ,
	RSI_GT,
	RSI_GE,
	RSI_LT,
	RSI_LE,

	RSI_AND,
	RSI_OR,
	RSI_XOR,
	RSI_NOT,

	RSI_P,
	RSI_PD,
	RSI_I,
	RSI_D,
	RSI_PI,
	RSI_PID,

	RSI_AXISCORR,
	RSI_POSCORR,
	RSI_TRANSFRAME,
	RSI_POSACT,
	RSI_AXISACT,
	RSI_GEARTORQUE,
	RSI_MOTORCURRENT,

	RSI_COMMUNICATION,

}POUId;


inline int rsi_sum(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = 0;
	for(int i = 0; i < params.size(); i ++) {
		addrspace[0] += addrspace[params[i]];
	}
	return 0;
}

inline int rsi_sub(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] - addrspace[params[1]];
	return 0;
}

inline int rsi_multi(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] * addrspace[params[1]];
	return 0;
}
inline int rsi_div(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] / addrspace[params[1]];
	return 0;
}

inline int rsi_abs(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_norm(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_exp(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_log(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_ceil(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_floor(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_pow(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}

inline int rsi_sin(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_cos(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_tan(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_asin(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_acos(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_atan(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_atan2(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}

inline int rsi_inc(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[params[0]] += 1;
	return 0;
}
inline int rsi_dec(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[params[0]] -= 1;
	return 0;
}

inline int rsi_print(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
#ifdef RSI_DEBUG_PRINT
	std::cout << std::endl << "[DEBUG: " << rdataIndexMap[params[0]] << " --> " << addrspace[params[0]] << " ]" << std::endl;
#else
	std::cout << std::endl << "Macro RSI_DEBUG_PRINT not defined" << std::endl;
#endif
	return 0;
}


inline int rsi_eq(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] == addrspace[params[1]] ? 1 : 0;
	return 0;
}

inline int rsi_gt(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] > addrspace[params[1]] ? 1 : 0;
	return 0;
}

inline int rsi_ge(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] >= addrspace[params[1]] ? 1 : 0;
	return 0;
}

inline int rsi_lt(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] < addrspace[params[1]] ? 1 : 0;
	return 0;
}

inline int rsi_le(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]] <= addrspace[params[1]] ? 1 : 0;
	return 0;
}

inline int rsi_and(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	int cond1 = addrspace[params[0]];
	int cond2 = addrspace[params[1]];
	addrspace[0] = cond1 && cond2;

	return 0;
}

inline int rsi_or(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	int cond1 = addrspace[params[0]];
	int cond2 = addrspace[params[1]];
	addrspace[0] = cond1 || cond2;

	return 0;
}

inline int rsi_xor(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	int cond1 = addrspace[params[0]];
	int cond2 = addrspace[params[1]];
	if(cond1 == 0 && cond2 != 0 || cond1 != 0 && cond2 == 0){
		addrspace[0] = 1;
	} else {
		addrspace[0] = 0;
	}
	
	return 0;
}

inline int rsi_not(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	int cond1 = addrspace[params[0]];
	addrspace[0] = (cond1 == 0 ? 1 : 0);
	return 0;
}

inline int rsi_p(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_pd(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_i(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_d(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_pi(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}

int rsi_pid(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) ;

inline int rsi_timer(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_limit(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}

inline int rsi_min(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]];
	for(int i = 0; i < params.size(); i ++) {
		addrspace[0] = addrspace[params[i]] < addrspace[0] ? addrspace[params[i]] : addrspace[0];
	}
	return 0;
}

inline int rsi_max(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	addrspace[0] = addrspace[params[0]];
	for(int i = 0; i < params.size(); i ++) {
		addrspace[0] = addrspace[params[i]] > addrspace[0] ? addrspace[params[i]] : addrspace[0];
	}
	return 0;
}

inline int rsi_delay(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_monitor(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_stop(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	if(addrspace[params[0]] == 1) {
		RSIStopFlag = true;
	}
}


int rsi_axiscorr(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) ;

int rsi_poscorr(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) ;

inline int rsi_transfame(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}

int rsi_posact(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) ;
int rsi_axisact(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) ;


inline int rsi_geartorque(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}
inline int rsi_motorcurrent(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {}

int rsi_comm_interface(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace);



#define VARIABLE_LEN 0

#define RSI_LIB_SIZE  51

/* ORDER SENSITIVE */
static const RSILibEntry libEntry[RSI_LIB_SIZE] = {
	{"SUM", VARIABLE_LEN, rsi_sum},
	{"SUB", 2, rsi_sub},
	{"MULTI", 2, rsi_multi},
	{"DIV", 2, rsi_div},

	{"ABS", 1, rsi_abs},
	{"NORM", VARIABLE_LEN, rsi_norm},
	{"EXP", 2, rsi_exp},
	{"LOG", 2, rsi_log},
	{"CEIL", 1, rsi_ceil},
	{"FLOOR", 1, rsi_floor},
	{"POW", 2, rsi_pow},

	{"SIN", 1, rsi_sin},
	{"COS", 1, rsi_cos},
	{"TAN", 1, rsi_tan},
	{"ASIN", 1, rsi_asin},
	{"ACOS", 1, rsi_acos},
	{"ATAN", 1, rsi_atan},
	{"ATAN2", 1, rsi_atan2},

	{"INC", 1, rsi_inc},
	{"DEC", 1, rsi_dec},

	{"PRINT", 1, rsi_print},

	{"EQ", 2, rsi_eq},
	{"GT", 2, rsi_gt},
	{"GE", 2, rsi_ge},
	{"LT", 2, rsi_lt},
	{"LE", 2, rsi_le},

	{"AND", 2, rsi_and},
	{"OR", 2, rsi_or},
	{"XOR", 2, rsi_xor},
	{"NOT", 1, rsi_not},

	{"P", 1, rsi_p},
	{"PD", 1, rsi_pd},
	{"I", 1, rsi_i},
	{"D", 1, rsi_d},
	{"PI", 1, rsi_pi},
	{"PID", 1, rsi_pid},

	{"TIMER", 1, rsi_timer},
	{"LIMIT", 3, rsi_limit},
	{"MIN", VARIABLE_LEN, rsi_min},
	{"MAX", VARIABLE_LEN, rsi_max},
	{"DELAY", 1, rsi_delay},
	{"MONITOR", VARIABLE_LEN, rsi_monitor},
	{"STOP", 1, rsi_stop},

	{"AXISCORR", VARIABLE_LEN, rsi_axiscorr},
	{"POSCORR", VARIABLE_LEN, rsi_poscorr},
	{"TRANSFRAME", VARIABLE_LEN, rsi_transfame},
	{"POSACT", VARIABLE_LEN, rsi_posact},
	{"AXISACT", VARIABLE_LEN, rsi_axisact},
	{"GEARTORQUE", VARIABLE_LEN, rsi_geartorque},
	{"MOTORCURRENT", VARIABLE_LEN, rsi_motorcurrent},

	{"COMMUNICATION", VARIABLE_LEN, rsi_comm_interface},
};


/* ************** the Entity type of function block ***************** */
class EntityBase {
public:
	EntityBase(std::string f) : funcName(f) {}

	virtual int setConfig(std::string key, std::string value) = 0;

	virtual int printInfo() { std::cout << "[> EntityBase <]" << std::endl; }

public:
	std::string funcName;
};

class EntityPID : public EntityBase{
public:
	EntityPID() : EntityBase("PID"), kp(0), ki(0), kd(0), Ts(10), prev(0) {}

	virtual int setConfig(std::string key, std::string value) override {
		if(key == "Kp")  {
			kp = std::stod(value);
		} else if(key == "Ki") {
			ki = std::stod(value);
		} else if(key == "Kd") {
			kd = std::stod(value);
		} else if(key == "Ts") {
			Ts = std::stod(value);
		} else {
			return -1;
		}
		return 0;
	}

	virtual int printInfo() override {
		std::cout << std::endl << "Type: " << funcName << " --> ";
		std::cout << "Kp=" << kp << " Ki=" << ki << " Kd=" << kd << " Ts=" << Ts << std::endl;
	}

public:
	IValue kp;
	IValue ki;
	IValue kd;

	IValue Ts;

	IValue prev;
};


class EntityDELAY : public EntityBase {
public:
	EntityDELAY() : EntityBase("DELAY"), T(0) {}

	virtual int setConfig(std::string key, std::string value) override {
		if(key == "T")  {
			T = std::stod(value);
		} else {
			return -1;
		}
		return 0;
	}

	virtual int printInfo() override {
		std::cout << std::endl << "Type: " << funcName << " --> ";
		std::cout << "T=" << T << std::endl;
	}

public:
	IValue T;
};


class EntityComm : public EntityBase {
public:
	struct SendDataElem {  		// just for EntityComm
		std::string tag;
		std::vector<std::pair<std::string, int>> dataMap;
		std::pair<std::string, int> tagText;
	};

public:
	EntityComm() : EntityBase("COMMUNICATION"), initflag(false) {
		memset(sendBuffer, 0, 4096);
		memset(recvBuffer, 0, 4096);
	}

	virtual int setConfig(std::string key, std::string value) override {
		if(key == "COM_TYPE")  {
			comm_type = value;
		} else if(key == "IP"){
			ip = value;
		} else if(key == "PORT"){
			port = value;
		} else if(key == "ROOTNAME"){
			rootNode = value;
		}else {
			return -1;
		}
		return 0;
	}

	int setSendDataMap(std::string &tagName, int varIndex) {
		int pos = 0;
		std::string tag;
		std::string key;

		if((pos = tagName.find(".")) != std::string::npos) {
			tag = tagName.substr(0, pos);
			key = tagName.substr(pos+1);
		} else {
			tag = tagName;
			key = tagName;
		}

		int index = 0;
		for(; index < sendDataMap.size(); index ++) {
			if(sendDataMap[index].tag == tag) {
				break;
			}
		}
		if(index == sendDataMap.size()) {
			SendDataElem tmp;
			tmp.tag = tag;
			if(tag != key) {
				tmp.dataMap.push_back(std::pair<std::string, int>(key, varIndex));
			} else {
				tmp.tagText = std::pair<std::string, int>(key, varIndex);
			}
			sendDataMap.push_back(tmp);
		} else {
			if(tag != key) {
				sendDataMap[index].dataMap.push_back(std::pair<std::string, int>(key, varIndex));
			} else{
				sendDataMap[index].tagText = std::pair<std::string, int>(key, varIndex);
			}
		}

		return 0;
	}

	int setRecvDataMap(std::string &tagName, int varIndex) {
		int pos = 0;
		std::string tag;
		std::string key;

		if((pos = tagName.find(".")) != std::string::npos) {
			tag = tagName.substr(0, pos);
			key = tagName.substr(pos+1);
		} else {
			tag = tagName;
			key = tagName;
		}

		if(recvDataMap.find(tag) != recvDataMap.end()) {
			recvDataMap[tag].insert(std::pair<std::string, int>(key, varIndex));
		} else {
			std::unordered_map<std::string, int> map;
			map.insert(std::pair<std::string, int>(key, varIndex));
			recvDataMap.insert(std::pair<std::string, std::unordered_map<std::string, int>>(tag, map));
		}

	}

	virtual int printInfo() override {
		std::cout << std::endl << "Type: " << funcName << " --> ";
		std::cout << "comm_type=" << comm_type << " ip=" << ip << " port=" << port << " rootNode=" << rootNode << std::endl;
		std::cout << "[> --- SENDDATAMAP --- <]" << std::endl;
		for(auto &e : sendDataMap) {
			std::cout << e.tag << std::endl;
			for(auto &elem : e.dataMap) {
				std::cout << elem.first << "<-->" << elem.second << " ";
			}
			std::cout << std::endl;
			if(e.tagText.first != "") {
				std::cout << e.tagText.first << "<-->" << e.tagText.second << std::endl;
			}
		}
		std::cout << "[> --- RECVDATAMAP --- <]" << std::endl;
		for(auto &e : recvDataMap) {
			std::cout << e.first << std::endl;
			for(auto &elem : e.second) {
				std::cout << elem.first << "<-->" << elem.second << " ";
			}
			std::cout << std::endl;
		}
	}

public:
	int xmlGenerate(std::vector<IValue>& addrspace) {
		TiXmlDocument doc;
		TiXmlElement *rootEle = new TiXmlElement(rootNode);
		for(auto &e : sendDataMap) {
			TiXmlElement *tagEle = new TiXmlElement(e.tag);
			for(auto &elem : e.dataMap) {
				tagEle->SetAttribute(elem.first, std::to_string(addrspace[elem.second]));
			}
			if(e.tagText.first != "") {
				tagEle->LinkEndChild(new TiXmlText(std::to_string(addrspace[e.tagText.second])));
			}
			rootEle->LinkEndChild(tagEle);
		}
		doc.LinkEndChild(rootEle);

		TiXmlPrinter printer;
		// printer.SetIndent( "\t" );

		doc.Accept( &printer );
		sprintf( sendBuffer, "%s", printer.CStr() );

		return 0;
	}


	int xmlParse(std::vector<IValue>& addrspace) {
		TiXmlDocument doc;
		doc.Parse(recvBuffer);
		
		TiXmlElement* rootEle = doc.RootElement();  

		if(rootEle != NULL && rootEle->Value() == rootNode) {
			TiXmlElement* dataEle = rootEle->FirstChildElement();

			for(; dataEle != NULL; dataEle = dataEle->NextSiblingElement()) {
				std::string tag = dataEle->Value();
				std::unordered_map<std::string, int> &mp = recvDataMap[tag];

				TiXmlAttribute* attr = dataEle->FirstAttribute();
				for(; attr != NULL; attr = attr->Next()) {
					addrspace[mp[attr->Name()]] = std::stod(attr->Value());
				}
				if(mp.find(tag) != mp.end()) {
					addrspace[mp[tag]] = std::stod(dataEle->GetText());
				} 
			}
			return 0;
		} else {
			return -1;
		}
	}

public:
	std::string comm_type;			// communication interface type (udp\tcp\...)
	std::string ip;					// ip address
	std::string port;				// port number

	std::string rootNode;			// the root node name of xml during transferation

	std::unordered_map<std::string, std::unordered_map<std::string, int>>  recvDataMap; // indicate <tag, <property, index>>

	std::vector<SendDataElem> sendDataMap;		// send data rule
public:
	bool initflag;		

	int sockfd;							// udp socket fd
	struct sockaddr_in addr;			// target socket address		

	char sendBuffer[4096];			
	char recvBuffer[4096];
};


class EntityPOSCORR : public EntityBase {
public:
	EntityPOSCORR() : EntityBase("POSCORR"), LowerLimX(0), LowerLimY(0), LowerLimZ(0),
						UpperLimX(0), UpperLimY(0), UpperLimZ(0), 
						MaxRotAngle(0), RefCorrSys(0), CorrType(0),
						overDeltaX(0), overDeltaY(0), overDeltaZ(0),
						overDeltaA(0), overDeltaB(0), overDeltaC(0),
						OverTransLim(0), OverRotAngle(0) {}

	virtual int setConfig(std::string key, std::string value) override {
		if(key == "LowerLimX")  {
			LowerLimX = std::stod(value);
		} else if(key == "LowerLimY") {
			LowerLimY = std::stod(value);
		} else if(key == "LowerLimZ") {
			LowerLimZ = std::stod(value);
		} else if(key == "UpperLimX") {
			UpperLimX = std::stod(value); 
		} else if(key == "UpperLimY") {
			UpperLimY = std::stod(value);
		} else if(key == "UpperLimZ") {
			UpperLimZ = std::stod(value);
		} else if(key == "MaxRotAngle") {
			MaxRotAngle = std::stod(value);
 		} else if(key == "RefCorrSys") {
 			RefCorrSys = std::stoi(value);
 		} else if(key == "CorrType") {
 			CorrType = std::stoi(value);
 		} else if(key == "OverTransLim") {
 			OverTransLim = std::stod(value);
 		} else if(key == "OverRotAngle") {
 			OverRotAngle = std::stod(value);
 		} else {
 			return -1;
 		}
		return 0;
	}

	virtual int printInfo() override {
		std::cout << std::endl << "Type: " << funcName << " --> " << " CorrType=" << CorrType << std::endl;;
		std::cout << " LowerLimX=" << LowerLimX << " LowerLimY=" << LowerLimY << " LowerLimZ=" << LowerLimZ
				  << " UpperLimX=" << UpperLimX << " UpperLimY=" << UpperLimY << " UpperLimZ=" << UpperLimZ
				  << " MaxRotAngle=" << MaxRotAngle << " RefCorrSys=" << RefCorrSys 
				  << std::endl << " OverTransLim=" << OverTransLim << " OverRotAngle=" << OverRotAngle
				  << std::endl;
				  
	}

public:
	int    CorrType;	// 0: relative 1: absolute
	int    RefCorrSys;  // 0: axis 1: base 2: tool

	IValue LowerLimX;
	IValue LowerLimY;
	IValue LowerLimZ;
	IValue UpperLimX;
	IValue UpperLimY;
	IValue UpperLimZ;
	IValue MaxRotAngle;

	IValue OverTransLim;
	IValue OverRotAngle;

public:
	IValue overDeltaX;
	IValue overDeltaY;
	IValue overDeltaZ;
	IValue overDeltaA;
	IValue overDeltaB;
	IValue overDeltaC;	  
};


class EntityAXISCORR : public EntityBase {
public:
	EntityAXISCORR() : EntityBase("AXISCORR"), CorrType(0), LowerLimA1(0), LowerLimA2(0), LowerLimA3(0),
						LowerLimA4(0), LowerLimA5(0), LowerLimA6(0), UpperLimA1(0), UpperLimA2(0),
						UpperLimA3(0), UpperLimA4(0), UpperLimA5(0), UpperLimA6(0),
						overDeltaA1(0), overDeltaA2(0), overDeltaA3(0), overDeltaA4(0),
						overDeltaA5(0), overDeltaA6(0) {}

	virtual int setConfig(std::string key, std::string value) override {
		if(key == "LowerLimA1")  {
			LowerLimA1 = std::stod(value);
		} else if(key == "LowerLimA2") {
			LowerLimA2 = std::stod(value);
		} else if(key == "LowerLimA3") {
			LowerLimA3 = std::stod(value);
		} else if(key == "LowerLimA4") {
			LowerLimA4 = std::stod(value);
		} else if(key == "LowerLimA5") {
			LowerLimA5 = std::stod(value);
		} else if(key == "LowerLimA6") {
			LowerLimA6 = std::stod(value);
		} else if(key == "UpperLimA1") {
			UpperLimA1 = std::stod(value);
		} else if(key == "UpperLimA2") {
			UpperLimA2 = std::stod(value);
		} else if(key == "UpperLimA3") {
			UpperLimA3 = std::stod(value);
		} else if(key == "UpperLimA4") {
			UpperLimA4 = std::stod(value); 
		} else if(key == "UpperLimA5") {
			UpperLimA5 = std::stod(value);
		} else if(key == "UpperLimA6") {
			UpperLimA6 = std::stod(value);
		} else if(key == "CorrType") {
 			CorrType = std::stoi(value);
 		} else if(key == "OverLimA1") {
 			OverLimA1 = std::stod(value);
 		} else if(key == "OverLimA2") {
 			OverLimA2 = std::stod(value);
 		} else if(key == "OverLimA3") {
 			OverLimA3 = std::stod(value);
 		} else if(key == "OverLimA4") {
 			OverLimA4 = std::stod(value);
 		} else if(key == "OverLimA5") {
 			OverLimA5 = std::stod(value);
 		} else if(key == "OverLimA6") {
 			OverLimA6 = std::stod(value);
 		} else {
 			return -1;
 		}
		return 0;
	}

	virtual int printInfo() override {
		std::cout << std::endl << "Type: " << funcName << " --> " << " CorrType=" << CorrType << std::endl;;
		std::cout << " LowerLimA1=" << LowerLimA1 << " LowerLimA2=" << LowerLimA2 << " LowerLimA3=" << LowerLimA3
				  << " LowerLimA4=" << LowerLimA4 << " LowerLimA5=" << LowerLimA5 << " LowerLimA6=" << LowerLimA6
				  << std::endl 
				  << " UpperLimA1=" << UpperLimA1 << " UpperLimA2=" << UpperLimA2 << " UpperLimA3=" << UpperLimA3
				  << " UpperLimA4=" << UpperLimA4 << " UpperLimA5=" << UpperLimA5 << " UpperLimA6=" << UpperLimA6
				  << std::endl
				  << " OverLimA1=" << OverLimA1 << " OverLimA2=" << OverLimA2 << " OverLimA3=" << OverLimA3
				  << " OverLimA4=" << OverLimA4 << " OverLimA5=" << OverLimA5 << " OverLimA6=" << OverLimA6
				  << std::endl;
				  
	}

public:
	int CorrType; 	// 0: relative  1: absolute

	IValue LowerLimA1;
	IValue LowerLimA2;
	IValue LowerLimA3;
	IValue LowerLimA4;
	IValue LowerLimA5;
	IValue LowerLimA6;

	IValue UpperLimA1;
	IValue UpperLimA2;
	IValue UpperLimA3;
	IValue UpperLimA4;
	IValue UpperLimA5;
	IValue UpperLimA6;

	IValue OverLimA1;
	IValue OverLimA2;
	IValue OverLimA3;
	IValue OverLimA4;
	IValue OverLimA5;
	IValue OverLimA6;

public:
	IValue overDeltaA1;
	IValue overDeltaA2;
	IValue overDeltaA3;
	IValue overDeltaA4;
	IValue overDeltaA5;
	IValue overDeltaA6;
};



class EntityFactory {
public:
	static EntityBase* getEntity(std::string name) {
		if(name == "PID") {
			return new EntityPID();
		} else if(name == "DELAY") {
			return new EntityDELAY();
		} else if(name == "COMMUNICATION"){
			return new EntityComm();
		} else if(name == "POSCORR") {
			return new EntityPOSCORR();
		} else if(name == "AXISCORR") {
			return new EntityAXISCORR();
		} else {
			return NULL;
		}
	}
};






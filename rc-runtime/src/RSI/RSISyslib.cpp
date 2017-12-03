
#include "RSISyslib.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "opmanager.hh"

bool RSIStopFlag = true; /* THIS IS VERY IMPORTANT, WHICH CONTROL THE WHOLE LIFECYCLE OF RSI */

// #define RSI_DEBUG

#ifdef RSI_DEBUG_PRINT
std::unordered_map<int, std::string> rdataIndexMap;   // index --> var
#endif

inline int rsi_pid(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	EntityPID *entity = dynamic_cast<EntityPID*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
		rc_fb_lackofconfig_exception("PID");
	}
	return 0;
}

inline int rsi_comm_interface( 	std::vector<int>& params, 
								EntityBase* config, 
								std::vector<IValue>& addrspace) 
{
	EntityComm *entity = dynamic_cast<EntityComm*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
		throw rc_fb_lackofconfig_exception("COMMUNICATION");
	}

    // struct sockaddr_in addr;
    int &sockfd = entity->sockfd;

    if(entity->initflag != true) {
    	sockfd = socket(AF_INET, SOCK_DGRAM, 0); 

    	struct timeval timeout = {0, 12};
    	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));

	    bzero(&entity->addr, sizeof(entity->addr));
	    entity->addr.sin_family = AF_INET;
	    entity->addr.sin_port = htons(atoi(entity->port.c_str()));
	    entity->addr.sin_addr.s_addr = inet_addr(entity->ip.c_str());

	    entity->initflag = true;
    }

    entity->xmlGenerate(addrspace);
    std::cout << entity->sendBuffer << std::endl;
    int sn = sendto(sockfd, entity->sendBuffer, strlen(entity->sendBuffer), 0, (struct sockaddr *)&entity->addr, sizeof(entity->addr));
    std::cout << "send ==> " << sn << " bytes" << std::endl;

/*    std::string str = "<?xml version=\"1.0\" encoding=\"GBK\"?>\
    <Root>\
          <RKorr X=\"1.2\" Y=\"2.3\" Z=\"3.4\" A=\"4.5\" B=\"5.6\" C=\"6.7\"/>\
          <FREE>100</FREE>\
          <STOP stopFlag=\"0\" />\
	<Root>";  
	sprintf(entity->recvBuffer, "%s", str.c_str());
	std::cout << entity->recvBuffer << std::endl;*/

    std::cout << "waiting for data ..." << std::endl;
    int rn = recvfrom(sockfd, entity->recvBuffer, 4096, 0, NULL, NULL);
    if(rn == -1 && errno == EAGAIN) {
    	std::cout << "receive timeout " << std::endl;
    	throw rc_rsicomm_outoftime_exception("rsi_comm_interface");
    } else {
    	std::cout << "recv <== " << rn << " bytes" << std::endl;
    	entity->xmlParse(addrspace);
    }
    

    // close(sockfd);
    
	return 0;
}



inline int rsi_poscorr(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	EntityPOSCORR *entity = dynamic_cast<EntityPOSCORR*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
		if(params.size() == 6) {
			if(addrspace[params[0]] < entity->LowerLimX || addrspace[params[0]] > entity->UpperLimX
				|| addrspace[params[1]] < entity->LowerLimY || addrspace[params[1]] > entity->UpperLimY
				|| addrspace[params[2]] < entity->LowerLimZ || addrspace[params[2]] > entity->UpperLimZ
				) {
				throw rc_exception();
			}
		} else {
			throw rc_wrongparam_exception();
		}

	} else {
		std::cout << "this fb does not have config entity" << std::endl;
		throw rc_fb_lackofconfig_exception("POSCORR");
	}

	/* define a temp robot inst */
	ROBOT_INST temp_inst;			
	/* step 1: setting the robot inst type */
	temp_inst.ri_type = CART_ADJUST;
	/* step 2: specify the first point(the current position) of CART_ADJUST inst  */	
	AxisPos_Deg p1(6);
	rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);
	for(int i = 0; i < 6; i ++) {
		p1[i] = -rc_shm->actual_info.axis_info[i].actual_pos;
	}
	p1[1] = rc_shm->actual_info.axis_info[1].actual_pos;
	rt_mutex_release(&rc_mutex_desc);

	temp_inst.args[0].apv = p1;
	/* step 3: specify the second point(the target position) of CART_ADJUST inst  */	
	XyzPose p2;
	
	for(int i = 0; i < 6; i ++) {
		p2[i] = addrspace[params[i]];
	}
	temp_inst.args[1].cpv = p2;
	/* step 4: setting frame */
	temp_inst.args[0].jjp.refsys = 2;
	/* step 5: insert inst into inst-buffer */
	inst_buffer_write(temp_inst);	


	return 0;
}


inline int rsi_axiscorr(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	EntityAXISCORR *entity = dynamic_cast<EntityAXISCORR*>(config);
	if(entity != NULL) {
#ifdef RSI_DEBUG
		entity->printInfo();
#endif
	} else {
		std::cout << "this fb does not have config entity" << std::endl;
		rc_fb_lackofconfig_exception("AXISCORR");
	}	

	/* define a temp robot inst */
	ROBOT_INST temp_inst;			
	/* step 1: setting the robot inst type */
	temp_inst.ri_type = AXIS_ADJUST;
	/* step 2: specify the first point(the current position) of CART_ADJUST inst  */	
	AxisPos_Deg p1(6);
	rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);
	for(int i = 0; i < 6; i ++) {
		p1[i] = -rc_shm->actual_info.axis_info[i].actual_pos;
	}
	p1[1] = rc_shm->actual_info.axis_info[1].actual_pos;
	rt_mutex_release(&rc_mutex_desc);

	temp_inst.args[0].apv = p1;
	std::cout << " input apv " << p1.transpose() << std::endl;
	/* step 3: specify the second point(the target position) of CART_ADJUST inst  */	
	AxisPos_Deg p2(6);
	
	for(int i = 0; i < 6; i ++) {
		p2[i] = addrspace[params[i]];
	}
	temp_inst.args[1].apv = p2;
	/* step 4: setting frame */
	temp_inst.args[0].jjp.refsys = 0;
	/* step 5: insert inst into inst-buffer */
	inst_buffer_write(temp_inst);	
	return 0;
}


inline int rsi_posact(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	int size = params.size();
	std::vector<IValue> pos(6, 0);
	cur_cartpos_get(pos);
	for(int i = 0; i < size; i ++) {
		addrspace[params[i]] = pos[i];
	}
	return 0;
}

inline int rsi_axisact(std::vector<int>& params, EntityBase* config, std::vector<IValue>& addrspace) {
	int size = params.size();
	std::vector<IValue> pos(6, 0);
	cur_axispos_get(pos);
	for(int i = 0; i < size; i ++) {
		addrspace[params[i]] = pos[i];
	}
	return 0;
}

/*
 * modbusConfig.cpp
 *
 *  Created on: 4 Nov 2022
 *      Author: Manish
 */

#include "chip.h"
#include "Fmutex.h"
#include "modbusConfig.h"

modbusConfig::modbusConfig():
	// TODO Auto-generated constructor stub
	node_hmp(HMP60_ID),
	node_gmp(GMP252_ID),
	temp_(&node_hmp, read_temp_r_ADD, true),
	rh_(&node_hmp, read_rh_r_ADD, true),
	co2_(&node_gmp, read_co2_r_ADD, true)
	{
		node_hmp.begin(TRANSMISSION_RATE);
		node_gmp.begin(TRANSMISSION_RATE);
	}


int modbusConfig::get_temp(){
	mutex.lock();
	return temp_.read()/10;
	mutex.unlock();
}

int modbusConfig::get_rh(){
	mutex.lock();
    return rh_.read()/10;
    mutex.unlock();
}

int modbusConfig::get_co2(){
	mutex.lock();
	return (co2_.read()/10)*100;
	mutex.unlock();
}

modbusConfig::~modbusConfig() {
	// TODO Auto-generated destructor stub
}

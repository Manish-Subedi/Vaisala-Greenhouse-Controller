/*
 * modbusConfig.cpp
 *
 *  Created on: 4 Nov 2022
 *      Author: Manish
 */

#include "chip.h"
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
	return temp_.read()/10;
}

int modbusConfig::get_rh(){
    return rh_.read()/10;
}

int modbusConfig::get_co2(){
	return (co2_.read()/10);
}

modbusConfig::~modbusConfig() {
	// TODO Auto-generated destructor stub
}

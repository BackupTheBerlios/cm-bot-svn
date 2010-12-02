/*
 * movement.c
 *
 *  Created on: 23.11.2010
 *      Author: christof
 */

#include "include/movement.h"
#include "include/xmega.h"
#include "include/utils.h"
#include "include/communication.h"
#include "include/dynamixel.h"
#include "include/kinematics.h"

#define MV_DST_X	168.5

void MV_action(DT_leg* const leg_r, DT_leg* const leg_l) {
	DNX_sendAction(leg_r->hip.id);
	DNX_sendAction(leg_r->knee.id);
	DNX_sendAction(leg_r->foot.id);

	DNX_sendAction(leg_l->hip.id);
	DNX_sendAction(leg_l->knee.id);
	DNX_sendAction(leg_l->foot.id);
}

void MV_slave(DT_byte cpuID, DT_leg* const leg_r, DT_leg* const leg_l) {
	DT_size len;
	DT_byte result[DT_RESULT_BUFFER_SIZE];

	while (1) {
		XM_LED_OFF
		len = COM_receive(&XM_com_data3, result);

		if (len == 0)
			continue;
		DEBUG (("sl_pck_rec",sizeof("sl_pck_rec")))
		if (result[2] != cpuID && result[2] != COM_BRDCAST_ID)
			continue;
		DEBUG (("sl_pck_acc",sizeof("sl_pck_acc")))

		XM_LED_ON
		switch (result[4]) {
		case COM_STATUS:
			DEBUG(("sl_rec_sts",sizeof("sl_rec_sts")))
			MV_slaveStatus(result, len);
			break;
		case COM_ACTION:
			DEBUG(("sl_rec_act",sizeof("sl_rec_act")))
			MV_action(leg_r, leg_l);
			break;
		case COM_POINT:
			DEBUG(("sl_rec_pnt",sizeof("sl_rec_pnt")))
			MV_slavePoint(leg_r, leg_l, result, len);
			break;
		default:
			DEBUG (("sl_err",sizeof("sl_err")))
			break;
		}
	}
}

void MV_masterCheckAlive() {
	// CHECK CPUs
	DT_bool isAlive = false;
	XM_LED_OFF
	do {
		if (COM_isAlive(COM_SLAVE1B) && COM_isAlive(COM_SLAVE3F)) {
			isAlive = true;
		} else
			UTL_wait(5);
	} while (isAlive == false);
	DEBUG (("ma_alv",sizeof("ma_alv")))
	XM_LED_ON
}

void MV_slaveStatus(const DT_byte* const result, const DT_size len) {
	switch (result[5]) {
	case COM_IS_ALIVE:
		COM_sendACK(COM_MASTER);
		DEBUG(("sl_snd_ack",sizeof("sl_snd_ack")))
		break;
	default:
		break;
	}
}

void MV_slavePoint(DT_leg* const leg_r, DT_leg* const leg_l,
		const DT_byte* const result, DT_size len) {
	DT_point p = COM_getPointFromPacket(result);
	DT_bool isGlobal = COM_isGlobal(result);
	DT_bool ret;

	if (COM_isLeftLeg(result)) {
		ret = MV_point(leg_l, &p, isGlobal);
	}
	if (COM_isRightLeg(result)) {
		ret = MV_point(leg_r, &p, isGlobal);
	}
	if(ret == true){
		COM_sendACK(COM_MASTER);
	}else{
		COM_sendNAK(COM_MASTER, COM_ERR_POINT_OUT_OF_BOUNDS);
	}
}

DT_bool MV_point(DT_leg* const leg, const DT_point* const point, DT_bool isGlobal) {
	DT_bool ret;
	if (isGlobal == true) {
		DT_point pLocal = KIN_calcLocalPoint(point, &leg->trans);
		ret = KIN_calcServos(&pLocal, leg);
	} else
		ret = KIN_calcServos(point, leg);

	if (ret == true) {
		leg->hip.set_value = UTL_getDegree(leg->hip.set_value);
		leg->knee.set_value = UTL_getDegree(leg->knee.set_value);
		leg->foot.set_value = UTL_getDegree(leg->foot.set_value);

		DNX_setAngle(leg->hip.id, leg->hip.set_value, true);
		DNX_setAngle(leg->knee.id, leg->knee.set_value, true);
		DNX_setAngle(leg->foot.id, leg->foot.set_value, true);
		return true;
	} else {
		return false;
	}
}

void MV_doInitPosition(DT_leg* const leg_r, DT_leg* const leg_l) {
	XM_LED_OFF
	DT_byte config;

	// Punkt fuer Null-Stellung mittleres rechtes Bein als Bezug
	DT_point pTmp;
	pTmp.x = 190 + MV_DST_X;
	pTmp.y = 0;
	pTmp.z = -14;
	DT_point pNull = pTmp;

	// Master rechts
	// pTmp = pNull;
	// pTmp.x = pNull.x;
	// pTmp.y = pNull.y + 0;
	MV_point(leg_r, &pTmp, true);

	// Master links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	// pTmp.y = pNull.y + 0;
	MV_point(leg_l, &pTmp, true);

	// Slave3 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y + MV_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE3F, &pTmp, config);

	// Slave3 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y + MV_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE3F, &pTmp, config);

	// Slave1 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y - MV_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	// Slave1 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y - MV_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	COM_sendAction(COM_BRDCAST_ID);
	MV_action(leg_r, leg_l);

	UTL_wait(20);

	// Roboter auf Beine stellen
	pTmp.x = 110.1041 + MV_DST_X;
	pTmp.y = 0;
	pTmp.z = -129.1041;
	pNull = pTmp;

	// Master rechts
	// pTmp = pNull;
	// pTmp.x = pNull.x;
	// pTmp.y = pNull.y + 0;
	MV_point(leg_r, &pTmp, true);

	// Master links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	// pTmp.y = pNull.y + 0;
	MV_point(leg_l, &pTmp, true);

	// Slave3 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y + MV_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE3F, &pTmp, config);

	// Slave3 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y + MV_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE3F, &pTmp, config);

	// Slave1 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y - MV_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	// Slave1 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y - MV_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	COM_sendAction(COM_BRDCAST_ID);
	MV_action(leg_r, leg_l);

	DEBUG(("ma_int_pos_ok",sizeof("ma_int_pos_ok")))
	XM_LED_ON
	UTL_wait(40);
}

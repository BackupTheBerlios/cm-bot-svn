/**
 * \file movement4Points.c
 *
 *  \brief	Algorithmus fuer das Vorwaertslaufen ueber 4 Punkte.
 */

#define TEST_ON TEST
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/communication.h"

// TODO Versatz eintragen
#define SL_DST_Y	208.5
#define DST_X	168.5

DT_leg leg_r, leg_l;
DT_byte cpuID;
DT_transformation trans_r, trans_l;
DT_byte result[DT_RESULT_BUFFER_SIZE];
DT_size len;

void master();
void ma_checkAlive();
void ma_setInitialPosition();
void ma_setPoints(DT_point* const , DT_point* const , DT_point* const ,
		DT_point* const );
void ma_prepare_step(DT_point* const , DT_point* const , const DT_bool);
void ma_do_step(DT_point* const , DT_point* const , const DT_bool);

void ma_sl_prepareLeg(DT_leg* const , const DT_point* const ,
		const DT_transformation* const );
void ma_sl_action();

void slave();
void sl_status();
void sl_point();

int main() {
	XM_init_cpu();
	XM_init_dnx();
	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	XM_init_com(cpuID);

	XM_LED_OFF

	trans_r = KIN_getTransMat(&leg_r);
	trans_l = KIN_getTransMat(&leg_l);

	XM_LED_ON

	switch (cpuID) {
	case COM_MASTER:
		DEBUG(("Master",sizeof("Master")))
		master();
		break;
	case COM_SLAVE1B:
		DEBUG(("Slave1",sizeof("Slave1")))
		slave();
		break;
	case COM_SLAVE3F:
		DEBUG(("Slave3",sizeof("Slave3")))
		slave();
		break;
	default: //case NOCPUID:
DEBUG		(("NoCpuID",sizeof("NoCpuID")))
		XM_LED_OFF
		break;
	}

	return 0;
}

/* ___ Methoden fuer Master ___ */
void master() {
	DEBUG(("ma_chk_al",sizeof("ma_chk_al")))
	ma_checkAlive();

	DEBUG(("ma_int_pnt",sizeof("ma_int_pnt")))
	DT_point pFntDwn, pFntUp, pBckUp, pBckDwn;
	ma_setPoints(&pFntDwn, &pFntUp, &pBckUp, &pBckDwn);

	DT_byte state = 0;

	// Automat: 0(->1->2->3->4)+
	while (1) {
		XM_LED_ON

		switch (state) {
		case 0:
			DEBUG(("ma_int_pos",sizeof("ma_int_pos")))
			ma_setInitialPosition();
			UTL_wait(40);
			state = 1;
			break;
		case 1:
			ma_prepare_step(&pFntDwn, &pBckUp, false);
			state = 2;
			break;
		case 2:
			ma_do_step(&pBckDwn, &pFntUp, false);
			state = 3;
			break;
		case 3:
			ma_prepare_step(&pFntDwn, &pBckUp, true);
			state = 4;
			break;
		case 4:
			ma_prepare_step(&pBckDwn, &pFntUp, true);
			state = 1;
			break;
		default:
			DEBUG(("ma_err",sizeof("ma_err")))
			break;
		}
	}
}

void ma_prepare_step(DT_point* const pDwn, DT_point* const pUp, DT_bool right) {
	DT_byte config;
	DT_point pTmp;
	// F_R, B_R, M_L: vorne unten
	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y += SL_DST_Y;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		ma_sl_prepareLeg(&leg_r, &pTmp, &trans_r);
	} else {
		pTmp.x = -pDwn->x;
		ma_sl_prepareLeg(&leg_l, &pTmp, &trans_l);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	ma_sl_action();

	// F_L, B_L, M_R: hinten oben
	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y += SL_DST_Y;

	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		pTmp.x = -pDwn->x;
		ma_sl_prepareLeg(&leg_l, &pTmp, &trans_l);
	} else {
		ma_sl_prepareLeg(&leg_r, &pTmp, &trans_r);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	ma_sl_action();
}

void ma_do_step(DT_point* const pDwn, DT_point* const pUp, DT_bool right) {
	DT_byte config;
	DT_point pTmp;
	// F_R, B_R, M_L: vorne unten
	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y += SL_DST_Y;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		ma_sl_prepareLeg(&leg_r, &pTmp, &trans_r);
	} else {
		pTmp.x = -pDwn->x;
		ma_sl_prepareLeg(&leg_l, &pTmp, &trans_l);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	ma_sl_action();

	// F_L, B_L, M_R: hinten oben
	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y += SL_DST_Y;

	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		pTmp.x = -pDwn->x;
		ma_sl_prepareLeg(&leg_l, &pTmp, &trans_l);
	} else {
		ma_sl_prepareLeg(&leg_r, &pTmp, &trans_r);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	ma_sl_action();
}

void ma_setPoints(DT_point* const pFntDwn, DT_point* const pFntUp,
		DT_point* const pBckUp, DT_point* const pBckDwn) {
	XM_LED_OFF
	// Fix-Koordinaten fuer Master, Berechnung fuer Slaves ueber Offset
	pFntUp->x = 103.4640 + DST_X;
	pFntUp->y = 37.6578;
	pFntUp->z = 101.1041;

	pFntDwn->x = 103.4640 + DST_X;
	pFntDwn->y = 37.6578;
	pFntDwn->z = -129.1041;

	pBckUp->x = 103.4640 + DST_X;
	pBckUp->y = -37.6578;
	pBckUp->z = 101.1041;

	pBckDwn->x = 103.4640 + DST_X;
	pBckDwn->y = -37.6578;
	pBckDwn->z = -129.1041;

DEBUG	(("ma_set_pnt",sizeof("ma_set_pnt")))
	XM_LED_ON
}

void ma_checkAlive() {
	// CHECK CPUs
	DT_bool isAlive = false;
	XM_LED_OFF
	do {
		if (COM_isAlive(COM_SLAVE1B) /*&& COM_isAlive(COM_SLAVE3)*/) {
			isAlive = true;
		} else
			UTL_wait(5);
	} while (isAlive == false);
DEBUG	(("ma_alv_sl",sizeof("ma_alv_sl")))
	XM_LED_ON
}

void ma_setInitialPosition() {
	XM_LED_OFF
	DT_byte config;
	// Punkt fuer Null-Stellung mittleres rechtes Bein als Bezug
	DT_point pTmp;
	pTmp.x = 190 + DST_X;
	pTmp.y = 0;
	pTmp.z = -14;
	const DT_point pNull = pTmp;

	// Master rechts
	// pTmp = pNull;
	// pTmp.x = pNull.x;
	// pTmp.y = pNull.y + 0;
	ma_sl_prepareLeg(&leg_r, &pTmp, &trans_r);

	// Master links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	// pTmp.y = pNull.y + 0;
	ma_sl_prepareLeg(&leg_l, &pTmp, &trans_l);

	// Slave3 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y + SL_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	// Slave3 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y + SL_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	// Slave1 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y - SL_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	// Slave1 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y - SL_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	ma_sl_action();
	COM_sendAction(COM_BRDCAST_ID);

DEBUG(("ma_int_pos_ok",sizeof("ma_int_pos_ok")))
	XM_LED_ON
}

/* ___ Methoden fuer Slaves ___ */
void slave() {
	while (1) {
		XM_LED_OFF
		len = COM_receive(&XM_com_data3, result);

		if (len == 0)
			continue;DEBUG(("sl_pck_rec",sizeof("sl_pck_rec")))
		if (result[2] != cpuID && result[2] != COM_BRDCAST_ID)
			continue;
DEBUG		(("sl_pck_acc",sizeof("sl_pck_acc")))

		XM_LED_ON
		switch (result[4]) {
			case COM_STATUS:
			DEBUG(("sl_rec_sts",sizeof("sl_rec_sts")))
			sl_status();
			break;
			case COM_ACTION:
			DEBUG(("sl_rec_act",sizeof("sl_rec_act")))
			ma_sl_action();
			break;
			case COM_POINT:
			DEBUG(("sl_rec_pnt",sizeof("sl_rec_pnt")))
			sl_point();
			break;
			default:
			DEBUG(("sl_err",sizeof("sl_err")))
			break;
		}
	}

}

void sl_status() {
	switch (result[5]) {
	case COM_IS_ALIVE:
		COM_sendACK(COM_MASTER);
		DEBUG(("sl_snd_ack",sizeof("sl_snd_ack")))
		break;
	default:
		break;
	}
}

void sl_point() {
	DT_point pGlobal;

	pGlobal = COM_getPointFromPacket(result);
	if (COM_isLeftLeg(result)) {
		ma_sl_prepareLeg(&leg_l, &pGlobal, &trans_l);
	}
	if (COM_isRightLeg(result)) {
		ma_sl_prepareLeg(&leg_r, &pGlobal, &trans_r);
	}

	COM_sendACK(COM_MASTER);
}

/* ___ Methoden fuer Master und Slave ___ */
void ma_sl_action() {
	DNX_sendAction(leg_r.hip.id);
	DNX_sendAction(leg_r.knee.id);
	DNX_sendAction(leg_r.foot.id);

	DNX_sendAction(leg_l.hip.id);
	DNX_sendAction(leg_l.knee.id);
	DNX_sendAction(leg_l.foot.id);
}

void ma_sl_prepareLeg(DT_leg* const leg, const DT_point* const pGlobal,
		const DT_transformation* const trans) {
	DT_point pLocal = KIN_calcLocalPoint(pGlobal, trans);
	KIN_calcServos(&pLocal, leg);

	DNX_setAngle(leg->hip.id, leg->hip.set_value, true);
	DNX_setAngle(leg->knee.id, leg->knee.set_value, true);
	DNX_setAngle(leg->foot.id, leg->foot.set_value, true);
}

#endif /* TEST_ON */

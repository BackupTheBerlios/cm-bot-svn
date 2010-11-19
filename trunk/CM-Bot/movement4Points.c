/**
 * \file movement4Points.c
 *
 *  \brief	Algorithmus fuer das Vorwaertslaufen ueber 4 Punkte.
 */

#define TEST_OFF TEST
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/communication.h"

// TODO Versatz eintragen
#define SL_DST_Y 0

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
void ma_state1(DT_point* const , DT_point* const );
void ma_state2(DT_point* const , DT_point* const );
void ma_state3(DT_point* const , DT_point* const );
void ma_state4(DT_point* const , DT_point* const );

void ma_sl_prepareLeg(DT_leg* const , const DT_point* const ,
		const DT_transformation* const );
void ma_sl_action();

void slave();
void sl_status();
void sl_point();

int main() {
	XM_init_cpu();
	XM_init_dnx();
	XM_init_com();

	XM_LED_OFF

	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	trans_r = KIN_getTransMat(&leg_r);
	trans_l = KIN_getTransMat(&leg_l);

	XM_LED_ON

	switch (cpuID) {
	case COM_MASTER:
		DEBUG(("Master",sizeof("Master")))
		master();
		break;
	case COM_SLAVE1:
		DEBUG(("Slave1",sizeof("Slave1")))
		slave();
		break;
	case COM_SLAVE3:
		DEBUG(("Slave3",sizeof("Slave3")))
		slave();
		break;
	default: //case NOCPUID:
		DEBUG(("NoCpuID",sizeof("NoCpuID")))
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
			state = 1;
			break;
		case 1:
			ma_state1(&pFntDwn, &pBckUp);
			state = 2;
			break;
		case 2:
			ma_state2(&pBckDwn, &pFntUp);
			state = 3;
			break;
		case 3:
			ma_state3(&pFntDwn, &pBckUp);
			state = 4;
			break;
		case 4:
			ma_state4(&pBckDwn, &pFntUp);
			state = 1;
			break;
		default:
			DEBUG(("ma_err",sizeof("ma_err")))
			break;
		}
	}
}

void ma_state1(DT_point* const pDwn, DT_point* const pUp) {
	// TODO rechts/links bei COM_sendPoint
	DT_point pTmp;
	// F_R, B_R, M_L: vorne unten
	pTmp = *pDwn;
	// pTmp.x = -pDwn->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pDwn;
	// pTmp.x = -pDown->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_l, pDwn, &trans_l);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();

	// TODO evt pause bzw warten bis beine unten sind

	// F_L, B_L, M_R: hinten oben
	pTmp = *pUp;
	pTmp.x = -pUp->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pUp;
	pTmp.x = -pUp->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_r, pUp, &trans_r);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();
}

void ma_state2(DT_point* const pDwn, DT_point* const pUp) {
	// TODO rechts/links bei COM_sendPoint
	DT_point pTmp;
	// F_R, B_R, M_L: hinten unten
	pTmp = *pDwn;
	// pTmp.x = -pDwn->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pDwn;
	// pTmp.x = -pDown->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_l, pDwn, &trans_l);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();

	// TODO evt pause bzw warten bis beine unten sind

	// F_L, B_L, M_R: vorne oben
	pTmp = *pUp;
	pTmp.x = -pUp->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pUp;
	pTmp.x = -pUp->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_r, pUp, &trans_r);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();
}

void ma_state3(DT_point* const pDwn, DT_point* const pUp) {
	// TODO rechts/links bei COM_sendPoint
	DT_point pTmp;
	// F_L, B_L, M_R: vorne unten
	pTmp = *pDwn;
	pTmp.x = -pDwn->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pDwn;
	pTmp.x = -pDwn->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_r, pDwn, &trans_r);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();

	// TODO evt pause bzw warten bis beine unten sind

	// F_R, B_R, M_L: hinten oben
	pTmp = *pUp;
	// pTmp.x = -pDwn->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pUp;
	// pTmp.x = -pDown->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_l, pUp, &trans_l);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();
}

void ma_state4(DT_point* const pDwn, DT_point* const pUp) {
	// TODO rechts/links bei COM_sendPoint
	DT_point pTmp;
	// F_L, B_L, M_R: hinten unten
	pTmp = *pDwn;
	pTmp.x = -pDwn->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pDwn;
	pTmp.x = -pDwn->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_r, pDwn, &trans_r);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();

	// TODO evt pause bzw warten bis beine unten sind

	// F_R, B_R, M_L: vorne oben
	pTmp = *pUp;
	// pTmp.x = -pDwn->x;
	pTmp.y += SL_DST_Y;
	COM_sendPoint(COM_SLAVE1, &pTmp); // TODO slave nr pruefen

	pTmp = *pUp;
	// pTmp.x = -pDown->x;
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE3, &pTmp); // TODO slave nr pruefen

	ma_sl_prepareLeg(&leg_l, pUp, &trans_l);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();
}

void ma_setPoints(DT_point* const pFntDwn, DT_point* const pFntUp,
		DT_point* const pBckUp, DT_point* const pBckDwn) {
	XM_LED_OFF
	// Fix-Koordinaten fuer Master, Berechnung fuer Slaves ueber Offset
	pFntUp->x = 0;
	pFntUp->y = 0;
	pFntUp->z = 0;

	pFntDwn->x = 77.8553;
	pFntDwn->y = 77.8553;
	pFntDwn->z = -129.1041;

	pBckUp->x = 0;
	pBckUp->y = 0;
	pBckUp->z = 0;

	pBckDwn->x = 95.9985;
	pBckDwn->y = -95.9985;
	pBckDwn->z = -116.2699;

	DEBUG(("ma_set_pnt",sizeof("ma_set_pnt")))
	XM_LED_ON
}

void ma_checkAlive() {
	// CHECK CPUs
	DT_bool isAlive = false;
	XM_LED_OFF
	do {
		if (COM_isAlive(COM_SLAVE1) && COM_isAlive(COM_SLAVE3)) {
			isAlive = true;
		} else
			UTL_wait(5);
	} while (isAlive == false);
	DEBUG(("ma_alv_sl",sizeof("ma_alv_sl")))
	XM_LED_ON
}

void ma_setInitialPosition() {
	XM_LED_OFF
	// Punkt fuer Null-Stellung mittleres rechtes Bein als Bezug
	DT_point pTmp;
	pTmp.x = 0;
	pTmp.y = 0;
	pTmp.z = 0;
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

	// Slave1 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y + SL_DST_Y; // +/- pruefen
	COM_sendPoint(COM_SLAVE1, &pTmp);

	// Slave1 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y + SL_DST_Y; // +/- pruefen
	COM_sendPoint(COM_SLAVE1, &pTmp);

	// Slave3 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y - SL_DST_Y; // +/- pruefen
	COM_sendPoint(COM_SLAVE3, &pTmp);

	// Slave3 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y - SL_DST_Y; // +/- pruefen
	COM_sendPoint(COM_SLAVE3, &pTmp);

	COM_sendAction(COM_BRDCAST_ID);
	void ma_sl_action();

	DEBUG(("ma_int_pos_ok",sizeof("ma_int_pos_ok")))
	XM_LED_ON
}

/* ___ Methoden fuer Slaves ___ */
void slave() {
	while (1) {
		XM_LED_OFF
		len = COM_receive(&XM_com_data, result);

		if (len == 0)
			continue;
		DEBUG(("sl_pck_rec",sizeof("sl_pck_rec")))
		if (result[2] != cpuID && result[2] != COM_BRDCAST_ID)
			continue;
		DEBUG(("sl_pck_acc",sizeof("sl_pck_acc")))

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
	// TODO Unterscheidung Links/Rechts
	ma_sl_prepareLeg(&leg_r, &pGlobal, &trans_r);

	COM_sendACK(COM_MASTER);
}

/* ___ Methoden fuer Master und Slave ___ */
void ma_sl_action() {
	/* TODO
	 DNX_sendAction(leg_r.hip.id);
	 DNX_sendAction(leg_r.knee.id);
	 DNX_sendAction(leg_r.foot.id);

	 DNX_sendAction(leg_l.hip.id);
	 DNX_sendAction(leg_l.knee.id);
	 DNX_sendAction(leg_l.foot.id);
	 */
}

void ma_sl_prepareLeg(DT_leg* const leg, const DT_point* const pGlobal,
		const DT_transformation* const trans) {
	DT_point pLocal = KIN_calcLocalPoint(pGlobal, trans);
	KIN_calcServos(&pLocal, leg);

	DNX_setAngle(leg->hip.id, leg->hip.set_value);
	DNX_setAngle(leg->knee.id, leg->knee.set_value);
	DNX_setAngle(leg->foot.id, leg->foot.set_value);
}

#endif /* TEST_ON */

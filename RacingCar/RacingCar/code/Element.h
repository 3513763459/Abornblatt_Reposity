/*
 * Element.h
 *
 *  Created on: Mar 22, 2024
 *      Author: LINEAR
 */

#ifndef CODE_ELEMENT_H_
#define CODE_ELEMENT_H_
#include "stdint.h"
#include "Camera.h"
#include "System.h"

typedef enum
{
    ELEMENMT_NONE,
    CROSS_IN,
    CROSS_U_EXIST,
    CROSS_U_DISMISS,

    LEFT_CIRCLE_IN,
    RIGHT_CIRCLE_IN,
    CIRCLE_PREV_D_M,
    CIRCLE_PREV_M,
    CIRCLE_WILL_IN,
    CIRCLE_HAVE_IN,
    CIRCLE_OUT_D,
    CIRCLE_OUT_U,

}Element_TendTypedef;


typedef struct
{
float Straight_Slope;
float Flat_Slope;
}Element_JustTypedef;
extern uint8_t iptsrd[POINTS_MAX_LEN][2];
extern uint8_t iptsld[POINTS_MAX_LEN][2];
extern uint8_t iptsld_num, iptsrd_num;

extern uint8_t iptslu[POINTS_MAX_LEN][2];
extern uint8_t iptsru[POINTS_MAX_LEN][2];
extern uint8_t iptslu_num, iptsru_num;
extern Element_TendTypedef Element_DataEnum;
extern Element_JustTypedef Element_JustStruct;
void find_RD_point(uint8_t (* ipts1)[2],uint8_t * ipts1_num);
void find_LD_point(uint8_t (*ipts0)[2],uint8_t *ipts0_num);
void find_RU_Point(image_t *image,uint8_t(*ipts1)[2]);
void find_LU_point(image_t * image,uint8_t (*ipts0)[2]);
void Trim_DU(Element_DataTypedef * sptr,uint8_t(*ipts)[2],uint8_t * ipts_num,uint8_t (*iptsc)[2]);
void Element_Info_Deliver(Element_DataTypedef * sptr);
void Element_Phase(void);
void Element_Knee(void);
void Element_Handle(void);
void Plug_in_line(image_t *image,uint8_t plug,uint8_t (*ipts)[2],uint8_t (*iptso)[2],uint8_t iptso_num,bool mode);
#endif /* CODE_ELEMENT_H_ */

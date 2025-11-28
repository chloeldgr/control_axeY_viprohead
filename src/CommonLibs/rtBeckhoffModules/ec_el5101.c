#include "ec_el5101.h"
#define PFX "EL5101 : "

static EL5101_STRUCT ec_el5101[MAX_NB_EL5101];

#define Beckhoff_EL5101 0x00000002, 0x13ed3052

ec_pdo_entry_info_t slave_EL5101_pdo_entries[] = {
    {0x7010, 0x01, 1},
    {0x7010, 0x02, 1},
    {0x7010, 0x03, 1},
    {0x7010, 0x04, 1},
    {0x0000, 0x00, 4}, /* Gap */
    {0x0000, 0x00, 8}, /* Gap */
    {0x7010, 0x11, 32},
    {0x6010, 0x01, 1},/* Latch C Valid*/
    {0x6010, 0x02, 1},/* Latch Extern valid*/
    {0x6010, 0x03, 1},/* Set Counter Done*/
    {0x6010, 0x04, 1},/* Counter underflow*/
    {0x6010, 0x05, 1},/* Counter overflow*/
    {0x6010, 0x06, 1},/* status of input status*/
    {0x6010, 0x07, 1},/* Open Circuit*/
    {0x6010, 0x08, 1},/* Extrapolation stall*/
    {0x6010, 0x09, 1},/* Status of Input A*/
    {0x6010, 0x0a, 1},/* Status of Input B*/
    {0x6010, 0x0b, 1},/* Status of Input C*/
    {0x6010, 0x0c, 1},/* Status of Input gate*/
    {0x6010, 0x0d, 1},/* Status of Input extern latch*/
    {0x1c32, 0x20, 1},
    {0x1804, 0x07, 1},
    {0x1804, 0x09, 1},
    {0x6010, 0x11, 32},/*Counter Value*/
    {0x6010, 0x12, 32},/*Latch Value*/
};

ec_pdo_info_t slave_EL5101_pdos[] = {
    {0x1603, 7, slave_EL5101_pdo_entries + 0},
    {0x1a04, 18, slave_EL5101_pdo_entries + 7},
};


ec_sync_info_t slave_EL5101_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_EL5101_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_EL5101_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

int EL5101_Config(  unsigned int    id,
                    uint16_t        alias,
                    uint16_t        position,
                    uint8_t         Enable_C_Reset,
                    ec_master_t     *pmaster,
                    ec_domain_t     *pdomain1)
{
    int ret;
    size_t ReturnSize;
    uint32_t abort_code;
    if ( id > MAX_NB_EL5101)
        return EL5101_MAX_NB_ERROR;

    ec_el5101[id].pmaster   = pmaster;
    ec_el5101[id].position  = position;
    ec_el5101[id].alias     = alias;
    ec_el5101[id].pdomain   = pdomain1;


    if (!(ec_el5101[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL5101)))
        return 1;
    if (ecrt_slave_config_pdos(ec_el5101[id].sc , EC_END, slave_EL5101_syncs))
    {
        printf(PFX "Failed to configure PDOs.\n");
        return 1;
    }

    ret=ecrt_master_sdo_download(   pmaster,
                                    position,
                                    0x8010,
                                    0x01,
                                    &Enable_C_Reset,
                                    sizeof(Enable_C_Reset),
                                    &abort_code);
    //Value in
    ec_el5101[id].offset_value_in = ecrt_slave_config_reg_pdo_entry(ec_el5101[id].sc,
                                                                    0x6010,
                                                                    0x11,
                                                                    pdomain1,
                                                                    NULL);
    if (ec_el5101[id].offset_value_in < 0)
    {
        printf(PFX "Failed to register PDO Entry\n");
        return 1;
    }
    //Latch
    ec_el5101[id].offset_latch = ecrt_slave_config_reg_pdo_entry(ec_el5101[id].sc,
                                                                 0x6010,
                                                                 0x12,
                                                                 pdomain1,
                                                                 NULL);
    if (ec_el5101[id].offset_latch < 0)
    {
        printf(PFX "Failed to register PDO Entry\n");
        return 1;
    }
    //Ctrl
    ec_el5101[id].offset_ctrl = ecrt_slave_config_reg_pdo_entry(ec_el5101[id].sc,
                                                                0x7010,
                                                                0x01,
                                                                pdomain1,
                                                                NULL);
    if (ec_el5101[id].offset_ctrl < 0)
    {
        printf(PFX "Failed to register PDO Entry\n");
        return 1;
    }

    //Value out
    ec_el5101[id].offset_value_out = ecrt_slave_config_reg_pdo_entry(ec_el5101[id].sc,
                                                                     0x7010,
                                                                     0x11,
                                                                     pdomain1,
                                                                     NULL);
    if (ec_el5101[id].offset_value_out < 0)
    {
        printf(PFX "Failed to register PDO Entry\n");
        return 1;
    }

//    // Status of input A
//    ec_el5101[id].offset_status_A = ecrt_slave_config_reg_pdo_entry(    ec_el5101[id].sc,
//                                                                        0x6010,
//                                                                        0x09,
//                                                                        pdomain1,
//                                                                        NULL);
//    if (ec_el5101[id].offset_status_A < 0)
//    {
//        printf(PFX "Failed to register PDO Entry\n");
//        return EL5101_FAILED_REG_PDO_ENTRY;
//    }
//    // Status of input B
//    ec_el5101[id].offset_status_B = ecrt_slave_config_reg_pdo_entry(    ec_el5101[id].sc,
//                                                                        0x6010,
//                                                                        0x0A,
//                                                                        pdomain1,
//                                                                        NULL);
//    if (ec_el5101[id].offset_status_B < 0)
//    {
//        printf(PFX "Failed to register PDO Entry\n");
//        return EL5101_FAILED_REG_PDO_ENTRY;
//    }
//    // Status of input C
//    ec_el5101[id].offset_status_C = ecrt_slave_config_reg_pdo_entry(    ec_el5101[id].sc,
//                                                                        0x6010,
//                                                                        0x0B,
//                                                                        pdomain1,
//                                                                        NULL);
//    if (ec_el5101[id].offset_status_C < 0)
//    {
//        printf(PFX "Failed to register PDO Entry\n");
//        return EL5101_FAILED_REG_PDO_ENTRY;
//    }
//    // Open Circuit
//    ec_el5101[id].offset_open_circuit = ecrt_slave_config_reg_pdo_entry(    ec_el5101[id].sc,
//                                                                            0x6010,
//                                                                            0x07,
//                                                                            pdomain1,
//                                                                            NULL);
//    if (ec_el5101[id].offset_open_circuit < 0)
//    {
//        printf(PFX "Failed to register PDO Entry\n");
//        return EL5101_FAILED_REG_PDO_ENTRY;
//    }
//    //



    return 0;
}


int EL5101_Init(uint8_t id, uint8_t *pdomain_pd)
{    
    if ( id > MAX_NB_EL5101)
        return EL5101_MAX_NB_ERROR;
    ec_el5101[id].turn          = 0;
    ec_el5101[id].prevoverfl    = 0;
    ec_el5101[id].prevunderfl   = 0;
    ec_el5101[id].value         = 0;
    ec_el5101[id].lat_val       = 0;
    ec_el5101[id].lat_ext_val   = 0;
    ec_el5101[id].cntset_acc    = 0;
    ec_el5101[id].underflow     = 0;
    ec_el5101[id].overflow      = 0;
    ec_el5101[id].input_status  = 0;
    ec_el5101[id].pdomain_pd    = pdomain_pd;

    // Initialisation des paramètres de la structure
    ec_el5101[id].bInputStatus      = 0;
    ec_el5101[id].bInputAStatus     = 0;
    ec_el5101[id].bInputBStatus     = 0;
    ec_el5101[id].bInputCStatus     = 0;
    ec_el5101[id].bCircuitAFailure  = 0;
    ec_el5101[id].bCircuitBFailure  = 0;
    ec_el5101[id].bCircuitCFailure  = 0;
    ec_el5101[id].bOpenCircuit      = 0;

    return EL5101_NO_ERROR;

}

int EL5101_Status(uint8_t id)
{
    int ret;
    size_t size_result;
    uint32_t abort_code;
    if ( id > MAX_NB_EL5101)
        return EL5101_MAX_NB_ERROR;

//    ec_el5101[id].input_status  = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_input_status);
//    ec_el5101[id].bInputAStatus = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_status_A);
//    ec_el5101[id].bInputBStatus = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_status_B);
//    ec_el5101[id].bInputCStatus = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_status_C);
//    ec_el5101[id].bOpenCircuit  = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_open_circuit);
//    ret = ecrt_master_sdo_upload(   ec_el5101[id].pmaster,
//                                    ec_el5101[id].position,
//                                    0x8010,
//                                    0x0B,
//                                    &ec_el5101[id].bCircuitAFailure,
//                                    sizeof(ec_el5101[id].bCircuitAFailure),
//                                    &size_result,
//                                    &abort_code);
//    if ( ret )
//        return EL5101_MASTER_SDO_UPLOAD_ERROR;

//    ret = ecrt_master_sdo_upload(   ec_el5101[id].pmaster,
//                                    ec_el5101[id].position,
//                                    0x8010,
//                                    0x0C,
//                                    &ec_el5101[id].bCircuitAFailure,
//                                    sizeof(ec_el5101[id].bCircuitBFailure),
//                                    &size_result,
//                                    &abort_code);
//    if ( ret )
//        return EL5101_MASTER_SDO_UPLOAD_ERROR;

//    ret = ecrt_master_sdo_upload(   ec_el5101[id].pmaster,
//                                    ec_el5101[id].position,
//                                    0x8010,
//                                    0x0D,
//                                    &ec_el5101[id].bCircuitAFailure,
//                                    sizeof(ec_el5101[id].bCircuitCFailure),
//                                    &size_result,
//                                    &abort_code);
//    if ( ret )
//        return EL5101_MASTER_SDO_UPLOAD_ERROR;

    return EL5101_NO_ERROR;
}
/**
  * \fn int write_el5101(int id, uint8_t *pdomain_pd, uint16_t value)
  * \brief l'écriture de la valeur sur le compteur n'est valide que lorsqu'il y a un front montant sur CNT_SET
  * \arg    id: identifiant du compteur
  *         pdomain_pd : pointeur sur la domaine auquel appartient le compteur
  *         value : valeur que l'on souhaite affectée au compteur
  * \return code d'erreur
  */

int EL5101_Write(int id, int value)// uint32_t value)
{
    uint8_t control;

    if ( id > MAX_NB_EL5101)
        return EL5101_MAX_NB_ERROR;


    // Lecture du registre de control
    // On vérifie que le bit CNT_SET est à 0 pour pouvoir faire le front montant
    control = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_ctrl);
    if ( (control>>2)& 1 )
        return -1;

    // Ecriture de la valeur codeur dans le registre value_out
    printf("EL5101 %d\n",value);
//    EC_WRITE_U32(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_value_out, value);
    EC_WRITE_S32(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_value_out, value);


    return EL5101_NO_ERROR;
}

int EL5101_Set_CNT_SET(uint8_t id)
{
    uint8_t control;
    control = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_ctrl);

    // On force le bit CNT_SET à 1 (pour le front montant)
    EC_WRITE_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_ctrl, control | 0xff);

    return EL5101_NO_ERROR;
}

int EL5101_Reset_CNT_SET(uint8_t id)
{
    uint8_t control;

    control = EC_READ_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_ctrl);
    if ( (control>>2)& 1 )
        return 0;
    EC_WRITE_U8(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_ctrl, control & 0xfb);
    return 0;
}

int EL5101_Read(uint8_t id, int32_t *encoder_value)
{
    uint8_t control;

    if (id > MAX_NB_EL5101)
        return -1;


    /* Vérification de l'état courant du compteur*/
   // EL5101_Status(id);
    /* Si l'entrée 1 n'est pas à 1 alors on sort*/
   /* if ( !ec_el5101[id].input_status )
        return EL5101_STATUS_INPUT_ERROR;*/

    //control = EC_READ_U8(pdomain_pd+ec_el5101[id].offset_ctrl);
    // printf("Write to EL5101 %d\n", value);
    if (( ec_el5101[id].prevoverfl == 0 ) && (ec_el5101[id].overflow ==1))
         ec_el5101[id].turn++;

     //Underflow de l'encodeur? (Bit 3 de status: underflow)
     else if ((ec_el5101[id].prevunderfl == 0) && (ec_el5101[id].underflow== 1))
         ec_el5101[id].turn--;

     ec_el5101[id].prevoverfl = ec_el5101[id].overflow;
     ec_el5101[id].prevunderfl = ec_el5101[id].underflow;

    ec_el5101[id].value = EC_READ_U32(ec_el5101[id].pdomain_pd+ec_el5101[id].offset_value_in);
    ec_el5101[id].value += ec_el5101[id].turn*2^32;

    *encoder_value =   (int32_t)ec_el5101[id].value;
    //EL5101_Reset_CNT_SET(id);

    return 0;
}

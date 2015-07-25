#include "rtl_433.h"

/* Currently this can decode the temperature and id from Rubicson sensors
 *
 * the sensor sends 36 bits 12 times pwm modulated
 * the data is grouped into 9 nibles
 * [id0] [id1], [unk0] [temp0], [temp1] [temp2], [unk1] [unk2], [unk3]
 *
 * sauna sensor sends only temperature on nibbles 5 6 7 (temp2, unk1, unk2)
 *
 * The id changes when the battery is changed in the sensor.
 * unk0 is always 1 0 0 0, most likely 2 channel bits as the sensor can recevice 3 channels
 * unk1-3 changes and the meaning is unknown
 * temp is 12 bit signed scaled by 10
 *
 * The sensor can be bought at Kjell&Co
 */

static int rubicson_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS],int16_t bits_per_row[BITBUF_ROWS]) {
    int temperature_before_dec;
    int temperature_after_dec;
    int16_t temp = 0;
    int16_t humidity = 0;
    int ch;
    unsigned candidate_rows = 0;
    int i,j;
    uint8_t *datarow = NULL;
    uint8_t *candidate_row = NULL;
    unsigned candidate_val = 0;
    for(i=1,candidate_rows=bits_per_row[0]>0;bits_per_row[i]>0;++i)
        ++candidate_rows;
    if(!(candidate_rows == 12||candidate_rows == 7))
        return 0 ;
    
    if (bits_per_row[0]==36)
        candidate_row = bb[0];
    
    for (i=1;bits_per_row[i]==36&&datarow==NULL;++i){
        j=i;
        if (!candidate_row)
            candidate_row = bb[j++];
        for (; bits_per_row[j]==36; ++j) {
            if (memcmp(candidate_row,bb[j],5)==0)
                ++candidate_val;
        }
        /* sauna sensor send 7 rows with last row missing last bit,
         compare 4 bytes(no data in last byte anyway) */
        if (candidate_rows == 7 && bits_per_row[0]==0 && bits_per_row[7] == 35)
            if (memcmp(candidate_row,bb[7],4)==0)
                ++candidate_val;
        /* if found matches more than half of sent codes,
         without counting candidate row, accepts as good signal*/
        if(candidate_val++ > candidate_rows/2){
            //printf("got candidate with %u/%u hits, accepting as datarow \n",candidate_val,candidate_rows);
            datarow = candidate_row;
        } else {
            candidate_row=NULL;
            candidate_val = 0;
        }
    }

    /* FIXME validate the received message better, figure out crc */
    if (datarow[0]&&datarow[1]) {
        /* channel number appears to last two bits of nibble 2,
         increment by 1 to match switch numbering */
        ch = ((datarow[1] &0x30)>>4) +1;
        /* sauna sensor sends on channel 4 (1-3 selection-switch on other transmitters)*/
        if (ch==4){
            temp = datarow[3] | ((datarow[2] & 0xf)<<8);
        }else {
            /* Nible 3,4,5 contains 12 bits of temperature
             * The temerature is signed and scaled by 10 */
            temp = (int16_t)((uint16_t)(datarow[1] << 12) | (datarow[2] << 4));
            temp = temp >> 4;
            /* Nible 7,8 contain humidity value */
            humidity = (int16_t)(((datarow[3]&0x0F)<<4)|(datarow[4]>>4));
        }
        
        
        temperature_before_dec = abs(temp / 10);
        temperature_after_dec = abs(temp % 10);

        fprintf(stdout, "Sensor temperature event:\n");
        fprintf(stdout, "protocol       = Rubicson/Auriol, %d bits\n",bits_per_row[1]);
        fprintf(stdout, "rid            = %x\n",datarow[0]);
        fprintf(stdout, "temp           = %s%d.%d\n",temp<0?"-":"",temperature_before_dec, temperature_after_dec);
        fprintf(stdout, "ch             = %u\n",ch);
        fprintf(stdout, "humidity       = %d\n",humidity);
        fprintf(stdout, "%02x %02x %02x %02x %02x\n",datarow[0],datarow[1],datarow[2],datarow[3],datarow[4]);

        if (debug_output)
            debug_callback(bb, bits_per_row);

        return 1;
    }
    return 0;
}

// timings based on samp_rate=1024000
r_device rubicson = {
    /* .name           = */ "Rubicson Temperature Sensor",
    /* .modulation     = */ OOK_PWM_D,
    /* .short_limit    = */ 1744/4,
    /* .long_limit     = */ 3500/4,
    /* .reset_limit    = */ 5000/4,
    /* .json_callback  = */ &rubicson_callback,
};


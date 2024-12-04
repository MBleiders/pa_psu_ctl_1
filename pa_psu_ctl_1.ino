#include "misc.h"
#include "ad7293.h"

////////////////////////////////////////////

#define LED0_PIN 25
#define BUTTON1_PIN 8
#define BUTTON2_PIN 7
#define BUTTON3_PIN 6
#define AD_ALERT0 9

#define LED1_PIN 0
#define LED2_PIN 1
#define PSU_EN_PIN 2
#define PSU_PG_PIN 3
#define FAN_CTL_PIN 4
#define PREAMP_CTL_PIN 5

#define TEMP_SENSOR_PIN 26 //ADC0

struct ad7293_dev* ad7293_obj;
uint16_t data_val;

float taget_amps = 2.0;
float rs0_volts;
float isense0_amps, isense1_amps;
float Ug0_volts, Ug1_volts;
float Ug0_volts_lim = 0, Ug1_volts_lim = 0; 

uint16_t rsx_alerts;
int ad_monitoring_halt = 1;
int open_loop_mode = 0;
int pa_on_state = 0;
int psu_pg_state = 0;
float temperature_degC = 0;
////////////////////////////////////////////
void psu_en(uint8_t en_state){
  if(en_state){
    digitalWrite(PSU_EN_PIN, HIGH);

    while(digitalRead(PSU_PG_PIN) == LOW){
      delay(100); 
      Serial.printf("Enabling the PSU...\n"); 
    }
  }else{
    digitalWrite(PSU_EN_PIN, LOW);

    while(digitalRead(PSU_PG_PIN) == HIGH){
      delay(100); 
      Serial.printf("Disabling the PSU...\n"); 
    }
  }

}

void fan_en(uint8_t en_state){
  if(en_state){
    digitalWrite(FAN_CTL_PIN, HIGH);
  }
  else{
    digitalWrite(FAN_CTL_PIN, LOW);
  }
}

void preamp_en(uint8_t en_state){
  delay(100);

  if(en_state){
    digitalWrite(PREAMP_CTL_PIN, HIGH);
  }
  else{
    digitalWrite(PREAMP_CTL_PIN, LOW);
  }

  delay(100);
}

void rs0_raw_to_voltage(uint16_t raw_in, float* voltage_out){

  *voltage_out = (((float)(raw_in>>4) + 0.5)/4096)*ADC_REF*50;
}

void bi_vout_raw_to_voltage(uint16_t raw_in, float* voltage_out){
  *voltage_out = (((float)(raw_in>>4) + 0.5)/4096)*ADC_REF*8 - 5.0;
}

void isense_raw_to_current(uint16_t raw_in, float* current_out, float r_sense){

  *current_out = 2*(((float)((raw_in>>4) - 0x7ff))/4096)*ADC_REF/(U_SENSE_GAIN*r_sense);
}

uint16_t get_dac_value(float target_amps, float r_sense){
  float u_sense = target_amps*r_sense;
  float dac_vout = u_sense*U_SENSE_GAIN;
  return (uint16_t)(4*DAC_MAX_RANGE*(dac_vout-DAC_OFFSET)/(2*DAC_VREF));
}

void open_loop_enable(uint8_t open_loop_state_in){

  ad_monitoring_halt = 1;
  if(pa_on_state && (open_loop_state_in == 1)){
              
    uint16_t v_out_0, v_out_1; 
    ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT0_MON, &v_out_0);
    bi_vout_raw_to_voltage(v_out_0, &Ug0_volts_lim);
    ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT1_MON, &v_out_1);
    bi_vout_raw_to_voltage(v_out_1, &Ug1_volts_lim);

    Serial.printf("Init Ug0: %0.3f V,  Ug1: %0.3f V\n", Ug0_volts_lim, Ug1_volts_lim);

    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_HL, v_out_0);
    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_LL, v_out_0);

    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_HL, v_out_1);
    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_LL, v_out_1);
    
    Serial.printf("Open loop mode enabled...\n");           
      
    open_loop_mode = 1;
  }
  else{
      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_HL, 0xfff0);
      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_LL, 0);

      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_HL, 0xfff0);
      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_LL, 0);
      Serial.printf("Open loop mode disabled...\n");
      open_loop_mode = 0;
  }

      ad_monitoring_halt = 0;
}

int ad7293_init_and_configure(uint8_t pa_on){

  ad_monitoring_halt = 1;
  int ret = -1;

  ad_reset(1);

  ret = ad7293_init(&ad7293_obj, NULL);
  if(ret != 0)
    return ret;

  open_loop_mode = 0;//reset state always off

  delay(2000);

  //internal ADC ref, ALERT0 clamp:
  data_val = (1 << 7)|(1 << 1);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_GENERAL, data_val, data_val); 
  if(ret != 0)
    return ret;
  //RS0+ and BiVout0, BiVeout1 mon voltage background monitoring enable:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RSX_MON_BG_EN, (1 << 8) | (1 << 4) | (1 << 5)); 
  if(ret != 0)
    return ret;
  //isense ch0, ch1 background monitoring enable:
  data_val = (1 << 0)|(1 << 1);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_ISENSE_BG_EN, data_val, data_val); 
  if(ret != 0)
    return ret;

  //rs0 alarm high and low limits:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RS0_MON_HL, (uint16_t)(4096*25.0/(50.0*ADC_REF)-0.5) << 4);
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RS0_MON_LL, (uint16_t)(4096*5.0/(50.0*ADC_REF)-0.5) << 4);
  if(ret != 0)
    return ret;

  //bipolar dac range: -5..0V:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_OFFSET, (0b10 << 4)); 
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_OFFSET, (0b10 << 4)); 
  if(ret != 0)
    return ret;

  //ramp time:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RAMP_TIME_0, 0xffff); 
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RAMP_TIME_1, 0xffff);
  if(ret != 0)
    return ret;

  //Closed loop time constants and fast-ramp disable (better transient in capacitive load):
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_CL_FR_IT, (0b111 << 0) | (0 << 3) | (0b111 << 4) | (0 << 7));
  if(ret != 0)
    return ret;

  delay(500);
  ////bipolar dac0, dac1 can be clamped by SLEEP0 pin
  //ad7293_spi_write(ad7293_obj, AD7293_REG_DAC_SNOOZE_O, (1 << 4) | (1 << 5)); 

  //PA on
  if(pa_on){
    psu_en(1);
    delay(100);

    data_val = (1 << PA_ON_BIT);
    ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_PA_ON_CTRL, data_val, data_val); 
    if(ret != 0)
      return ret;

    delay(100);    
  }

  //closed loop ch0, ch1 enable
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_INTEGR_CL, (1 << CL0_BIT) | (1 << CL1_BIT) | (1 << INT_CL_LIMIT_CH0) | (1 << INT_CL_LIMIT_CH1));
  if(ret != 0)
    return ret;

  //writing dac registers
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0, (get_dac_value(taget_amps, RSENSE0) << 4));
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1, (get_dac_value(taget_amps+0.05, RSENSE1) << 4));  
  if(ret != 0)
    return ret;

  if(pa_on){
    //Bipolar dac ch0, ch1 enable (unclamping)
    data_val = (1 << BI_VOUT0_BIT)|(1 << BI_VOUT1_BIT);
    ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_DAC_EN, data_val, data_val); 
    if(ret != 0)
      return ret;

    delay(1000);
  }

  //rsx alert to ALERT0 routing:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RSX_MON_ALERT0, (1 << 8) | (1 << 0)); 
  if(ret != 0)
    return ret;

  //ALERT0 to GPIO3:
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_DIGITAL_INOUT_FUNC, (1 << 3), 0);
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_DIGITAL_OUT_EN, (1 << 3));
  if(ret != 0)
    return ret;

  
  if(pa_on){
    pa_on_state = 1;
  }
  else{
    pa_on_state = 0;
  }

  ad_monitoring_halt = 0;
  return 0;
}

int ad7293_power_off(void){

  ad_monitoring_halt = 1;
  int ret = -1;
  
  //DACs in clamp
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_DAC_EN, 0); 
  if(ret != 0)
    return ret;

  delay(100);

  //PA off:
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_PA_ON_CTRL, (1 << PA_ON_BIT), 0); 
  if(ret != 0)
    return ret;

  delay(100);
  psu_en(0);

  pa_on_state = 0;
  ad_monitoring_halt = 0;
  return 0;
}

void setup() {
  Serial.begin(9600);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
}


void loop() {
  int ret = -1;
  uint8_t button1_state = 0;
  uint8_t button2_state = 0;
  uint8_t button3_state = 0;

  while(1){

    if((digitalRead(BUTTON1_PIN) == LOW) && (button1_state == 0)){
      Serial.printf("Re-configuring...\n");

      int ret = ad7293_init_and_configure(1);
      if(ret != 0){
        ad_monitoring_halt = 2;
      }
      else{
        preamp_en(1);
      }
      
      Serial.printf("ad7293_init_and_configure() result: %d\n", ret);

      fan_en(1);

      button1_state = 1;
    }else if(digitalRead(BUTTON1_PIN) == HIGH){
        button1_state = 0;
    }

    if((digitalRead(BUTTON2_PIN) == LOW) && (button2_state == 0)){
      Serial.printf("Powering off...\n");

      int ret = ad7293_power_off();
      preamp_en(0);
      fan_en(0);
      open_loop_enable(0);

      if(ret != 0){
        ad_monitoring_halt = 2;
      }

      Serial.printf("ad7293_power_off() result: %d\n", ret);
    
      button2_state = 1;
    }else if(digitalRead(BUTTON2_PIN) == HIGH){
        button2_state = 0;
    }

    if((digitalRead(BUTTON3_PIN) == LOW) && (button3_state == 0)){//this is latched button

      if(open_loop_mode > 0){
        open_loop_enable(0);
      }else{
        open_loop_enable(1);
      }
        
      button3_state = 1;

    }else if(digitalRead(BUTTON3_PIN) == HIGH){
      button3_state = 0;
    }


    if(open_loop_mode){
      digitalWrite(LED1_PIN, HIGH);
    }
    else{
      digitalWrite(LED1_PIN, LOW);
      Ug0_volts_lim = 0;
      Ug1_volts_lim = 0;
    }
    delay(100);     
  }

}

void setup1() {
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(AD_ALERT0, INPUT);
  
  pinMode(PSU_EN_PIN, OUTPUT);
  pinMode(PSU_PG_PIN, INPUT); 

  pinMode(FAN_CTL_PIN, OUTPUT);
  pinMode(PREAMP_CTL_PIN, OUTPUT);

  psu_en(0);
  fan_en(0);
  preamp_en(0);

  int ret = ad7293_init_and_configure(0);
  Serial.printf("ad7293_init_and_configure() result: %d\n", ret);

  if(ret != 0){
    ad_monitoring_halt = 2;
  }
}

void loop1() {
    uint16_t adc_raw;

    while(1){
      if(ad_monitoring_halt == 0){//if in halt state, ether AD is not ready or is used in other loop
        ad7293_spi_read(ad7293_obj, AD7293_REG_RS0_MON, &adc_raw);
        rs0_raw_to_voltage(adc_raw, &rs0_volts);

        ad7293_spi_read(ad7293_obj, AD7293_REG_ISENSE_0, &adc_raw);
        isense_raw_to_current(adc_raw, &isense0_amps, RSENSE0);

        ad7293_spi_read(ad7293_obj, AD7293_REG_ISENSE_1, &adc_raw);
        isense_raw_to_current(adc_raw, &isense1_amps, RSENSE1);
        
        ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT0_MON, &adc_raw);
        bi_vout_raw_to_voltage(adc_raw, &Ug0_volts);

        ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT1_MON, &adc_raw);
        bi_vout_raw_to_voltage(adc_raw, &Ug1_volts);

        ad7293_spi_read(ad7293_obj, AD7293_REG_RSX_MON_ALERT, &rsx_alerts);
        uint8_t rs0_alert_high = (rsx_alerts>>8)&0x1;
        uint8_t rs0_alert_low = (rsx_alerts>>0)&0x1;

        uint8_t alert0_state = 0;
        if(digitalRead(AD_ALERT0)){
          alert0_state = 1;
          pa_on_state = 0;
          digitalWrite(LED2_PIN, HIGH);
        }else{
          digitalWrite(LED2_PIN, LOW);
        }

        Serial.printf("pon: %d, pg: %d, olm: %d, rs0: %0.3f V, rs0_hi: %u, rs0_lo: %u, al0: %u, sen0: %0.3f A, sen1: %0.3f A, Ug0: %0.3f V (%0.3f), Ug1: %0.3f V (%0.3f), tmp: %0.3f degC\n", 
                pa_on_state, psu_pg_state, open_loop_mode, rs0_volts, (unsigned int)rs0_alert_high, (unsigned int)rs0_alert_low, (unsigned int)alert0_state, 
                isense0_amps, isense1_amps, Ug0_volts, Ug0_volts_lim-Ug0_volts, Ug1_volts, Ug1_volts_lim-Ug1_volts, temperature_degC);        
      }else if(ad_monitoring_halt == 2){
        Serial.printf("ad7293 init failed ...\n");
      }
      else{
        Serial.printf("Monitoring halted...\n");
      }

      psu_pg_state = (int)digitalRead(PSU_PG_PIN);

      float voltage = analogRead(TEMP_SENSOR_PIN)*(3300 / 1023.0);
      temperature_degC = voltage / 10;

      digitalWrite(LED0_PIN, LOW);
      delay(500);
      digitalWrite(LED0_PIN, HIGH);
      delay(500);     
  }
}

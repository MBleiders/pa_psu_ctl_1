#include "misc.h"
#include "ad7293.h"

////////////////////////////////////////////

#define LED0_PIN 25
#define BUTTON1_PIN 8
#define BUTTON2_PIN 7
#define AD_ALERT0 9

#define LED1_PIN 0
#define LED2_PIN 1
#define PSU_EN_PIN 2

#define TEMP_SENSOR_PIN 26 //ADC0

struct ad7293_dev* ad7293_obj;
uint16_t data_val;

float taget_amps = 2.0;
float rs0_volts;
float isense0_amps, isense1_amps;
float Ug0_volts, Ug1_volts;
uint16_t rsx_alerts;
int ad_monitoring_halt = 1;
////////////////////////////////////////////


void rs0_raw_to_voltage(uint16_t raw_in, float* voltage_out){

  *voltage_out = (((float)(raw_in>>4) + 0.5)/4096)*ADC_REF*50;
}

void bi_vout_raw_to_voltage(uint16_t raw_in, float* voltage_out){
  *voltage_out = (((float)(raw_in>>4) + 0.5)/4096)*ADC_REF*8 - 5.0;
}

void isense_raw_to_current(uint16_t raw_in, float* current_out){

  *current_out = 2*(((float)((raw_in>>4) - 0x7ff))/4096)*ADC_REF/(U_SENSE_GAIN*RSENSE);
}

uint16_t get_dac_value(float target_amps){
  float u_sense = target_amps*RSENSE;
  float dac_vout = u_sense*U_SENSE_GAIN;
  return (uint16_t)(4*DAC_MAX_RANGE*(dac_vout-DAC_OFFSET)/(2*DAC_VREF));
}

int ad7293_init_and_configure(void){

  ad_monitoring_halt = 1;
  int ret = -1;

  ad_reset(1);

  ret = ad7293_init(&ad7293_obj, NULL);
  if(ret != 0)
    return ret;

  delay(2000);

  //internal ADC ref, ALERT0 clamp:
  data_val = (1 << 7)|(1 << 1);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_GENERAL, data_val, data_val); 
  if(ret != 0)
    return ret;
  //RS0+ ans BiVout0, BiVeout1 mon voltage background monitoring enable:
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
  data_val = (1 << PA_ON_BIT);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_PA_ON_CTRL, data_val, data_val); 
  if(ret != 0)
    return ret;

  delay(100);

  //closed loop ch0, ch1 enable
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_INTEGR_CL, (1 << CL0_BIT) | (1 << CL1_BIT) | (1 << INT_CL_LIMIT_CH0) | (1 << INT_CL_LIMIT_CH1));
  if(ret != 0)
    return ret;

  uint16_t dac_val = get_dac_value(taget_amps);
  //writing dac registers
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0, (dac_val << 4));
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1, (dac_val << 4));  
  if(ret != 0)
    return ret;

  //Bipolar dac ch0, ch1 enable (unclamping)
  data_val = (1 << BI_VOUT0_BIT)|(1 << BI_VOUT1_BIT);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_DAC_EN, data_val, data_val); 
  if(ret != 0)
    return ret;

  delay(1000);
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

  ad_monitoring_halt = 0;
  return 0;
}

void setup() {
  Serial.begin(9600);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
}


void loop() {
  int ret = -1;
  uint8_t button1_state = 0;
  uint8_t button2_state = 0;

  while(1){

    if((digitalRead(BUTTON1_PIN) == LOW) && (button1_state == 0)){
      Serial.printf("Re-configuring...\n");

      int ret = ad7293_init_and_configure();
      if(ret != 0){
        ad_monitoring_halt = 2;
      }

      Serial.printf("ad7293_init_and_configure() result: %d\n", ret);

      button1_state = 1;
    }else if(digitalRead(BUTTON1_PIN) == HIGH){
        button1_state = 0;
    }

    if((digitalRead(BUTTON2_PIN) == LOW) && (button2_state == 0)){
      Serial.printf("Powering off...\n");

      int ret = ad7293_power_off();
      if(ret != 0){
        ad_monitoring_halt = 2;
      }

      Serial.printf("ad7293_power_off() result: %d\n", ret);

      button2_state = 1;
    }else if(digitalRead(BUTTON2_PIN) == HIGH){
        button2_state = 0;
    }

    delay(100);     
  }

}

void setup1() {
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(AD_ALERT0, INPUT);
  
  int ret = ad7293_init_and_configure();
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
        isense_raw_to_current(adc_raw, &isense0_amps);
        ad7293_spi_read(ad7293_obj, AD7293_REG_ISENSE_1, &adc_raw);
        isense_raw_to_current(adc_raw, &isense1_amps);
        ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT0_MON, &adc_raw);
        bi_vout_raw_to_voltage(adc_raw, &Ug0_volts);
        ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT1_MON, &adc_raw);
        bi_vout_raw_to_voltage(adc_raw, &Ug1_volts);
        ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT1_MON, &adc_raw);

        ad7293_spi_read(ad7293_obj, AD7293_REG_RSX_MON_ALERT, &rsx_alerts);
        uint8_t rs0_alert_high = (rsx_alerts>>8)&0x1;
        uint8_t rs0_alert_low = (rsx_alerts>>0)&0x1;

        uint8_t alert0_state = 0;
        if(digitalRead(AD_ALERT0)){
          alert0_state = 1;
          digitalWrite(LED2_PIN, HIGH);
        }else{
          digitalWrite(LED2_PIN, LOW);
        }

        Serial.printf("rs0: %0.3f V, rs0_al_hi: %u, rs0_al_lo: %u, alert0: %u, isense0: %0.3f A, isense1: %0.3f A, Ug0: %0.3f V, Ug1: %0.3f V\n", 
                rs0_volts, (unsigned int)rs0_alert_high, (unsigned int)rs0_alert_low, (unsigned int)alert0_state, 
                isense0_amps, isense1_amps, Ug0_volts, Ug1_volts);        
      }else if(ad_monitoring_halt == 2){
        Serial.printf("ad7293 init failed ...\n");
      }
      else{
        Serial.printf("Monitoring halted...\n");
      }


      digitalWrite(LED0_PIN, LOW);
      delay(500);
      digitalWrite(LED0_PIN, HIGH);
      delay(500);     
  }
}

#include "mbed.h"

//BufferedSerial uart0(P1_7, P1_6,115200);  //TX, RX
SPI spi(P0_9, P0_8, P0_6);    //mosi, miso, sclk
I2C i2c(P0_5,P0_4);
DigitalOut cs1(P0_2);   //adc
DigitalOut cs2(P0_7);   //dac, en, v+
DigitalOut cs3(P0_11);  //dac, vc, v-
DigitalOut rc(P1_5);
DigitalIn tsd(P0_1);
DigitalIn drdy(P0_3);
DigitalIn comp(P1_9);
DigitalIn on(P1_4);
DigitalIn a1(P1_1);
DigitalIn b1(P1_0);
DigitalIn a2(P1_8);
DigitalIn b2(P1_2);

//OLED
const uint8_t oled1=0x78;   //oled i2c addr 0x3c<<1
void oled_init(uint8_t addr);     //lcd init func
void char_disp(uint8_t addr, uint8_t position, char data);    //char disp func
void cont(uint8_t addr,uint8_t val);     //contrast set
void vs_disp(uint8_t addr, uint8_t position, int16_t val);
void is_disp(uint8_t addr, uint8_t position, uint16_t val);
void vm_disp(uint8_t addr, uint8_t position, float val);
void im_disp(uint8_t addr, uint8_t position, float val);
void off_disp(uint8_t addr);

//Rotary state and display
const uint16_t tc_on=4000;    //lcd tick on time
const uint16_t tc_off=3000;    //lcd tick off time
const uint16_t disp_ref=10000;      //val disp refresh rate
uint16_t tc, disp_c;          //tick counter, temp disp refresh counter
uint8_t r1_state, r2_state;       //rotary 1 state, rotary 2 state
uint8_t r1_val, r2_val;       //0->idle, 1->incr, 2->decr
uint8_t cur_pos, tick_pos;

//adc/dac control
const uint8_t rst=0b0110;
const uint8_t wreg=0b0100;
const uint8_t start=0b1000;
void drdy_wait();
int16_t adc_read(uint8_t ch, uint8_t avg);
void dac_send(uint8_t ch, uint16_t val);

//set
#define gv 4     //LT1970A dif amp gain
#define dac_res 65536   //2^16 dac resolution
#define dac_ref 5000   //dac FS. mV unit
#define vs_cal 1.002808//1.005092 //vs calib
#define is_cal 1.003110//0.996115 //is calib
int16_t vs, vs_p, is=100, is_p;       //mV/mA unit
uint8_t en_p;

//meas
#define adc_res 32768 //2^15 adc resolution. unipolar
#define adc_ref 2048   //2.048V adc reference
#define gi 2.6   //AD8226 inst. amp gain. 49.4/(20-1)
#define sens_R 1   //sens R
#define att 0.0869565   //vm att. 10/115
#define vm_cal 1.000534//1.000267 //vm calib
#define im_cal 0.744444//0.747664 //im calib
float im, vm;
uint8_t comp_p=1, tsd_p=1;

void v_set(int16_t val);
void i_set(int16_t val);

int main(){
    spi.format(8,1);
    spi.frequency(5000000); //SPI clk 5MHz
    i2c.frequency(400000);  //I2C clk 400kHz
    cs1=1;
    cs2=1;
    cs3=1;
    rc=0;

    //OLED init
    thread_sleep_for(50);  //wait for OLED power on
    oled_init(oled1);
    thread_sleep_for(10);
    cont(oled1,0xff);
    
    //adc init
    cs1=0;
    spi.write(rst);
    cs1=1;
    thread_sleep_for(10);
    cs1=0;
    spi.write((wreg<<4)+(0x01<<2)+0);    //write addr 0x01, 1byte
    spi.write((0b010<<5)+(0b10<<3)+(0b1<<2));       //180sps, turbo mode, cc mode
    cs1=1;

    char_disp(oled1,7,'V');
    char_disp(oled1,13,'m');
    char_disp(oled1,14,'A');
    char_disp(oled1,0x20+7,'V');
    char_disp(oled1,0x20+13,'m');
    char_disp(oled1,0x20+14,'A');
    off_disp(oled1);
    im_disp(oled1,0x20+9,0);

    while (true){
        //rotary scan
        r1_state=((r1_state<<1)+a1)&0b0011;
        if((r1_state==1)&&(b1==0))r1_val=1;      //r1 incr
        else if((r1_state==1)&&(b1==1))r1_val=2; //r1 decr

        r2_state=((r2_state<<1)+a2)&0b0011;
        if((r2_state==1)&&(b2==0))r2_val=1;      //r2 incr
        else if((r2_state==1)&&(b2==1))r2_val=2; //r2 decr        

        if(r2_val==1){
            if(cur_pos<9) ++cur_pos;
            else cur_pos=0;
        }else if(r2_val==2){
            if(cur_pos>0) --cur_pos;
            else cur_pos=9;
        }

        if(cur_pos==0){
            tick_pos=16;
        }else if(cur_pos==1){
            tick_pos=0;
            if(r1_val==1)vs=abs(vs);
            else if(r1_val==2) vs=-1*abs(vs);
        }else if(cur_pos==2){
            tick_pos=1;
            if(r1_val==1)vs=vs+10000;
            else if(r1_val==2) vs=vs-10000;
        }else if(cur_pos==3){
            tick_pos=2;
            if(r1_val==1)vs=vs+1000;
            else if(r1_val==2) vs=vs-1000;
        }else if(cur_pos==4){
            tick_pos=4;
            if(r1_val==1)vs=vs+100;
            else if(r1_val==2) vs=vs-100;
        }else if(cur_pos==5){
            tick_pos=5;
            if(r1_val==1)vs=vs+10;
            else if(r1_val==2) vs=vs-10;
        }else if(cur_pos==6){
            tick_pos=6;
            if(r1_val==1)vs=vs+1;
            else if(r1_val==2) vs=vs-1;
        }else if(cur_pos==7){
            tick_pos=10;
            if(r1_val==1)is=is+100;
            else if(r1_val==2) is=is-100;
        }else if(cur_pos==8){
            tick_pos=11;
            if(r1_val==1)is=is+10;
            else if(r1_val==2) is=is-10;
        }else if(cur_pos==9){
            tick_pos=12;
            if(r1_val==1)is=is+1;
            else if(r1_val==2) is=is-1;
        }

        r1_val=0;
        r2_val=0;

        //overflow check
        if(vs>=15000)vs=15000;
        if(vs<=-15000)vs=-15000;
        if(is>=400)is=400;
        if(is<=0)is=0;

        //vset/iset display
        if(tc==0){
            vs_disp(oled1,0,vs);
            is_disp(oled1,10,is);
            tc++;
        }else if((tc>0)&&(tc<tc_on)){
            if(vs_p!=vs)vs_disp(oled1,0,vs);
            if(is_p!=is)is_disp(oled1,10,is);
            tc++; 
        }else if(tc==tc_on){
            char_disp(oled1,tick_pos,' ');
            tc++;
        }else if((tc>tc_on)&&(tc<(tc_on+tc_off))){
            tc++;
        }else{
            tc=0;
        }

        if((en_p==1)&&(on==1)){  //sw off changed
            dac_send(0,0);//disable
            i_set(0);
            rc=0;
            v_set(0);
            off_disp(oled1);
            im_disp(oled1,0x20+9,0);
            en_p=0;
        }else if((en_p==0)&&(on==1)){  //sw off steady
            //nothing
        }else if((en_p==0)&&(on==0)){  //sw on changed
            i_set(is);
            rc=1;
            thread_sleep_for(200);
            v_set(vs);
            dac_send(0,65535);//enable
            en_p=1;
        }else{      //sw on steady
            if(vs_p!=vs)v_set(vs);
            if(is_p!=is)i_set(is);
            if(disp_c==disp_ref){
                vm=(float)adc_read(1,0)*adc_ref/adc_res/att*vm_cal;  //dummy read
                im=(float)adc_read(0,2)*adc_ref/adc_res/sens_R/gi*im_cal; //IM
                im_disp(oled1,0x20+9,im);
                disp_c=0;
            }else if(disp_c==(disp_ref/2)){
                im=(float)adc_read(0,0)*adc_ref/adc_res/sens_R/gi*im_cal; //dummy read
                vm=(float)adc_read(1,2)*adc_ref/adc_res/att*vm_cal;  //VM
                vm_disp(oled1,0x20+0,vm);
                disp_c++;
            }else{
                disp_c++;
            }
        }
        vs_p=vs;
        is_p=is;

        //emg
        if(comp_p!=comp){
            if(!comp)char_disp(oled1,0x20+15, 'C');
            else char_disp(oled1,0x20+15, ' ');
        }
        if(tsd_p!=tsd){
            if(!tsd)char_disp(oled1,15, 'T');
            else char_disp(oled1,15, ' ');
        }
        comp_p=comp;
        tsd_p=tsd;
    }
}

//OLED func
void oled_init(uint8_t addr){
    char lcd_data[2];
    lcd_data[0] = 0x0;
    lcd_data[1]=0x01;           //0x01 clear disp
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x02;           //0x02 return home
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x0C;           //0x0c disp on
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x01;           //0x01 clear disp
    i2c.write(addr, lcd_data, 2);
    thread_sleep_for(20);
}

void char_disp(uint8_t addr, uint8_t position, char data){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf, 2);
    buf[0]=0x40;            //ahr disp cmd
    buf[1]=data;
    i2c.write(addr,buf, 2);
}

void cont(uint8_t addr,uint8_t val){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x2a;
    i2c.write(addr,buf,2);
    buf[1]=0x79;    //SD=1
    i2c.write(addr,buf,2);
    buf[1]=0x81;    //contrast set
    i2c.write(addr,buf,2);
    buf[1]=val;    //contrast value
    i2c.write(addr,buf,2);
    buf[1]=0x78;    //SD=0
    i2c.write(addr,buf,2);
    buf[1]=0x28;    //0x2C, 0x28
    i2c.write(addr,buf,2);
}

//dac func
void dac_send(uint8_t ch, uint16_t val){
    uint8_t cmd=0b0011; //Write & update
    uint8_t addr;
    if(ch<=1)cs2=0;
    else cs3=0;
    if((ch==0)||(ch==2)) addr=0b0001;  //DAC A
    else addr=0b1000;       //DAC B
    spi.write((cmd<<4)+addr);  //cmd & addr
    spi.write(val>>8);          //MSB
    spi.write(val&0xff);        //LSB
    if(ch<=1)cs2=1;
    else cs3=1;
}

//adc func
void drdy_wait(){
    while(true){
        if(drdy==0) break;
    }
}

int16_t adc_read(uint8_t ch, uint8_t avg){
    uint8_t i;
    uint8_t buf[2];     //spi receive buf
    int16_t raw_val;
    int32_t total;
    cs1=0;
    spi.write((wreg<<4));       //write addr 0x00, 1byte
    if(ch==0)spi.write((0b0000<<4)+1);   //ch0-1 mux, pga disable. im
    if(ch==1)spi.write((0b0101<<4)+1);   //ch2-3 mux, pga disable. vm
    cs1=1;
    for (i=0;i<avg;++i){
        drdy_wait();
        cs1=0;
        buf[1]=spi.write(0x00);
        buf[0]=spi.write(0x00);
        cs1=1;
        raw_val=(buf[1]<<8)+buf[0];
        total=total+(int32_t)raw_val;
    }
    return (int16_t)(total/avg);
}

void vs_disp(uint8_t addr, uint8_t position, int16_t val){
    char buf[8];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;        //write cmd
    if(val>=0)buf[1]=0x2B;      //+
    else{
        buf[1]=0x2D;    //-
        val=abs(val);
    }
    buf[2]=0x30+(val/10000)%10;//10
    buf[3]=0x30+(val/1000)%10; //1
    buf[4]=0x2E;               //.
    buf[5]=0x30+(val/100)%10;  //0.1
    buf[6]=0x30+(val/10)%10;   //0.01
    buf[7]=0x30+val%10;        //0.001
    i2c.write(addr,buf,8);
}

void is_disp(uint8_t addr, uint8_t position, uint16_t val){
    char buf[4];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;        //write cmd
    buf[1]=0x30+(val/100)%10;//100
    buf[2]=0x30+(val/10)%10; //10
    buf[3]=0x30+val%10;     //1
    i2c.write(addr,buf,4);
}

void vm_disp(uint8_t addr, uint8_t position, float val){
    char buf[8];
    int16_t val_int;
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;        //write cmd
    val_int=(int16_t)val;
    if(val_int>=0)buf[1]=0x2B;      //+
    else{
        buf[1]=0x2D;    //-
        val_int=abs(val_int);
    }
    buf[2]=0x30+(val_int/10000)%10;//10
    buf[3]=0x30+(val_int/1000)%10; //1
    buf[4]=0x2E;                   //.
    buf[5]=0x30+(val_int/100)%10;  //0.1
    buf[6]=0x30+(val_int/10)%10;   //0.01
    buf[7]=0x30+val_int%10;        //0.001
    i2c.write(addr,buf,8);
}

void im_disp(uint8_t addr, uint8_t position, float val){
    char buf[5];
    int16_t val_int;
    uint8_t i;
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;        //write cmd
    val_int=(int16_t)val;
    if(val_int>=0)buf[1]=0x2B;      //+
    else{
        buf[1]=0x2D;    //-
        val_int=abs(val_int);
    }
    buf[2]=0x30+(val_int/100)%10;   //100
    buf[3]=0x30+(val_int/10)%10;        //10
    buf[4]=0x30+val_int%10;   //1
    i2c.write(addr,buf,5);
}

void off_disp(uint8_t addr){
    char buf[8];
    buf[0]=0x0;
    buf[1]=0x80+0x20+0;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf, 2);
    buf[0]=0x40;
    buf[1]=0x20;    //" "
    buf[2]=0x20;    //" "
    buf[3]=0x20;    //" "
    buf[4]=0x4F;    //"O"
    buf[5]=0x46;    //"F"
    buf[6]=0x46;    //"F"
    buf[7]=0x20;    //" "
    i2c.write(addr,buf,8);
}

void v_set(int16_t val){
    float vs_f;
    uint16_t send;
    vs_f=(float)abs(val)*dac_res/dac_ref/gv*vs_cal;
    send=(uint16_t)vs_f;
    if(val>=0){
        dac_send(3,0);//V-
        dac_send(1,send);//V+
    }else{
        dac_send(3,send);//V-
        dac_send(1,0);//V+
    }        
}

void i_set(int16_t val){
    float is_f;
    uint16_t send;
    is_f=(float)abs(val)*10*sens_R/dac_ref*dac_res*is_cal;
    send=(uint16_t)is_f;
    dac_send(2,send);
}

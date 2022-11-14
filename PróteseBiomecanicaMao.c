#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24LO1.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#define SERVOMIN 150 //Esta é a contagem de comprimento de pulso 'mínimo' (de 4096)
#define SERVOMAX 600 //Esta é a contagem de comprimento de pulso 'máximo' (de 4096)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int rf_cen = 9; //Pino de habilitação do chip nRF24
int rf_cs = 8; //Pino nRF24 CS

RF24 rf(rf_cen, r_cs);
//Endereço do tubo - código rígido no lado do uECG
uint8_t pipe_rx[8] = {0x0E, 0xE6, 0x0D, 0xA7, 0, 0, 0, 0};

uint8_t swapbits(uint8_t a){ //O endereço da tubagem uECG utiliza a encomenda de bits trocados
//Inverter a ordem de bits num único byte
    uint8_t v=0;
    if(a & 0x80) v |= 0x01;
    if(a & 0x40) v |= 0x02;
    if(a & 0x20) v |= 0x04;
    if(a & 0x10) v |= 0x08;
    if(a & 0x08) v |= 0x10;
    if(a & 0x04) v |= 0x20;
    if(a & 0x02) v |= 0x40;
    if(a & 0x01) v |= 0x80;
    return v;
}

long last_servo_upd = 0; //Tempo em que atualizamos pela última vez os valores dos servos - não queremos fazer isso com muita freqüência
byte in_pack[32]; //Matriz(Array) para pacotes de RF de entrada
unsigned long unit_ids = {4294963881, 4294943100, 28358}; //Array de IDs uECG conhecidas - necessidade de preencher com suas próprias IDs de unidade
int unit_vals[3] = {0, 0, 0}; //Matriz de valores uECG com estes IDs

float tgt_angles[5]; //Ângulos de alvo para 5 dedos
float cur_angles[5]; //Ângulos de corrente para 5 dedos

float angle_open = 30; //Ângulo que corresponde a um dedo aberto
float angle_closed = 150; //Ângulo que corresponde ao dedo fechado

void setup() {
    //nRF24 requer SPI relativamente lento, provavelmente funcionaria a 2MHz também
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODEO));

    for (int x = 0. x < 8; x++) //nRF24 e uECG têm ordem de bits diferentes para endereço de tubos
        pipe_rx[x] = swapbits(pipe_rx[x]);

    //Configurar parâmetros de rádio
    rf.begin();
    rf.setDataRate(RF24_1MBPS);
    rf.setAddressWidth(4);
    rf.setChannel(22);
    rf.setRetries(0, 0);
    rf.AutoAck(0);
    rf.disableDynamicPayloads();
    rf.setPayloadsSize(32);
    rf.openReadingPipe(0, pipe_rx);
    rf.setCRCLength(RF24_CRC_DISABLED);
    rf.disableCRC();
    rf.startListening();
    //Nota que a uECG deve ser mudada para o modo de dados brutos (através de um longo aperto de botão)
    //Para enviar pacotes compatíveis, por padrão ele envia os dados em modo BLE
    //Não pode ser recebido pelo nRF24

    Serial.begin(115200); //Saída serial - muito útil para depuração
    pwm.begin(); //Inicio PWM driver
    pwm.setPWMfreq(60); //Servos analógicos rodam em atualizações de ~60 Hz
    for(int i = 0; i < 5; i++) //Define as posições iniciais dos dedos
    {
        tgt_angles[i] = angle_open;
        cur_angles[i] = angle_open;
    }   s
}
void setAngle(int n, float angle){ //Enviar o valor do ângulo para determinado canal
    pwm.setPWM(n, 0, SERVOMIN + angle * 0.005556 * (SERVOMAX - SERVOMIN));
}
float angle_speed = 15; //Quão rápido os dedos se movem

float v0 = 0, v1 = 0, v2 = 0; //Valores de atividade muscular filtrados por 3 canais

void loop()
{
    if(rf.available())
    {
        rf.read(in_pack, 32); //Processamento de pacotes
        byte u1 = in_pack[3]; //32-bit unit ID, único para cada dispositivo uECG
        byte u2 = in_pack[4];
        byte u3 = in_pack[5];
        byte u4 = in_pack[6];
        unsigned long id = (u1<<24) | (u2<<16) | (u3<<8) | u4;
        //Serial.println(id); // Descomente esta linha para fazer uma lista de suas identificações uECG
        if(in_pack[7] != 32) id = 0; //Tipo de pacote errado: no modo EMG este byte deve ser 32
        int val = in_pack[10]; //Valor da atividade muscular
        if(val != in_pack[11]) id = 0; //Valor é duplicado em 2 bytes porque o ruído RF pode corromper o pacote, e não temos CRC com nRF24
        
        //Procurar qual ID corresponde à ID atual e valor de preenchimento
        for(int n = 0; n < 3; n++)
            if (id == unit_ids[n])
                unit_vals[n] = val;
    }
    long ms = millis();
    if(ms - last_servo_upd > 20) //Não atualizar servos com muita freqüência
    {
        last_servo_upd = ms;
        for(int n = 0; n < 5; n++) // Passar pelos dedos, se os ângulos de alvo e corrente não corresponderem - ajuste-os
    {
        if(cur_angles[n] < tgt_angles[n] - angle_speed/2) cur_angles[n] += angle_speed;
        if(cur_angles[n] > tgt_angles[n] + angle_speed/2) cur_angles[n] -= angle_speed;
    }
    for(int n = 0; n < 5; n++) //Aplicar ângulos aos dedos
        setAngle(n, cur_angles[n]);
        
    // Média exponencial: evita que picos únicos afetem o estado dos dedos   
    v0 = v0*0.7 + 0.3*(float)unit_vals[0];
    v1 = v1*0.7 + 0.3*(float)unit_vals[1];
    v2 = v2*0.7 + 0.3*(float)unit_vals[2];

    //Cálculo de notas a partir de valores brutos
    float scor0 = 4.0*v0*v0/((v1*0.3 + 20)*(v2*1.3 + 15));
    float scor1 = 4.0*v1*v1/((v0*2.0 + 20)*(v2*2.0 + 20));
    float scor2 = 4.0*v2*v2/((v0*1.2 + 20)*(v1*0.5 + 15));

    // Imprimir pontuação para depuração
    Serial.print(scor0)
    Serial.print(' ');
    Serial.print(scor1)
    Serial.print(' ');
    Serial.println(scor2);

    //comparar cada pontuação com o limiar e mudar os estados dos dedos de forma correspondente
    if(scor2 < 0.5) //sinal fraco - dedo aberto
        tgt_angles[0] = angle_open;
    if(scor2 > 1.0) //Sinal forte - dedo fechado
        tgr_angles[0] = angle_closed;

    if(scor1 < 0.5)
    {
        tgt_angles[1] = angle_open;
        tgt_angles[2] = angle_open;
    }
    if(scor1 > 1.0)
    {
        tgt_angles[1] = angle_closed;
        tgt_angles[2] = angle_closed;
    }
    
    if(scor0 < 0.5)
    {
        tgt_angles[3] = angle_open;
        tgt_angles[4] = angle_open;
    }
    if(scor0 > 1.0)
    {
        tgt_angles[3] = angle_closed;
        tgt_angles[4] = angle_closed;
    }
        
    }
  }
}  
        
 
}

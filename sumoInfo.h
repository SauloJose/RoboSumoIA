#ifndef SUMOINFO_H
#define SUMOINFO_H

#include "driver/mcpwm.h"

/*================|| DEFINIÇÕES ||====================*/
//Definições do sensor ultrasônico
#define ECHO 35
#define TRIG 32

//Definições gerais-=
#define BOT 2
#define NumberIRsensor 1
#define NumberUltrassonics 1

//Velocidade de ataque, rastreamento e distância de ataque.
#define _VATK 100
#define _VRST 80
#define _DATK 30
#define _DATKMIN 10

//Cor de detecção (Opicional)
#define _DETCOR 40

//Definição dos sensores infravermelhos
#define SD1 19
#define SD2 7
#define SD3 5
#define SD4 4

//Lado de giro
#define RST 896  //Para a direita
#define LST 897  //Para a esquerda

//Definições para os motores
//motor esquerdo:
#define _MESQA 12
#define _MESQB 14


//Motor direito:
#define _MDIRA 27
#define _MDIRB 26

//Modos de funcionamento do sensor infravermelho
#define _DigFunc 888
#define _AlogFunc 889

//Struct responsável por pegar os dados dos sensores infravermelhos
typedef struct sensorIR {
  bool State[NumberIRsensor];
} SensorIR;

//enum que relaciona as possíveis funções que o cérebro irá enviar
enum Mov { Forward,   //Para frente
           Backward,  //Para trás
           Left,      //Esquerda
           Right,     //Direita
           TrackL,    //Virar esquerda
           TrackR,    //Virar direita
           Stop,      //Parar
           Test };    //rotina de testes

//Struct para enviar dados de movimento (velocidade e função de movimento)
typedef struct MovOrder {
  Mov func;   //Movimento para o robô
  float vel;  //Velocidade de rotação do robô
} OrderMotor;

/*=================================|| CLASSES ||=======================================*/

//Classe para controlar um motor DC
class motorDC {
private:
  //Atributos
  uint16_t pin1, pin2;                      //Pin1 -> Pino positivo / Pin2 -> Pino negativo
  float Speed = 100.0;                      //Velocidade de rotação (porcentagem)
  mcpwm_unit_t mcpwm_num = MCPWM_UNIT_0;    //Unidade que está configurada o motor
  mcpwm_timer_t timer_num = MCPWM_TIMER_0;  //Timer da unidade correspondente

public:
  //Constructor do motor
  motorDC(
    //Pino positivo do motor DC
    uint16_t _pin1,
    //Pino negativo do motor DC
    uint16_t _pin2,
    //Velocidade inicial do motor.
    uint8_t _Speed,
    //Unidade configurada
    mcpwm_unit_t _MCPWM_UNIT,
    //Timer configurada
    mcpwm_timer_t _MCPWM_TIMER);


  //Construtor sem entradas
  motorDC();

  //Cadastra velocidade do motor
  void SpeedConfig(uint16_t _spd);

  //Girar no sentido direto
  void Forward(float _vel);

  //Girar no sentido inverso
  void Backward(float _vel);

  //Parar o motor
  void Stop();

  //Girar o motor tantos graus.
  void turnM(uint16_t spd, uint16_t angle);
};

//classe que controla o sistema de movimento do robô. ( ) Adaptar valores
class MotorSystem : private motorDC {
private:
  //Velocidade total de ambos os motores.
  float _spd = 100.0;  //Velocidade de rotação do sistema em PWM

  //ME->Motor esquerdo MD->Motor Direitos
  motorDC _ME, _MD;

public:
  //Construtor da classe
  MotorSystem(
    motorDC ME,  //Endereço para controlar o motor do lado esquerdo. Por default, o valor é NULL (Sem endereço).
    motorDC MD,  //Endereço para controlar o motor do lado direito. Por default, o valor é NULL (Sem endereço).
    float spd);  //Velocidade padrão inicial do robô. Por default, o valor é 100

  //MotorSystem();  //Esse construtor é o que se utiliza sem nenhum valor enviado. Aconselhado não utilizar.

  //Andar para frente
  void ForwardCar(float _vel, uint16_t t);  //Andar para frente por t segundos
  void ForwardCar(float _vel);

  //Dar ré
  void BackwardCar(float _vel, uint16_t t);  //Ré por t segundos
  void BackwardCar(float _vel);

  //Parar o robô
  void StopCar(uint16_t t);  //Parar por t segundos
  void StopCar();

  //Andar para a direita
  void RightCar(float _vel, uint16_t t);  //Andar para a direitar por t segundos
  void RightCar(float _vel);

  //Andar para a esquerda
  void LeftCar(float _vel, uint16_t t);  // Andar para a esquerda por t segundos
  void LeftCar(float _vel);

  //Girar o robô x graus
  void turnR(float _vel, uint16_t _angle);

  //Método que inicia rotina de TrackBack (Rastrear) um carro robô inimigo.
  void TrackBack(float _vel, uint8_t _sideOfturn);

  //Método para realizar rotina de teste dos motores
  void TestRoutine(float _vel, uint16_t t);

  //Método para variar a velocidade do sistema de movimento.
  void SpeedConfig(float _spd);
};

//Classe do sensor ultrassÔnico
class ultrassonic {
private:
  uint8_t _pinTrig;  //Pino do TRIG do sensor.
  uint8_t _pinEcho;  //Pino do ECHO do sensor.
  float _distance;   //Distância atual capturada pelo sensor
  unsigned long _timeD;       //Variável para manipulação interna do objeto

public:
  //Construtor
  ultrassonic(
    uint8_t pinTRIG,   //Qual pino está conectado o  TRIG
    uint8_t pinECHO);  //Qual pino está conectado o  ECHO

  //métodos
  float captureDistance();  //Método para retornar a distância que o sensor capturou
};

//Classe do sensor infravermelho;
class sensorIV {
private:
  //uint8_t _pinAn;
  uint8_t _pinSI;                 //Pino digital associado ao módulo
  bool _state;                    //Estado do pino . True (Tem algo captado), False (Não tem nada captado)
  uint8_t _stateFunc = _DigFunc;  //Funciona utilizando a computação analógica.
                                  //_DigFunc: Função pela leitura digital
                                  //_AlogFunc: Função para leitura pela analógica
public:
  /*Construtor*/

  //Responsável por cadastrar o dado do sensor no sistema
  sensorIV(uint8_t pinDG);  //Apenas coloco o pino que ele está conectado.

  /*Métodos*/

  //Retorna o estado atual so sensor, ou seja, se detectou ou não algo.
  bool ReadSensor();  //Retorna se o sensor detectou ou nada algo.
};

/*================|| Funções ||====================*/
//Procedimentos

//Inicializar sistema do FreeRTOS do robô
void initFreeRTOS();

//Procedimento para inicializar pinos
void initSystem();

//Função para iniciar código MCPWM
void initMCPWM();


#endif
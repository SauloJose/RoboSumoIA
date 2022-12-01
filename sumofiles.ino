#include "sumoInfo.h"

//Implementação das funções
//Criador: @Sauloj2022.1 (instagram)
/*==================================|| DEFINIÇÃO DAS FUNÇÕES ||==============================*/

//Função para iniciar robô
void initSystem() {

  //Inicia comunicação Serial 0 e 1
  Serial.begin(115200);
  Serial.println("[BMO]: Iniciando configuração dos pinos...");

  Serial.print("[BMO]: Distância de ataque configurada: ");
  Serial.print(_DATK);
  Serial.println(" cm");
}


void initMCPWM() {
  //Struct para configurações do pwm
  Serial.println("[MCPWM]: Iniciando configuração do MCPWM...");
  mcpwm_config_t pwm_config;

  //Configurando informações do pwm
  pwm_config.frequency = 1000;                 //Frequência de 500 hz -> Utiliza o percentual
  pwm_config.cmpr_a = 0;                       //Ciclo de trabalho do PWMxA = 0
  pwm_config.cmpr_b = 0;                       //Ciclo de Trabalho do PWMxB = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;  //Para um controle de motores assimétrico
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;    //Define ciclo de trabalho em nível alto
  Serial.println("[MCPWM]: Configurações de sinal concluídas. ");

  //Funções para iniciar controle dos motores
  //Motor do lado esquerdo
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _MESQA);  //Inicializo PWM na unidade 0 na saída A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, _MESQB);  //Inicializo PWM na unidade 0 na saída B
  Serial.println("[MCPWM]: Configurações do motor esquerdo concluídas ");

  //motor do lado direito
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, _MDIRA);  //Inicializo PWM na unidade 1 na saida positiva
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, _MDIRB);  //Inicializo o PWM na unidade 2 na saída negativa
  Serial.println("[MCPWM]: Configurações do motor direito concluídas ");

  //Iniciando pwm dos motores
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  //Define PWM0A e PWM0B com as configurações dadas acima
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  //Define PWM0A e PWM0B com as configurações dadas acima
  Serial.println("[MCPWM]: Iniciando configurações MCPWM...\n\n");
  delay(300);
}
/*=====================================|| DEFINIÇÃO DOS MÉTODOS ||=================================================================*/
//MÉTODOS DA CLASSE MOTOR


//Método construtor do motor DC
motorDC::motorDC(uint16_t _pin1, uint16_t _pin2, uint8_t _Speed, mcpwm_unit_t _MCPWM_UNIT, mcpwm_timer_t _MCPWM_TIMER) {  //Configura pinos do motor
  //Atribuindo valores
  pin1 = _pin1;
  pin2 = _pin2;

  constrain(_Speed, 0, 100);
  Speed = _Speed;

  //Atribui valores do controle de motores
  mcpwm_num = _MCPWM_UNIT;
  timer_num = _MCPWM_TIMER;

  //Pinos
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

//Construtor da classe motor DC
motorDC::motorDC() {
  //Esses são os valores por DEFAULT
  pin1 = 10;
  pin2 = 9;
  Speed = 100;
  mcpwm_num = MCPWM_UNIT_0;
  timer_num = MCPWM_TIMER_0;

  //Pinos
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

//configura velocidade de giro do motor
void motorDC::SpeedConfig(uint16_t _spd) {
  constrain(_spd, 0, 100);  //Mantém a velocidade no intervalo de 0 a 255.
  Speed = _spd;
}

//Configura motor para rodar no sentido direto
void motorDC::Forward(float _vel) {
  //A -> PWM B->LOW
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);                    // (Unidade, número do timer, operador (A ou B))
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, _vel);                    //(Unidade, número do timer (A ou B), Cicleo de trabalho)
  mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);  // (UNIDADE, TIMER, NÍVEL DO CICLO DE TRABALHO (ALTO OU BAIXO))
}

//Configura motor para girar no sentido inverso
void motorDC::Backward(float _vel) {
  //A -> LOW; B->PWM
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);                    // (Unidade, número do timer, operador (A ou B))
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, _vel);                    //(Unidade, número do timer (A ou B), Cicleo de trabalho)
  mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  // (UNIDADE, TIMER, NÍVEL DO CICLO DE TRABALHO (ALTO OU BAIXO))
}

//Para a rotação do motor
void motorDC::Stop() {
  //A,B -> LOW
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);  //Desliga o sinal do MCPWM no Operador A
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);  //Desliga o sinal do MCPWM no Operador B
}

//Gira o motor em tantos graus (servos motores bons!)
void motorDC::turnM(uint16_t spd, uint16_t angle) {
  //Falta programar essa questão do giro.
}


//*MÉTODOS DA CLASSE MotorSystem*//

//Construtor para Cadastrar motores para controlar o carro
MotorSystem::MotorSystem(motorDC ME, motorDC MD, float spd) {
  //Salvando endereços
  _ME = ME;
  _MD = MD;

  //Configurando velocidade
  constrain(_spd, 0, 100);
  _spd = spd;
}

//Andar para frente
void MotorSystem::ForwardCar(float _vel, uint16_t t) {  //Andar para frente por t segundos
  _ME.Forward(_vel);
  _MD.Forward(_vel);
  vTaskDelay(pdMS_TO_TICKS(t));
}
void MotorSystem::ForwardCar(float _vel) {
  _ME.Forward(_vel);
  _MD.Forward(_vel);
}

//Dar ré
void MotorSystem::BackwardCar(float _vel, uint16_t t) {  //Ré por t segundos
  _ME.Backward(_vel);
  _MD.Backward(_vel);
  vTaskDelay(pdMS_TO_TICKS(t));
}
void MotorSystem::BackwardCar(float _vel) {
  _ME.Backward(_vel);
  _MD.Backward(_vel);
}

//Parar o robô
void MotorSystem::StopCar(uint16_t t) {
  _ME.Stop();
  _MD.Stop();
  vTaskDelay(pdMS_TO_TICKS(t));
}
void MotorSystem::StopCar() {
  _ME.Stop();
  _MD.Stop();
}

//Andar para a direita
void MotorSystem::RightCar(float _vel, uint16_t t) {
  _ME.Stop();
  _MD.Forward(_vel);
  vTaskDelay(pdMS_TO_TICKS(t));
}
void MotorSystem::RightCar(float _vel) {
  _ME.Stop();
  _MD.Forward(_vel);
}

//Andar para a esquerda
void MotorSystem::LeftCar(float _vel, uint16_t t) {
  _ME.Forward(_vel);
  _MD.Stop();
  vTaskDelay(pdMS_TO_TICKS(t));
}
void MotorSystem::LeftCar(float _vel) {
  _ME.Forward(_vel);
  _MD.Stop();
}

//Girar o robô x graus
void MotorSystem::turnR(float _vel, uint16_t _angle) {
  //Falta programar
}

//Método que inicia rotina de TrackBack de um carro inimigo.
void MotorSystem::TrackBack(float _vel, uint8_t _sideOfturn) {
  //Aqui o robô vai estar rodando até encontrar alguém.
  if (_sideOfturn == RST) {  //Gira para a direita
    _ME.Forward(_spd);
    _MD.Backward(_spd);
  } else if (_sideOfturn == LST) {  //Gira para a esquerda
    _ME.Backward(_spd);
    _MD.Forward(_spd);
  } else {  //Se for enviado um valor diferente desses dois, considera como rotação para a direita
    _ME.Forward(_spd);
    _MD.Backward(_spd);
  }
}

void MotorSystem::SpeedConfig(float _spd) {
  _ME.SpeedConfig(_spd);
  _MD.SpeedConfig(_spd);
}

//Método para realizar rotina de teste dos motores
void MotorSystem::TestRoutine(float _vel, uint16_t t) {
  //Testando sentido de movimento
  ForwardCar(_vel, t);  //Para frente
  StopCar(100);
  BackwardCar(_vel, t);  //Para trás
  StopCar(100);
  RightCar(_vel, t);  //Para a direita
  StopCar(100);
  LeftCar(_vel, t);  //Para a esquerda.
  //Testanto controle de velocidade
  for (int it = 0; it <= 100; it = it + 5) {
    ForwardCar(_vel);
    SpeedConfig(it);
    vTaskDelay(pdMS_TO_TICKS(100));  //0.1 s cada variação de velocidade
  }
}

//Métodos da Classe ultrassônica

//Construtor
ultrassonic::ultrassonic(uint8_t pinTRIG = 32, uint8_t pinECHO = 35) {
  //Modificando valor da variável interna
  _pinTrig = pinTRIG;
  _pinEcho = pinECHO;

  //Configurando a função dos pinos
  pinMode(_pinTrig, OUTPUT);
  pinMode(_pinEcho, INPUT);
  //Por default, TRIG=12 e ECHO=13
}

//Retorna a distância detectada pelo sensor ultrasônico.
float ultrassonic::captureDistance() {
  //Capturando tempo de emissão do sinal
  // o bug está aqui
  digitalWrite(_pinTrig, LOW);
  vTaskDelay(pdMS_TO_TICKS(3));
  digitalWrite(_pinTrig, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  digitalWrite(_pinTrig, LOW);

  float _timeMax;  
  _timeD = pulseIn(_pinEcho, HIGH );  //Tempo de ida e vinda

  _timeD = _timeD / 2.0;  //Calculando a distância.

  _distance = _timeD * 0.0334;  //Valor da distância em cm.

  //OBS: _timeD e distance são variáveis internas a classe,
  //Portanto, podem ser vistar após a execução desse método.
  //Zerando o tempo para que não ocorra erro na próxima execução

  return _distance;
}


//MÉTODOS DA CLASSE SENSORIV (InfraVermelho)

//Construtor - configura o pino assocaido a esse sensor.
sensorIV::sensorIV(uint8_t pinDG) {
  _pinSI = pinDG;
  pinMode(_pinSI, INPUT);  //Configura o pino como entrada de dados.
}

//Método para descobir se o sensor foi ou não acionado
bool sensorIV::ReadSensor() {
  if (_stateFunc == _DigFunc) {
    //Funcionamento digital
    _state = (HIGH == digitalRead(_pinSI));
    return _state;
  } else if (_stateFunc == _AlogFunc) {
    //Funcionamento analógico.
    _state = (analogRead(_pinSI) > _DETCOR);
    return _state;
  } else {
    //Caso de default para detecção
    _stateFunc = _DigFunc;                      //Por definição, transforma em entrada digital e então faz a leitura
    _state = (HIGH == digitalRead(_pinSI));  // Retorna LOW quando detecta algo
    return _state;
  }

  //Analisando pelo analógico


  //O módulo utilizado retorna LOW quando encontra um objeto, portanto
  //Caso o objeto esteja em Low,ele irá retornar esse valor.
}
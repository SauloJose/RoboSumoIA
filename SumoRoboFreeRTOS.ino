/********************************************************************************************************
   < PROGRAMA PRINCIPAL DO PROJETO DE ROBÔ SUMÔ - IFAL PALMEIRA - GRUPO DE ROBÓTICA - VERSÃO 2.0 >
   Instituto Federal de Alagoas (IFAL) - Campus Palmeira dos Índios.
   <Equipe>: Alan, Italo, Saulo, Wellington.
   <Objetivo>: O robô em questão irá batalhar em um ringue de batalha de robô sumo (classificação 2.5kg)
   no qual deverá agir e batalhar de forma autônoma, sem ajuda de terceiros.
   <Ultima modificação>: 03/10/2022 
*********************************************************************************************************/

/****************************************************************************************************
  |     NOME       |     CORE       |    PRIORIDADE     |                 DESCRIÇÃO                 |
  |----------------|----------------|-------------------|-------------------------------------------|
  |-vTaskBrain     |      01        |         03        |    Tem como objetivo analisar os dados    |
  |                |                |                   |  Colhidos pela tarefa sensor e então tomar|
  |                |                |                   |  decisões de ataque e defesa.             |
  |----------------|----------------|-------------------|-------------------------------------------|
  |-vTaskSensor    |      00        |         02        |  Tarefa responsável por analisar o ambi-  |
  |                |                |                   |ente e enviar as informações para o cerebro|
  |----------------|----------------|-------------------|-------------------------------------------|
  |-vTaskOnOFF     |      01        |         02        |  Responsável por ligar ou desligar o robô |
  |----------------|----------------|-------------------|-------------------------------------------|
  |-vTaskMotor     |      00        |         04        |  Responsável por realizar os movimentos   |
  |                |                |                   |                                           |
****************************************************************************************************/
/*
    BUGS (DEVO CORRIGIR)
    (x) Adaptar motor pwm no código
    (x) Testar código no robô
    (x) Bug no sensor ultrassônico
*/

//Incluindo bibliotecas
//@Saulo: Para ESP32, as bibliotecas do freeRTOS são acessadas de outra forma

// #include <Arduino_FreeRTOS.h>
// #include "queue.h"

//Driver de PWM para controle de motores
#include "sumoInfo.h"

//Gerando filas

//Fila responsável por enviar a tarefa de movimento qual deve ser a manobra realizada pelo robô
QueueHandle_t xFilaMov;

//Fila responsável por enviar os dados dos sensores infravermelhos do robô
QueueHandle_t xFilaPosition;

//Fila que retorna a distância captada pelo sensor ultrassônico
QueueHandle_t xFilaDistance;

//Gerando os Handles (Endereços) das tarefas
TaskHandle_t vTaskBrainHandle;
TaskHandle_t vTaskSensorHandle;
TaskHandle_t vTaskOnOFFhandle;
TaskHandle_t vTaskMotorHandle;

//Gerando protótipo das tarefas;

//Tarefa responsável por tomar as decisões de acordo com os dados dos sensores
void vTaskBrain(void* pvParamaters);

//Tarefa responsável por fazer leitura dos sensores
void vTaskSensor(void* pvParamaters);

//Tarefa responsável por ligar e desligar o robÔ
void vTaskOnOFF(void* pvParamaters);

//Tarefa responsável por controlar os motores
void vTaskMotor(void* pvParamaters);

//Variável responsável por ligar ou desligar o robô
volatile bool onMotorSystem = true;  //Variável volatile
                                     //true -> LIGADO
                                     //false -> DESLIGADO
/*======================================================================================================*/
//Função setup
void setup() {
  //Inicializando configurações iniciais do robô
  initSystem();  //Inicializa as configurações dos pinos do robô
  initMCPWM();  //Iniciando as configurações MCPWM;
  initFreeRTOS();  //Inicializando o sistema operacional (tarefas e filas necessárias)
}

//A função loop é inútil, mas pode ser utilizada se você quiser...
void loop() {
  /*bool pinOH;
    pinMode(pinOH,INPUT);
    pinOH= (HIGH == digitalRead(SD1));
    Serial.print("[LOOP]: ");
    Serial.println(pinOH);
    vTaskDelay(1000);*/
  vTaskDelete(NULL);

  /*sensorIV Sir1(SD1);
  Serial.println(Sir1.ReadSensor());
  vTaskDelay(1000);*/
}

/*=======================================|| Tarefas ||===================================================*/
//Tarefa cérebro . Responsável por computador os dados dos sensores
void vTaskBrain(void* pvParamaters) {
  (void*)pvParamaters;

  //Variáveis para capturar os valores e tomar decisões.
  SensorIR sensorValues;  //Irá recuperar os valores dos sensores
  float distUS = 0.0;

  //Variável de tomada de decisão (Função + Velocidade)
  OrderMotor movDecision;  //Irá enviar para as tarefas do motor

  //A tarefa do cérebro deve receber os dados dos sensores e decidir o que fazer.
  for (;;) {
    if (xQueueReceive(xFilaPosition, &sensorValues, portMAX_DELAY) == pdTRUE && xQueueReceive(xFilaDistance, &distUS, portMAX_DELAY) == pdTRUE) {
      //Se ele recebe informações dos sensores, ele computa o que deve fazer, caso não receba, ele apenas espera chegar algo.

      movDecision.func = Forward;  //Iniciar uma rotina de testes
      movDecision.vel = 100;
      Serial.println("[CEREBRO]: Iniciando teste de rotina");
      Serial.print("[CEREBRO]:Dados recebidos. Distancia detectada: ");
      Serial.print(distUS);
      Serial.println(" cm");
      Serial.print("[CEREBRO]: Presença de Objeto:");
      if (sensorValues.State[0] == true) {
        Serial.print(" SIM\n");
      } else {
        Serial.print(" NÃO\n\n");
      }

      //Toma a decisão e envia para a tarefa do sistema motor do robô.
      xQueueSend(xFilaMov, &movDecision, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//Tarefa sensor . Responsável por pegar os dados dos sensores e enviar para o cérebro
void vTaskSensor(void* pvParamaters) {
  (void*)pvParamaters;  //Parâmetro de inicialização da task

  //Criando objetos para manipulação dos dados

  //Sensores infravermelhos (Classes) instâncias
  sensorIV SIV1(SD1);  //Frontal-Esquerdo
    //SIV2(SD2),         //Frontal-Direito
    //SIV3(SD3),         //Traseiro-esquerdo
    //SIV4(SD4);         //Traseiro-Direito

  //sensor ultrassônico
  ultrassonic US(TRIG, ECHO);

  //Estrutura de dados para capturar valores
  SensorIR sensorValues;  //Valores dos sensores infravermelhos (Estrutura)
  float distance = 5.0;   //Sensor de distância em cm.

  for (;;) {
    //Capturando valores dos sensores infravermelhos para deterctar posição do robô
    sensorValues.State[0] = SIV1.ReadSensor();  //Capitura a leitura do primeiro sensor IR
    //sensorValues.State[1] = SIV2.ReadSensor();  //Capitura a leitura do segundo sensor IR
    //sensorValues.State[2] = SIV3.ReadSensor();  //Capitura a leitura do terceiro sensor IR
    //sensorValues.State[3] = SIV4.ReadSensor();  //Capitura a leitura do quarto sensor IR

    //Capturando dado do sensor infravermelho frontal (em cm);
    distance = US.captureDistance();
    Serial.print("[SENSOR]: Distancia: ");
    Serial.print(distance);
    Serial.print(" cm");
    Serial.print("\n[SENSOR]: Presença de Objeto:");
    if (sensorValues.State[0] == true) {
      Serial.print(" SIM\n");
    } else {
      Serial.print(" NÃO\n");
    }


    //Enviando dado para as respectivas filas.
    xQueueSend(xFilaPosition, &sensorValues, portMAX_DELAY);  //Fila dos sensores de posição
    xQueueSend(xFilaDistance, &distance, portMAX_DELAY);      //Fila para o valor da distância capturada pelo sensor.
    Serial.println("[SENSOR]: Dados enviados..\n\n");

    //Delay para releitura dos dados.
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

//Tarefa liga/desliga . Vai ligar, ou desligar, o robô
void vTaskOnOFF(void* pvParamaters) {
  (void*)pvParamaters;

  for (;;) {
    //Algo...
    vTaskDelay(300);
  }
}

//Tarefa motor . Irá controlar o robô, fazendo ele andar e realizar as manobras necessárias.
void vTaskMotor(void* pvParamaters) {
  (void*)pvParamaters;

  //Cria o objeto do motor para manipular os movimentos dele.
  //Configura inicialmente os motores)->Alocados estáticamente
  motorDC MotorLeft(_MESQA, _MESQB, 100.0, MCPWM_UNIT_0, MCPWM_TIMER_0),  //Motor da esquerda
    MotorRight(_MDIRA, _MDIRB, 100.0, MCPWM_UNIT_0, MCPWM_TIMER_1);       //Motor da direita

  //Criando o sistema de controle de movimento BMO.
  MotorSystem BMO(MotorLeft, MotorRight, 100.0);

  //Variável de controle do movimento. (Recupera velocidade, E o movimento)
  OrderMotor movState;

  //Configura o robô para que a tarefa trabalhe com ele
  for (;;) {
    if (xQueueReceive(xFilaMov, &movState, portMAX_DELAY) == pdTRUE) {
      Serial.println("\n[BMO]:Ordens recebidas.");  //Caso tenha chegado uma informação do cérebro, ela executa algo.
      switch (movState.func) {
        case Forward:  //Se move para frente
          Serial.println("[BMO]: Frente");
          BMO.ForwardCar(movState.vel);
          break;
        case Backward:  //Se move para trás
          Serial.println("[BMO]: Ré");
          BMO.BackwardCar(movState.vel);
          break;
        case Left:  //Se move para esquerda
          Serial.println("[BMO]: esquerda");
          BMO.LeftCar(movState.vel);
          break;
        case Right:  //Se move para direita
          Serial.println("[BMO]: direita");
          BMO.RightCar(movState.vel);
          break;
        case TrackL:  //Gira em um eixo fixo para a direita (sentido anti-horário);
          Serial.println("[BMO]: virar para esquerda");
          BMO.TrackBack(movState.vel, RST);
          break;
        case TrackR:  //Gira em um eixo fixo para a Esquerda(Sentido horário);
          Serial.println("[BMO]: virar para a direita");
          BMO.TrackBack(movState.vel, LST);
          break;
        case Test:
          Serial.println("[BMO]: rotina de testes");
          BMO.TestRoutine(movState.vel, 3000);
          break;
        default:  //Em qualquer caso diferente dos citados, ele apenas fica parado (Código de erro vindo do cérebro).
          Serial.println("[BMO]: Ficar parado.");
          BMO.StopCar();
          break;
      }
    }
    Serial.println("[BMO]: Tarefa realizada com sucesso.\n\n");
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

/*============================|| DEFINIÇÃO DA FUNÇÃO DE INICIALIZAÇÃO DO FREERTOS||==============================*/
//Obs: Precisei colocar esses dados aqui, para que as tarefas rodem normalmente.

//Procedimento para iniciar o FreeRTOS com as tarefas
void initFreeRTOS() {
  Serial.println("[FreeRTOS]: Iniciando FreeRTOS.");
  BaseType_t returnSensor, returnBrain, returnMotor, returnOnOFF;

  //Gerando filas necessárias.
  //Fila para movimentação do robÔ
  xFilaMov = xQueueCreate(
    1,                    //Quantidade de comandos enviados;
    sizeof(OrderMotor));  //Tamanho do comando enviado;

  //Fila para receber dados dos sensores infravermelhos.
  xFilaPosition = xQueueCreate(
    3,                  //Quantidade de dados enviados
    sizeof(SensorIR));  //Tamanho de cada dado

  //Fila para enviar dados do sensor de distância
  xFilaDistance = xQueueCreate(
    3,
    sizeof(float));

  //Gerando então as tasks;
  if (xFilaMov != NULL && xFilaDistance != NULL && xFilaPosition != NULL) {

    Serial.println("[FreeRTOS]: Filas criadas com sucesso");

    returnSensor = xTaskCreatePinnedToCore(
      vTaskSensor  //Tarefa que irá chamar
      ,
      "TaskSensor"  //Nome do identificador para debug
      ,
      configMINIMAL_STACK_SIZE + 1024  //Memória reservada para essa tarefa
      ,
      NULL  //Parâmetro de inicialização da tarefa;
      ,
      2  //Prioridade da tarefa
      ,
      &vTaskSensorHandle, 0);  //Objeto identificador para que o FreeRTOS o reconheça

    if (returnSensor != pdTRUE) {
      Serial.println("[FreeRTOS]: Não foi possível gerar a tarefa do sensor.");
      while (1)
        ;
    }
    Serial.println("[FreeRTOS]: Tarefa do sensor criada...");

    returnBrain = xTaskCreatePinnedToCore(
      vTaskBrain  //Tarefa que irá ser executada
      ,
      "TaskCerebro"  //Nome para debug
      ,
      configMINIMAL_STACK_SIZE + 1024  //Memória reservada
      ,
      NULL  //Parâmetro de inicialização da tarefa
      ,
      4  //Prioridade da tarefa
      ,
      &vTaskBrainHandle, 1);  //Objeto identificador (Handle)

    if (returnBrain != pdTRUE) {
      Serial.println("[FreeRTOS]: Não foi possível gerar a tarefa BRAIN.");
      while (1)
        ;
    }
    Serial.println("[FreeRTOS]: Tarefa do Cérebro criada...");
    returnMotor = xTaskCreatePinnedToCore(
      vTaskMotor  //Tarefa que será executada
      ,
      "TaskMotor"  //Nome para debug
      ,
      configMINIMAL_STACK_SIZE + 1024  //Memória reservada
      ,
      NULL  //Parâmetro de inicialização da tarefa
      ,
      3  //Prioridad da tarefa
      ,
      &vTaskMotorHandle, 0);  //Objeto para manipular a tarefa

    if (returnMotor != pdTRUE) {
      Serial.println("[FreeRTOS]: Não foi possível gerar a tarefa do motor.");
      while (1)
        ;
    }
    Serial.println("[FreeRTOS]: Tarefa do motor criada...");
    returnOnOFF = xTaskCreatePinnedToCore(
      vTaskOnOFF  //Tarefa que será executada
      ,
      "TaskOnOFF"  //Nome para debug
      ,
      configMINIMAL_STACK_SIZE + 1024  //Memória reserva
      ,
      NULL  //Parâmetro de inicialização da tarefa
      ,
      5  //Priodiade da tarefa
      ,
      &vTaskOnOFFhandle, 1);  //Objeto para manipular a tarefa
    if (returnOnOFF != pdTRUE) {
      Serial.println("[FreeRTOS]:Não foi possível gerar a tarefa do motor.");
      while (1)
        ;
    }
    Serial.println("[FreeRTOS]: Tarefa do OnOFF criada...");
    Serial.println("[FreeRTOS]: FreeRTOS configurado com sucesso.\n\n");
  } else {
    Serial.println("[FreeRTOS]: Não foi possível gerar nenhuma fila, portanto, o sistema está com erro...");
    while (1)
      ;
  }
}
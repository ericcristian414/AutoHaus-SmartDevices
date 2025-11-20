#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Espalexa.h>
#include <WiFiClientSecure.h>

#include <L298NX2.h>
#include <L298N.h>

const char* mqttServer = "e637335d485a428f98d9fb45c66b6923.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "Erickkk";
const char* mqttPassword = "96488941Ab";

const char* client_id_base = "ESP32_ToldoJanela_Erick";
char client_id[50];

const char* topic_base = "casa/erick/toldo_janela";
char topic_toldo_cmd[100];
char topic_toldo_estado[100];
char topic_toldo_velocidade_set[100];
char topic_toldo_velocidade_reset[100];
char topic_janela_cmd[100];
char topic_janela_estado[100];
char topic_janela_velocidade_set[100];
char topic_janela_velocidade_reset[100];
char topic_sensor_config[100];
char topic_sensor_estado[100];
char topic_config_resetwifi[100];
char topic_status_online[100];
char topic_velocidades_geral_estado[100];
char topic_parar_todos_motores[100]; 

char topic_motor_toldo_A_fwd[100];
char topic_motor_toldo_A_bwd[100];
char topic_motor_toldo_B_fwd[100];
char topic_motor_toldo_B_bwd[100];
char topic_motor_janela_fwd[100];
char topic_motor_janela_bwd[100];

#define ENA 13
#define IN1 12
#define IN2 14
#define IN3 27
#define IN4 26
#define ENB 25
#define ENC 18
#define IN5 19
#define IN6 21
#define LED_TOLDO 16
#define LED_JANELA 5
#define BOTAO_MODO 15
#define BOTAO_ABRIR 4
#define BOTAO_FECHAR 32
#define sensorChuvaPin 33
#define DEFAULT_VEL_A_FRENTE 92
#define DEFAULT_VEL_B_FRENTE 200
#define DEFAULT_VEL_A_TRAS 210
#define DEFAULT_VEL_B_TRAS 85
#define DEFAULT_VEL_JANELA_ABRIR 195
#define DEFAULT_VEL_JANELA_FECHAR 190

#define PULSE_DURATION_MS 150 
#define DEFAULT_PULSE_SPEED 200 

L298NX2 motoresToldo(ENA, IN1, IN2, ENB, IN3, IN4);
L298N motorJanela(ENC, IN5, IN6);
WiFiClientSecure espClientSecure;
PubSubClient mqttClient(espClientSecure);
Preferences preferences;
Espalexa espalexa;

bool modoToldo = true;
int posicaoAtualToldo = 0;
int posicaoAtualJanela = 0;
String estadoMovimentoToldo = "PARADO";
String estadoMovimentoJanela = "PARADO";
bool sensorGeralAtivo = true;
bool movimentoAutomaticoToldoAtivo = false;
bool movimentoAutomaticoJanelaAtivo = false;
unsigned long ultimaPublicacaoStatus = 0;
const long intervaloPublicacaoStatus = 5000;

int velocidadeAFrente = DEFAULT_VEL_A_FRENTE;
int velocidadeBFrente = DEFAULT_VEL_B_FRENTE;
int velocidadeATras = DEFAULT_VEL_A_TRAS;
int velocidadeBTras = DEFAULT_VEL_B_TRAS;
int velocidadeJanelaAbrir = DEFAULT_VEL_JANELA_ABRIR;
int velocidadeJanelaFechar = DEFAULT_VEL_JANELA_FECHAR;

unsigned long tempoTotalAbrirToldo = 2750;
unsigned long tempoTotalFecharToldo = 2280;
unsigned long tempoTotalAbrirJanela = 1000;
unsigned long tempoTotalFecharJanela = 1000;

unsigned long tempoInicioPressionadoAbrir = 0;
unsigned long tempoInicioPressionadoFechar = 0;
bool botaoAbrirPressionado = false;
bool botaoFecharPressionado = false;

void setupWiFi();
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void setupMQTT();
void reconnectMQTT();
void publicarEstadoToldo();
void publicarEstadoJanela();
void publicarEstadoSensorGeral();
void publicarTodasVelocidades();
void controleAbrirToldo(int targetPercent);
void controleFecharToldo(int targetPercent);
void controlePararToldo();
void controleAbrirJanela(int targetPercent);
void controleFecharJanela(int targetPercent);
void controlePararJanela();
void verificarBotoes();
bool estaMolhado();
void verificarChuva();
void pararTodosMotores();


void pulsoMotorToldoA(bool forward, int speed);
void pulsoMotorToldoB(bool forward, int speed);
void pulsoMotorJanela(bool forward, int speed);

void controleAbrirToldo(int targetPercent) {
  targetPercent = constrain(targetPercent, 0, 100);
  Serial.printf("[TOLDO] Comando Abrir para: %d%%\n", targetPercent);

  if (targetPercent <= posicaoAtualToldo && targetPercent != 0) {
    if (posicaoAtualToldo == targetPercent) Serial.println("[TOLDO] Já está na posição alvo.");
    else Serial.printf("[TOLDO] Já está em %d%% ou mais aberto.\n", targetPercent);
    estadoMovimentoToldo = "PARADO";
    publicarEstadoToldo();
    return;
  }
  
  controlePararToldo();
  estadoMovimentoToldo = "ABRINDO";
  publicarEstadoToldo();

  int diferencaPercentual = abs(targetPercent - posicaoAtualToldo);
  unsigned long tempoMovimentoCalculado = map(diferencaPercentual, 0, 100, 0, tempoTotalAbrirToldo);

  Serial.printf("[TOLDO] Diferença: %d%%, Tempo Movimento: %lu ms\n", diferencaPercentual, tempoMovimentoCalculado);

  if (tempoMovimentoCalculado > 0) {
    motoresToldo.setSpeedB(255); motoresToldo.backwardB();
    motoresToldo.setSpeedA(140); motoresToldo.forwardB();
    delay(20);
    motoresToldo.setSpeedA(velocidadeATras);
    motoresToldo.setSpeedB(velocidadeBTras);
    motoresToldo.forwardA();
    motoresToldo.backwardB();
    delay(tempoMovimentoCalculado);
  }
  
  motoresToldo.stop();
  posicaoAtualToldo = targetPercent;
  estadoMovimentoToldo = "PARADO";
  Serial.printf("[TOLDO] Posição atualizada: %d%%\n", posicaoAtualToldo);
  publicarEstadoToldo();
}

void controleFecharToldo(int targetPercent) {
  targetPercent = constrain(targetPercent, 0, 100);
  Serial.printf("[TOLDO] Comando Fechar para: %d%%\n", targetPercent);

  if (targetPercent >= posicaoAtualToldo && targetPercent != 100) {
    if (posicaoAtualToldo == targetPercent) Serial.println("[TOLDO] Já está na posição alvo.");
    else Serial.printf("[TOLDO] Já está em %d%% ou mais fechado.\n", targetPercent);
    estadoMovimentoToldo = "PARADO";
    publicarEstadoToldo();
    return;
  }

  controlePararToldo();
  estadoMovimentoToldo = "FECHANDO";
  publicarEstadoToldo();

  int diferencaPercentual = abs(posicaoAtualToldo - targetPercent);
  unsigned long tempoMovimentoCalculado = map(diferencaPercentual, 0, 100, 0, tempoTotalFecharToldo);

  Serial.printf("[TOLDO] Diferença: %d%%, Tempo Movimento: %lu ms\n", diferencaPercentual, tempoMovimentoCalculado);
  
  if (tempoMovimentoCalculado > 0) {
    motoresToldo.setSpeedA(255); motoresToldo.backwardA();
    delay(20);
    motoresToldo.setSpeedA(velocidadeAFrente);
    motoresToldo.setSpeedB(velocidadeBFrente);
    motoresToldo.backwardA();
    motoresToldo.forwardB();
    delay(tempoMovimentoCalculado);
  }

  motoresToldo.stop();
  posicaoAtualToldo = targetPercent;
  estadoMovimentoToldo = "PARADO";
  Serial.printf("[TOLDO] Posição atualizada: %d%%\n", posicaoAtualToldo);
  publicarEstadoToldo();
}

void controlePararToldo() {
  motoresToldo.stop();
  estadoMovimentoToldo = "PARADO";
  Serial.println("[TOLDO] Parado.");
  publicarEstadoToldo();
}

void controleAbrirJanela(int targetPercent) {
  targetPercent = constrain(targetPercent, 0, 100);
  Serial.printf("[JANELA] Comando Abrir para: %d%%\n", targetPercent);

  if (targetPercent <= posicaoAtualJanela && targetPercent != 0) {
    if (posicaoAtualJanela == targetPercent) Serial.println("[JANELA] Já está na posição alvo.");
    else Serial.printf("[JANELA] Já está em %d%% ou mais aberta.\n", targetPercent);
    estadoMovimentoJanela = "PARADO";
    publicarEstadoJanela();
    return;
  }
  
  controlePararJanela();
  estadoMovimentoJanela = "ABRINDO";
  publicarEstadoJanela();

  int diferencaPercentual = abs(targetPercent - posicaoAtualJanela);
  unsigned long tempoMovimentoCalculado = map(diferencaPercentual, 0, 100, 0, tempoTotalAbrirJanela);
  Serial.printf("[JANELA] Diferença: %d%%, Tempo Movimento: %lu ms\n", diferencaPercentual, tempoMovimentoCalculado);

  bool movimentoInterrompido = false; 

  if (tempoMovimentoCalculado > 0) {
    motorJanela.setSpeed(velocidadeJanelaAbrir);
    motorJanela.forward();
    unsigned long startMoveTime = millis();
    while(millis() - startMoveTime < tempoMovimentoCalculado) { 
        delay(50);
    }
  }
  
  motorJanela.stop();
  if (!movimentoInterrompido) {
    posicaoAtualJanela = targetPercent;
  } else {
    Serial.println("[JANELA] Posição não atualizada para 100% devido à interrupção.");
  }
  estadoMovimentoJanela = "PARADO";
  Serial.printf("[JANELA] Posição atualizada para estado: %d%%\n", posicaoAtualJanela);
  publicarEstadoJanela();
}

void controleFecharJanela(int targetPercent) {
  targetPercent = constrain(targetPercent, 0, 100);
  Serial.printf("[JANELA] Comando Fechar para: %d%%\n", targetPercent);

  if (targetPercent >= posicaoAtualJanela && targetPercent != 100) {
      if (posicaoAtualJanela == targetPercent) Serial.println("[JANELA] Já está na posição alvo.");
      else Serial.printf("[JANELA] Já está em %d%% ou mais fechada.\n", targetPercent);
      estadoMovimentoJanela = "PARADO";
      publicarEstadoJanela();
      return;
  }
  
  controlePararJanela();
  estadoMovimentoJanela = "FECHANDO";
  publicarEstadoJanela();

  int diferencaPercentual = abs(posicaoAtualJanela - targetPercent);
  unsigned long tempoMovimentoCalculado = map(diferencaPercentual, 0, 100, 0, tempoTotalFecharJanela);
  Serial.printf("[JANELA] Diferença: %d%%, Tempo Movimento: %lu ms\n", diferencaPercentual, tempoMovimentoCalculado);

  bool movimentoInterrompido = false; 

  if (tempoMovimentoCalculado > 0) {
    motorJanela.setSpeed(velocidadeJanelaFechar);
    motorJanela.backward();
    unsigned long startMoveTime = millis();
    while(millis() - startMoveTime < tempoMovimentoCalculado) {
        delay(50); 
    }
  }
  
  motorJanela.stop();
  posicaoAtualJanela = targetPercent; 
  
  estadoMovimentoJanela = "PARADO";
  Serial.printf("[JANELA] Posição atualizada: %d%%\n", posicaoAtualJanela);
  publicarEstadoJanela();
}

void controlePararJanela() {
  motorJanela.stop();
  estadoMovimentoJanela = "PARADO";
  Serial.println("[JANELA] Parada.");
  publicarEstadoJanela();
}

void pararTodosMotores() {
  Serial.println("Comando para PARAR TODOS OS MOTORES recebido!");
  motoresToldo.stop();
  motorJanela.stop();
  estadoMovimentoToldo = "PARADO";
  estadoMovimentoJanela = "PARADO";
  publicarEstadoToldo();
  publicarEstadoJanela();
  Serial.println("Todos os motores foram PARADOS.");
}

void pulsoMotorToldoA(bool forward, int speed) {
  Serial.printf("[PULSO] Motor Toldo A: %s (Vel: %d) por %d ms\n", forward ? "Frente" : "Trás", speed, PULSE_DURATION_MS);
  if (speed == 0) speed = DEFAULT_PULSE_SPEED; 

  motoresToldo.setSpeedA(constrain(speed, 0, 255));
  if (forward) {
    motoresToldo.forwardA();
  } else {
    motoresToldo.backwardA();
  }
  delay(PULSE_DURATION_MS);
  motoresToldo.stopA();
  Serial.println("[PULSO] Motor Toldo A parado.");
}

void pulsoMotorToldoB(bool forward, int speed) {
  Serial.printf("[PULSO] Motor Toldo B: %s (Vel: %d) por %d ms\n", forward ? "Frente" : "Trás", speed, PULSE_DURATION_MS);
  if (speed == 0) speed = DEFAULT_PULSE_SPEED;

  motoresToldo.setSpeedB(constrain(speed, 0, 255));
  if (forward) {
    motoresToldo.forwardB();
  } else {
    motoresToldo.backwardB();
  }
  delay(PULSE_DURATION_MS);
  motoresToldo.stopB();
  Serial.println("[PULSO] Motor Toldo B parado.");
}

void pulsoMotorJanela(bool forward, int speed) {
  Serial.printf("[PULSO] Motor Janela: %s (Vel: %d) por %d ms\n", forward ? "Abrir" : "Fechar", speed, PULSE_DURATION_MS);
  if (speed == 0) speed = DEFAULT_PULSE_SPEED; 
  motorJanela.setSpeed(constrain(speed, 0, 255));
  if (forward) {
    motorJanela.forward();
  } else {
    motorJanela.backward();
  }
  delay(PULSE_DURATION_MS);
  motorJanela.stop(); 
  Serial.println("[PULSO] Motor Janela parado.");
}

void setupWiFi() {
  preferences.begin("wifi-config", false);
  String savedSSID = preferences.getString("ssid", "");
  String savedPassword = preferences.getString("password", "");

  if (savedSSID != "") {
    Serial.println("Tentando conectar com rede salva: " + savedSSID);
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
      delay(500); Serial.print(".");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFalha ao conectar ou sem rede salva.");
    WiFiManager wifiManager;
    wifiManager.setConfigPortalTimeout(180);
    wifiManager.setConnectTimeout(30);
    String apName = "ESP32-ToldoJanela-Setup";
    if (!wifiManager.autoConnect(apName.c_str())) {
      Serial.println("Falha no WiFiManager. Reiniciando.");
      delay(3000); ESP.restart();
    } else {
      preferences.putString("ssid", WiFi.SSID());
      preferences.putString("password", WiFi.psk());
      Serial.println("Credenciais WiFi salvas!");
    }
  }
  Serial.println("\nConectado à rede WiFi!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  espClientSecure.setInsecure(); 
  Serial.println("WiFiClientSecure configurado (modo inseguro para teste).");

  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(client_id, sizeof(client_id), "%s-%02X%02X%02X", client_id_base, mac[3], mac[4], mac[5]);
  Serial.print("ID do Cliente MQTT: "); Serial.println(client_id);

  snprintf(topic_toldo_cmd, sizeof(topic_toldo_cmd), "%s/toldo/cmd", topic_base);
  snprintf(topic_toldo_estado, sizeof(topic_toldo_estado), "%s/toldo/estado", topic_base);
  snprintf(topic_toldo_velocidade_set, sizeof(topic_toldo_velocidade_set), "%s/toldo/velocidade/set", topic_base);
  snprintf(topic_toldo_velocidade_reset, sizeof(topic_toldo_velocidade_reset), "%s/toldo/velocidade/reset", topic_base);
  snprintf(topic_janela_cmd, sizeof(topic_janela_cmd), "%s/janela/cmd", topic_base);
  snprintf(topic_janela_estado, sizeof(topic_janela_estado), "%s/janela/estado", topic_base);
  snprintf(topic_janela_velocidade_set, sizeof(topic_janela_velocidade_set), "%s/janela/velocidade/set", topic_base);
  snprintf(topic_janela_velocidade_reset, sizeof(topic_janela_velocidade_reset), "%s/janela/velocidade/reset", topic_base);
  snprintf(topic_sensor_config, sizeof(topic_sensor_config), "%s/sensor/config", topic_base);
  snprintf(topic_sensor_estado, sizeof(topic_sensor_estado), "%s/sensor/estado", topic_base);
  snprintf(topic_config_resetwifi, sizeof(topic_config_resetwifi), "%s/config/resetwifi", topic_base);
  snprintf(topic_status_online, sizeof(topic_status_online), "%s/online", topic_base);
  snprintf(topic_velocidades_geral_estado, sizeof(topic_velocidades_geral_estado), "%s/velocidades/estado", topic_base);
  snprintf(topic_parar_todos_motores, sizeof(topic_parar_todos_motores), "%s/parar_todos", topic_base); 

  snprintf(topic_motor_toldo_A_fwd, sizeof(topic_motor_toldo_A_fwd), "%s/motor/toldoA/fwd", topic_base);
  snprintf(topic_motor_toldo_A_bwd, sizeof(topic_motor_toldo_A_bwd), "%s/motor/toldoA/bwd", topic_base);
  snprintf(topic_motor_toldo_B_fwd, sizeof(topic_motor_toldo_B_fwd), "%s/motor/toldoB/fwd", topic_base);
  snprintf(topic_motor_toldo_B_bwd, sizeof(topic_motor_toldo_B_bwd), "%s/motor/toldoB/bwd", topic_base);
  snprintf(topic_motor_janela_fwd, sizeof(topic_motor_janela_fwd), "%s/motor/janela/fwd", topic_base);
  snprintf(topic_motor_janela_bwd, sizeof(topic_motor_janela_bwd), "%s/motor/janela/bwd", topic_base);

  Serial.println("Tópicos MQTT inicializados.");
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem ["); Serial.print(topic); Serial.print("]: ");
  String message;
  for (int i = 0; i < length; i++) { message += (char)payload[i]; }
  Serial.println(message);

  StaticJsonDocument<384> doc;

  if (strcmp(topic, topic_toldo_cmd) == 0) {
    if (message.equalsIgnoreCase("ABRIR")) controleAbrirToldo(100);
    else if (message.equalsIgnoreCase("FECHAR")) controleFecharToldo(0);
    else if (message.equalsIgnoreCase("PARAR")) controlePararToldo();
    else if (message.startsWith("POS:")) {
      int percent = message.substring(4).toInt();
      percent = constrain(percent, 0, 100);
      if (percent > posicaoAtualToldo) controleAbrirToldo(percent);
      else if (percent < posicaoAtualToldo) controleFecharToldo(percent);
      else {
        Serial.printf("[TOLDO] Já está em %d%%.\n", percent);
        estadoMovimentoToldo = "PARADO";
        publicarEstadoToldo();
      }
    }
  }

  else if (strcmp(topic, topic_janela_cmd) == 0) {
    if (message.equalsIgnoreCase("ABRIR")) controleAbrirJanela(100);
    else if (message.equalsIgnoreCase("FECHAR")) controleFecharJanela(0);
    else if (message.equalsIgnoreCase("PARAR")) controlePararJanela();
    else if (message.startsWith("POS:")) {
      int percent = message.substring(4).toInt();
      percent = constrain(percent, 0, 100);
      if (percent > posicaoAtualJanela) controleAbrirJanela(percent);
      else if (percent < posicaoAtualJanela) controleFecharJanela(percent);
      else {
        Serial.printf("[JANELA] Já está em %d%%.\n", percent);
        estadoMovimentoJanela = "PARADO";
        publicarEstadoJanela();
      }
    }
  }

  else if (strcmp(topic, topic_parar_todos_motores) == 0) {
    if (message.equalsIgnoreCase("PARAR")) {
      pararTodosMotores();
    }
  }

  else if (strcmp(topic, topic_motor_toldo_A_fwd) == 0) {
    int speed = message.toInt();
    pulsoMotorToldoA(true, speed);
  }
  else if (strcmp(topic, topic_motor_toldo_A_bwd) == 0) {
    int speed = message.toInt();
    pulsoMotorToldoA(false, speed);
  }
  else if (strcmp(topic, topic_motor_toldo_B_fwd) == 0) {
    int speed = message.toInt();
    pulsoMotorToldoB(true, speed);
  }
  else if (strcmp(topic, topic_motor_toldo_B_bwd) == 0) {
    int speed = message.toInt();
    pulsoMotorToldoB(false, speed);
  }
  else if (strcmp(topic, topic_motor_janela_fwd) == 0) { 
    int speed = message.toInt();
    pulsoMotorJanela(true, speed);
  }
  else if (strcmp(topic, topic_motor_janela_bwd) == 0) { 
    int speed = message.toInt();
    pulsoMotorJanela(false, speed);
  }
  
  else if (strcmp(topic, topic_toldo_velocidade_set) == 0) {
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) { Serial.print(F("JSON toldo vel falhou: ")); Serial.println(error.f_str()); return; }
    velocidadeAFrente = doc["vaf"] | velocidadeAFrente;
    velocidadeBFrente = doc["vbf"] | velocidadeBFrente;
    velocidadeATras = doc["vat"] | velocidadeATras;
    velocidadeBTras = doc["vbt"] | velocidadeBTras;
    Serial.println("Velocidades do toldo atualizadas.");
    publicarTodasVelocidades();
  }
  else if (strcmp(topic, topic_toldo_velocidade_reset) == 0) {
    velocidadeAFrente = DEFAULT_VEL_A_FRENTE; velocidadeBFrente = DEFAULT_VEL_B_FRENTE;
    velocidadeATras = DEFAULT_VEL_A_TRAS; velocidadeBTras = DEFAULT_VEL_B_TRAS;
    Serial.println("Velocidades do toldo resetadas.");
    publicarTodasVelocidades();
  }
  else if (strcmp(topic, topic_janela_velocidade_set) == 0) {
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) { Serial.print(F("JSON janela vel falhou: ")); Serial.println(error.f_str()); return; }
    velocidadeJanelaAbrir = doc["vabrir"] | velocidadeJanelaAbrir;
    velocidadeJanelaFechar = doc["vfechar"] | velocidadeJanelaFechar;
    Serial.println("Velocidades da janela atualizadas.");
    publicarTodasVelocidades();
  }
  else if (strcmp(topic, topic_janela_velocidade_reset) == 0) {
    velocidadeJanelaAbrir = DEFAULT_VEL_JANELA_ABRIR;
    velocidadeJanelaFechar = DEFAULT_VEL_JANELA_FECHAR;
    Serial.println("Velocidades da janela resetadas.");
    publicarTodasVelocidades();
  }
  else if (strcmp(topic, topic_sensor_config) == 0) {
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) { Serial.print(F("JSON sensor cfg falhou: ")); Serial.println(error.f_str()); return; }
    sensorGeralAtivo = doc["sensorAtivo"] | sensorGeralAtivo;
    movimentoAutomaticoToldoAtivo = doc["movToldoAuto"] | movimentoAutomaticoToldoAtivo;
    movimentoAutomaticoJanelaAtivo = doc["movJanelaAuto"] | movimentoAutomaticoJanelaAtivo;
    Serial.println("Configuração do sensor/automação atualizada.");
    publicarEstadoSensorGeral();
  }
  else if (strcmp(topic, topic_config_resetwifi) == 0) {
    Serial.println("Comando para resetar WiFi recebido.");
    preferences.remove("ssid"); preferences.remove("password");
    Serial.println("Credenciais WiFi removidas. Reiniciando em 3s...");
    delay(3000); ESP.restart();
  }
}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callbackMQTT);
}

void reconnectMQTT() {
  int tentativas = 0;
  while (!mqttClient.connected()) {
    Serial.print("Tentando conexão MQTT ("); Serial.print(mqttServer); Serial.print(")... Tentativa: ");
    Serial.println(tentativas + 1);
    
    if (mqttClient.connect(client_id, mqttUser, mqttPassword, topic_status_online, 1, true, "false")) {
      Serial.println("Conectado ao broker MQTT!");
      mqttClient.publish(topic_status_online, (const uint8_t*)"true", 4, true); 

      mqttClient.subscribe(topic_toldo_cmd);
      mqttClient.subscribe(topic_janela_cmd);
      mqttClient.subscribe(topic_toldo_velocidade_set);
      mqttClient.subscribe(topic_toldo_velocidade_reset);
      mqttClient.subscribe(topic_janela_velocidade_set);
      mqttClient.subscribe(topic_janela_velocidade_reset);
      mqttClient.subscribe(topic_sensor_config);
      mqttClient.subscribe(topic_config_resetwifi);
      mqttClient.subscribe(topic_parar_todos_motores);
      Serial.println("Subscrito aos tópicos MQTT.");

      mqttClient.subscribe(topic_motor_toldo_A_fwd);
      mqttClient.subscribe(topic_motor_toldo_A_bwd);
      mqttClient.subscribe(topic_motor_toldo_B_fwd);
      mqttClient.subscribe(topic_motor_toldo_B_bwd);
      mqttClient.subscribe(topic_motor_janela_fwd);
      mqttClient.subscribe(topic_motor_janela_bwd);

      publicarEstadoToldo();
      publicarEstadoJanela();
      publicarEstadoSensorGeral();
      publicarTodasVelocidades();
    } else {
      Serial.print("Falha, rc="); Serial.print(mqttClient.state());
      int estadoCliente = mqttClient.state(); 
      if (estadoCliente == MQTT_CONNECT_BAD_PROTOCOL || estadoCliente == MQTT_CONNECT_BAD_CLIENT_ID) {
          Serial.println("ERRO MQTT: Protocolo/ID Cliente Rejeitado. Verifique configuração do broker/cliente.");
          delay(30000);
      } else if (estadoCliente == MQTT_CONNECT_BAD_CREDENTIALS || estadoCliente == MQTT_CONNECT_UNAUTHORIZED) {
          Serial.println("ERRO MQTT: Credenciais inválidas ou não autorizado. Verifique usuário/senha.");
          delay(30000);
      } else {
        Serial.println(" Tentando novamente em 5 segundos");
        delay(5000);
      }
      tentativas++;
      if (tentativas > 10 && (estadoCliente != MQTT_CONNECT_BAD_CREDENTIALS && estadoCliente != MQTT_CONNECT_UNAUTHORIZED && estadoCliente != MQTT_CONNECT_BAD_PROTOCOL && estadoCliente != MQTT_CONNECT_BAD_CLIENT_ID) ) { 
        Serial.println("Muitas tentativas de reconexão falharam. Verifique a rede/broker. Reiniciando ESP para tentar do zero.");
        ESP.restart(); 
      }
    }
  }
}

void publicarEstadoToldo() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<128> doc;
  doc["posicao"] = posicaoAtualToldo;
  doc["movimento"] = estadoMovimentoToldo;
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(topic_toldo_estado, (const uint8_t*)buffer, n, true);
}

void publicarEstadoJanela() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<128> doc;
  doc["posicao"] = posicaoAtualJanela;
  doc["movimento"] = estadoMovimentoJanela;
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(topic_janela_estado, (const uint8_t*)buffer, n, true);
}

void publicarEstadoSensorGeral() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["chuva"] = estaMolhado() ? "MOLHADO" : "SECO";
  doc["posicaoToldo"] = posicaoAtualToldo;
  doc["posicaoJanela"] = posicaoAtualJanela;
  doc["sensorGeralAtivo"] = sensorGeralAtivo;
  doc["autoMovToldo"] = movimentoAutomaticoToldoAtivo;
  doc["autoMovJanela"] = movimentoAutomaticoJanelaAtivo;
  char buffer[256];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(topic_sensor_estado, (const uint8_t*)buffer, n, true);
}

void publicarTodasVelocidades() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  JsonObject toldoVels = doc.createNestedObject("toldo");
  toldoVels["vaf"] = velocidadeAFrente; toldoVels["vbf"] = velocidadeBFrente;
  toldoVels["vat"] = velocidadeATras; toldoVels["vbt"] = velocidadeBTras;
  JsonObject janelaVels = doc.createNestedObject("janela");
  janelaVels["vabrir"] = velocidadeJanelaAbrir; janelaVels["vfechar"] = velocidadeJanelaFechar;
  char buffer[256];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(topic_velocidades_geral_estado, (const uint8_t*)buffer, n, true);
}

bool estaMolhado() {
  return digitalRead(sensorChuvaPin) == LOW;
}

void verificarBotoes() {
  static unsigned long ultimaLeituraBotao = 0;
  const long debounceBotao = 70;
  
  bool estadoBotaoAbrirAtual = digitalRead(BOTAO_ABRIR) == LOW; 
  bool estadoBotaoFecharAtual = digitalRead(BOTAO_FECHAR) == LOW; 

  if (digitalRead(BOTAO_MODO) == LOW) {
    delay(debounceBotao); 
    if (digitalRead(BOTAO_MODO) == LOW) {
      modoToldo = !modoToldo;
      digitalWrite(LED_TOLDO, modoToldo ? HIGH : LOW);
      digitalWrite(LED_JANELA, modoToldo ? LOW : HIGH);
      controlePararToldo(); controlePararJanela();
      Serial.print("Modo alterado para: "); Serial.println(modoToldo ? "TOLDO" : "JANELA");
      motoresToldo.stop();
      motorJanela.stop();

      botaoAbrirPressionado = false;
      botaoFecharPressionado = false;
      tempoInicioPressionadoAbrir = 0;
      tempoInicioPressionadoFechar = 0;
      while (digitalRead(BOTAO_MODO) == LOW) delay(10);
    }
  }

  if (estadoBotaoAbrirAtual && !botaoAbrirPressionado) { 
    botaoAbrirPressionado = true;
    tempoInicioPressionadoAbrir = millis();
    Serial.println("[BOTAO] Botão ABRIR pressionado. Iniciando movimento gradual.");
    if (modoToldo) {
      motoresToldo.setSpeedB(velocidadeBTras); motoresToldo.backwardB();
      motoresToldo.setSpeedA(velocidadeATras); motoresToldo.forwardA();
      estadoMovimentoToldo = "ABRINDO_GRADUAL";
      publicarEstadoToldo();
    } else { 
      motorJanela.setSpeed(velocidadeJanelaAbrir); motorJanela.forward(); 
      estadoMovimentoJanela = "ABRINDO_GRADUAL";
      publicarEstadoJanela();
    }
  } else if (!estadoBotaoAbrirAtual && botaoAbrirPressionado) {
    botaoAbrirPressionado = false;
    unsigned long tempoPressionado = millis() - tempoInicioPressionadoAbrir;
    Serial.printf("[BOTAO] Botão ABRIR liberado. Tempo pressionado: %lu ms\n", tempoPressionado);

    if (modoToldo) {
      motoresToldo.stop();
      estadoMovimentoToldo = "PARADO";
      int novaPosicaoToldo = posicaoAtualToldo + map(tempoPressionado, 0, tempoTotalAbrirToldo, 0, 100);
      posicaoAtualToldo = constrain(novaPosicaoToldo, 0, 100);
      Serial.printf("[TOLDO] Nova posição: %d%%\n", posicaoAtualToldo);
      publicarEstadoToldo();
    } else {
      motorJanela.stop();
      estadoMovimentoJanela = "PARADO";
      int novaPosicaoJanela = posicaoAtualJanela + map(tempoPressionado, 0, tempoTotalAbrirJanela, 0, 100);
      posicaoAtualJanela = constrain(novaPosicaoJanela, 0, 100); 
      Serial.printf("[JANELA] Nova posição: %d%%\n", posicaoAtualJanela);
      publicarEstadoJanela();
    }
  }

  if (estadoBotaoFecharAtual && !botaoFecharPressionado) {
    botaoFecharPressionado = true;
    tempoInicioPressionadoFechar = millis();
    Serial.println("[BOTAO] Botão FECHAR pressionado. Iniciando movimento gradual.");
    if (modoToldo) {
      motoresToldo.setSpeedA(velocidadeAFrente); motoresToldo.backwardA(); 
      motoresToldo.setSpeedB(velocidadeBFrente); motoresToldo.forwardB();
      estadoMovimentoToldo = "FECHANDO_GRADUAL";
      publicarEstadoToldo();
    } else { 
      motorJanela.setSpeed(velocidadeJanelaFechar); motorJanela.backward();
      estadoMovimentoJanela = "FECHANDO_GRADUAL";
      publicarEstadoJanela();
    }
  } else if (!estadoBotaoFecharAtual && botaoFecharPressionado) { 
    botaoFecharPressionado = false;
    unsigned long tempoPressionado = millis() - tempoInicioPressionadoFechar;
    Serial.printf("[BOTAO] Botão FECHAR liberado. Tempo pressionado: %lu ms\n", tempoPressionado);

    if (modoToldo) {
      motoresToldo.stop();
      estadoMovimentoToldo = "PARADO";
      int novaPosicaoToldo = posicaoAtualToldo - map(tempoPressionado, 0, tempoTotalFecharToldo, 0, 100);
      posicaoAtualToldo = constrain(novaPosicaoToldo, 0, 100);
      Serial.printf("[TOLDO] Nova posição: %d%%\n", posicaoAtualToldo);
      publicarEstadoToldo();
    } else {
      motorJanela.stop();
      estadoMovimentoJanela = "PARADO";
      int novaPosicaoJanela = posicaoAtualJanela - map(tempoPressionado, 0, tempoTotalFecharJanela, 0, 100);
      posicaoAtualJanela = constrain(novaPosicaoJanela, 0, 100); 
      Serial.printf("[JANELA] Nova posição: %d%%\n", posicaoAtualJanela);
      publicarEstadoJanela();
    }
  }
}

void verificarChuva() {
  static bool ultimoEstadoMolhado = false;
  bool molhadoAgora = estaMolhado();
  delay(200);

  if (molhadoAgora != ultimoEstadoMolhado) {
    ultimoEstadoMolhado = molhadoAgora;
    Serial.print("Estado da chuva mudou para: ");
    Serial.println(molhadoAgora ? "MOLHADO" : "SECO");
    publicarEstadoSensorGeral();

    if (movimentoAutomaticoToldoAtivo) {
      if (molhadoAgora) { 
        if (posicaoAtualToldo < 100) { 
          Serial.println("Chuva detectada: Abrindo toldo...");
          controleAbrirToldo(100); 
        }
      } else { 
        if (posicaoAtualToldo > 0) { 
          Serial.println("Chuva parou: Fechando toldo...");
          controleFecharToldo(0); 
        }
      }
    }

    if (movimentoAutomaticoJanelaAtivo) {
      if (molhadoAgora) {
        if (posicaoAtualJanela < 100) { 
          Serial.println("Chuva detectada: Fechando janela...");
          controleAbrirJanela(100); 
        }
      } else {
        if (posicaoAtualJanela > 0) { 
          Serial.println("Chuva parou: Reabrindo janela...");
          controleFecharJanela(0);
        }
      }
    }
  }
}

void alexaControleAbrirToldo(uint8_t brightness) {
  Serial.println("[Alexa] Comando Abrir Toldo");
  controleAbrirToldo( (brightness > 0 || brightness == 255) ? 100 : posicaoAtualToldo );
}
void alexaControleFecharToldo(uint8_t brightness) {
  Serial.println("[Alexa] Comando Fechar Toldo");
  controleFecharToldo(0);
}
void alexaControleAbrirJanela(uint8_t brightness) {
  Serial.println("[Alexa] Comando Abrir Janela");
  controleAbrirJanela( (brightness > 0 || brightness == 255) ? 100 : posicaoAtualJanela );
}
void alexaControleFecharJanela(uint8_t brightness) {
  Serial.println("[Alexa] Comando Fechar Janela");
  controleFecharJanela(0);
}

void setup() {
  Serial.begin(115200);
  motoresToldo.stop();
  motorJanela.stop();

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT); pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  pinMode(BOTAO_MODO, INPUT_PULLUP);
  pinMode(BOTAO_ABRIR, INPUT_PULLUP);
  pinMode(BOTAO_FECHAR, INPUT_PULLUP);
  pinMode(LED_TOLDO, OUTPUT);
  pinMode(LED_JANELA, OUTPUT);
  pinMode(sensorChuvaPin, INPUT);

  digitalWrite(LED_TOLDO, modoToldo ? HIGH : LOW);
  digitalWrite(LED_JANELA, modoToldo ? LOW : HIGH);

  setupWiFi();
  setupMQTT();

  espalexa.addDevice("Open Awning", alexaControleAbrirToldo);
  espalexa.addDevice("Open Window", alexaControleAbrirJanela);
  espalexa.addDevice("Close Awning", alexaControleFecharToldo);
  espalexa.addDevice("Close Window", alexaControleFecharJanela);
  espalexa.begin();

  Serial.println("Setup concluído. ESP32 pronto.");
  delay(1000);
  if(mqttClient.connected()){
    publicarEstadoToldo();
    publicarEstadoJanela();
    publicarEstadoSensorGeral();
    publicarTodasVelocidades();
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexão WiFi perdida. Tentando reconectar (automático)...");
    delay(5000); 
  }

  if (!mqttClient.connected()) {
    reconnectMQTT(); 
  }
  mqttClient.loop(); 
  espalexa.loop(); 
  verificarBotoes(); 

  if (sensorGeralAtivo) {
      verificarChuva(); 
  }
}

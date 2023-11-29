#include <Adafruit_ADS1015.h>
#include <PID_v1.h>

// Pino de entrada analógica para leitura do valor de pH
const int pH_PIN = A0;

Adafruit_ADS1015 ads;  // Objeto para o conversor analógico-digital ADS1015

// Configuração do PID
double setpoint = 7.0; // Valor de pH desejado
double input, output;
double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error, lastError;
double cumError, rateError;

// Definindo os limites de saída do PID
double outMin = 0;
double outMax = 255;

// Pinagem do dispositivo de controle de pH (por exemplo, uma bomba peristáltica para adicionar solução tampão)
// Substitua pelo pino que controla o dispositivo em seu hardware específico
const int CONTROL_PIN = 13;

void setup() {
  Serial.begin(9600);

  ads.begin();

  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);
}

void loop() {
  double currentpH = readpH();

  // Chama a função do PID
  PID_Controller(currentpH);

  // Exibe informações no Serial Monitor
  Serial.print("Valor de pH Atual: ");
  Serial.print(currentpH);
  Serial.print(" | Saída do PID: ");
  Serial.println(output);

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

double readpH() {
  // Lê o valor de pH do sensor Adafruit STEMMA Analog pH
  // Certifique-se de calibrar o sensor adequadamente
  int rawValue = ads.readADC_SingleEnded(pH_PIN);
  double voltage = rawValue * 0.0001875;  // Convertendo valor para volts
  double pHValue = 3.5 * voltage + 3.5;    // Convertendo volts para pH
  return pHValue;
}

void PID_Controller(double currentpH) {
  // Calcula o erro
  error = setpoint - currentpH;

  // Calcula o tempo decorrido
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  // Calcula as componentes PID
  double P = Kp * error;
  cumError += error * elapsedTime;
  double I = Ki * cumError;
  rateError = (error - lastError) / elapsedTime;
  double D = Kd * rateError;

  // Calcula a saída PID
  output = P + I + D;

  // Limita a saída do PID dentro dos limites especificados
  output = constrain(output, outMin, outMax);

  // Aplica a saída do PID para controlar algum dispositivo (por exemplo, uma bomba peristáltica)
  // Neste exemplo, apenas exibimos a saída no Serial Monitor
  // Você deve adaptar esta parte para controlar seu sistema específico
  // (por exemplo, controle de uma bomba peristáltica para adicionar solução tampão)
  digitalWrite(CONTROL_PIN, output > 50); // Liga o dispositivo se a saída for maior que 50

  // Atualiza variáveis para a próxima iteração
  lastError = error;
  previousTime = currentTime;
}

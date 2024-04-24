/*
Universidad Mariano Gálvez de Guatemala
Facultad de Ingeniería, Matematica y Ciencias Fisicas
Escuela de Ingeniería Electrónica
Ingeniería Electrónica
Evaluación General Privada - Área de Electrónica
Evaluadora: Ingenieria Tatiana Vallejo
Alumno: Luis Alejandro López Navas
Carnet: 091-13-12977
Version: 01
Revisiones:  Abril 06, Diseño de la maqueta
             Abril 08, Servo - clasificador
             Abril 10, Servo - tolva
             Abril 10, Sensor de color - transporte
             Abril 21, pruebas finales

*/

//*********************Librerias***************************
#include <Servo.h>                  //Libreria que permite el uso de servomotores(Configura los timmers del mega328p)
#include <Wire.h>                   //Libreria que permite el uso del protocolo TWI (I2C)
#include <Adafruit_TCS34725.h>      //Libreria que permite el uso del sensor de color TCS34725 por medio de TWI
#include <ColorConverterLib.h>      //Libreria que permite la conversión de RGB a HSL
#include <Adafruit_NeoPixel.h>      //Libreria que permite usar el led DS18B20 (NeoPixel)
#include <LiquidCrystal.h>          //Libreria que permite usar la LCD 

//*********************Macros o Mnemonicos***************************
#define servo_clas_pin      6     //Pin 6 del arduino donde esta conectado el servomotor de la rampa clasificadora
#define dirA1_motor A1            //Pin A1 del arduino donde esta conectado el pin IN1 de dirección de giro del motor de la rueda
#define dirA2_motor A0            //Pin A0 del arduino donde esta conectado el pin IN2 de dirección de giro del motor de la rueda
#define sensor_hall 2             //Pin 2 del arduino donde esta conectado el sensor Hall que para la rueda
#define servo_clas_pos_red  13    //Posición de la bandeja para skittles rojos
#define servo_clas_pos_blue 52    //Posición de la bandeja para skittles azules
#define servo_clas_pos_grn  90    //Posición de la bandeja para skittles verdes
#define servo_clas_pos_orng 135   //Posición de la bandeja para skittles naranja
#define servo_clas_pos_yllo 170   //Posición de la bandeja para skittles amarillo 
#define servo_class_pos_emp 90    //Posicion del servo al no tener skittle que clasificar
#define pin_neopixel 4            //Pin 4 del arduino donde esta conectado el neopixel   
#define num_neopixel 1            //Cantidad de neopixel a utilizar
#define pin_rs       3            //Pin 3 del arduino donde esta conectado el pin de rs de la pantalla LCD
#define pin_en       5            //Pin 5 del arduino donde esta conectado el pin de enable de la pantalla LCD
#define pin_d4       7            //Pin 7 del arduino donde esta conectado el pin d4 de la pantalla LCD
#define pin_d5       8            //Pin 8 del arduino donde esta conectado el pin d5 de la pantalla LCD
#define pin_d6       9            //Pin 9 del arduino donde esta conectado el pin d6 de la pantalla LCD
#define pin_d7      10            //Pin 10 del arduino donde esta conectado el pin d7 de la pantalla LCD.

//*********************Enumeradores***************************
//enumeradores para los colores de los skittles
enum colors_skittles
{
  red,
  green,
  blue,
  yellow,
  orange,
  black
};

//*********************Constructores***************************
Servo servo_clas;                                                                                         //Constructor del servo que se encarga de llevar el skittle a la bandeja  correspondiente
Adafruit_TCS34725 sensor_color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);     //Constructor del sensor que se encarga de obtener el color de los skittles
Adafruit_NeoPixel pixel_indicator = Adafruit_NeoPixel(num_neopixel, pin_neopixel, NEO_GRB + NEO_KHZ800);  //Constructor del led digital NeoPixel 
LiquidCrystal lcd(pin_rs, pin_en, pin_d4, pin_d5, pin_d6, pin_d7);                                        //Constructor de la pantalla LCD 

//*********************Prototipos******************************
void  debug_chute(void);            //debug del servomotor que lleva los skitlles a las bandejas
void  anim_setup(void);             //animacion de inicio, a implementar en conjunto con la LCD
float read_color(void);             //lectura del color de los skittles 
void  gpios_config(void);           //configuraciones de entrada y salida
void  setup_sensor(void);           //configuración del sensor de color
void  setup_servos(void);           //configuraicon de los servos
void  setup_lcd(void);              //configuracion de la LCD
void setColorPixel(unsigned int color);   //encender el neopixel del color solicitado

//*********************Variables*******************************
String inf_recibida;                   //variable tipo string - utilizada para la depuración por medio de UART - debug
unsigned int readRed, readBlue, readGreen, clear;   //variables enteras sin signo donde almaceno el valor de los colores leidos
float color_lectura;                  //variable donde guardo el valor leido por por el sensor, este se usa en el loop
int delay_servo = 150;                

//Ciclo de configuraciones
void setup() {
  //Serial.begin(9600);                  //Inicio la comunicacion serial (UART) a 9600bps - debug
  setup_lcd();                         //Inicio 
  setup_gpios();                       //Configuracion de las entradas y salidas de proposito general
  setup_sensor();                      //Inicialización del sensor
  setup_servos();                      //Inicalización del servomotor
  pixel_indicator.begin();             //Inicializo el uso del led Neopixel
  //Serial.println("Integración final 22 de abril");       //-debug
  servo_clas.write(servo_class_pos_emp);    //coloco el servo en la posicion inicial
  setColorPixel(black);                //"Pinto" el ledNeopixel de color negro

}


//Programa infinito - funcionamiento.
void loop() {

run_motor_wheel();    //pongo en marcha el motor

  if(digitalRead(sensor_hall)==LOW)     //Si el sensor detecta el iman
  {   
    stop_motor_wheel();                 //detengo el motor
    delay(2000);                        //pausa de 2 segundos
    //Serial.println("Paro el motor");    //-debug
    //Serial.println("Leo el skittle:");  //-debug
    color_lectura = read_color();       //leo el color del skittle - si no encuentra color por defecto asigna verde
    delay(100);                         //pausa de 100 milisegundos
    //Serial.println(color_lectura);    //-debug  
    //Serial.println(read_color());     //-debug
  }
  //Si el valor de lectura se encuentra en los margenes establecidos se determina el color
  //Color azul

  if((color_lectura>=275.80)&&(color_lectura<=299.30))
  {
    //Serial.println("azul");             //-debug
    setColorPixel(blue);                  //"Pinto" el ledNeopixel de color azul
    color_lectura = 0;                    //reinicio el valor de lectura del color del skittle
    servo_clas.write(servo_clas_pos_blue); //coloco el servomotor en la posición de su bandeja de clasificación
    //delay(delay_servo);
    //servo_clas.write(servo_class_pos_emp);
  }
  else if ((color_lectura>=326.49)&&(color_lectura<=332.80))
 {
  //Serial.println("verde");             //-debug 
  setColorPixel(green);                  //"Pinto" el ledNeopixel de color verde
  color_lectura =0;                      //reinicio el valor de lectura del color del skittle
  servo_clas.write(servo_clas_pos_grn); //coloco el servomotor en la posición de su bandeja de clasificación
  //delay(delay_servo);
  //servo_clas.write(servo_class_pos_emp);  
  }
  else if((color_lectura>=335.59)&&(color_lectura<=343.90))
  {
    //Serial.println("rojo");             //-debug
    setColorPixel(red);                   //"Pinto" el ledNeopixel de color rojo
    color_lectura=0;                      //reinicio el valor de lectura del color del skittle
    servo_clas.write(servo_clas_pos_red); //coloco el servomotor en la posición de su bandeja de clasificación
    //delay(delay_servo);
    //servo_clas.write(servo_class_pos_emp);
  }
  else if((color_lectura>=344.25)&&(color_lectura<=348.49))
  {
    //Serial.println("amarillo");             //-debug
    setColorPixel(yellow);                    //"Pinto" el ledNeopixel de color amarillo
    color_lectura = 0;                        //reinicio el valor de lectura del color del skittle
    servo_clas.write(servo_clas_pos_yllo);    //coloco el servomotor en la posición de su bandeja de clasificación
    //delay(delay_servo);
    //servo_clas.write(servo_class_pos_emp);
  }
  else if((color_lectura>=348.50) &&(color_lectura<=351.20))
  {
    //Serial.println("Naranja");             //-debug
    setColorPixel(orange);                   //"Pinto" el ledNeopixel de color naranja
    color_lectura = 0;                       //reinicio el valor de lectura del color del skittle
    servo_clas.write(servo_clas_pos_orng);   //coloco el servomotor en la posición de su bandeja de clasificación
    //delay(delay_servo);
    //servo_clas.write(servo_class_pos_emp);
  }
}


//*********************Funciones******************************
/*
Funcion usada para pruebas del servomotor que realiza la clasificación - debug
*/
/*
void debug_chute(void)
{
     while(Serial.available() > 0)
    {
      inf_recibida = Serial.readStringUntil('\n');      //Espero una cadena de texto que me indique el color del skittle
      //Serial.print("He recibido:");
      //Serial.println(inf_recibida);
    }

    //Si el valor leido en el string es rojo
    if(inf_recibida == String("rojo"))
    {
      Serial.println("Me muevo hacia la bandeja para skittles rojos");
      inf_recibida = "";
      servo_clas.write(servo_clas_pos_red);
      delay(delay_servo);
      servo_clas.write(servo_class_pos_emp);
      //Serial.println(inf_recibida);
    }

    //Si el valor leido en el string es azul
    if(inf_recibida == String("azul"))
    {
      Serial.println("Me muevo hacia la bandeja para skittles azul");
      inf_recibida = "";
      servo_clas.write(servo_clas_pos_blue);
      delay(delay_servo);
      servo_clas.write(servo_class_pos_emp);
      //Serial.println(inf_recibida);
    }

    //Si el valor leido en el string es verde
    if(inf_recibida == String("verde"))
    {
      Serial.println("Me muevo hacia la bandeja para skittles verde");
      inf_recibida = "";
      servo_clas.write(servo_clas_pos_grn);
      delay(delay_servo);
      servo_clas.write(servo_class_pos_emp);      
      //Serial.println(inf_recibida);
    }

    //Si el valor leido en el string es naranja
    if(inf_recibida == String("naranja"))
    {
      Serial.println("Me muevo hacia la bandeja para skittles naranja");
      inf_recibida = "";
      servo_clas.write(servo_clas_pos_orng);
      delay(delay_servo);
      servo_clas.write(servo_class_pos_emp);
      //Serial.println(inf_recibida);
    }

    //Si el valor leido en el string es amarillo
    if(inf_recibida == String("amarillo"))
    {
      Serial.println("Me muevo hacia la bandeja para skittles amarillo");
      inf_recibida = "";
      servo_clas.write(servo_clas_pos_yllo);
      delay(delay_servo);
      servo_clas.write(servo_class_pos_emp);
      //Serial.println(inf_recibida);
    }
}*/

/*
Funcion usada para realizar la configuración y asignación de la señal de PWM a utilizar en un pin
*/
void setup_servos(void)
{
  servo_clas.attach(servo_clas_pin);  //Configuro el pin3 como salida PWM para poder utilizar el servomotor
}

/*
Funcion usada para realizar la configuración y asignación de la señal de PWM a utilizar en un pin
*/
void setup_sensor(void)
{
  if(sensor_color.begin())
  {
    Serial.println("He encontrado el sensor");
  }
  else
  {
    Serial.println("Sensor no encontrado, revisa conexiones");
    while(1);
  }
}

/*
Funcion usada para relizar la lectura del color por parte del sensor digital TCS34725
*/
float read_color(void)
{
  /*
  delay(70);      //espera de 50ms, para lectura. (Integration time)
  sensor_color.getRGB(&readRead,&readGreen,&readBlue);    //obtencion de los colores

  Serial.println("Color leido por el sensor");
  Serial.print("R:");
  Serial.println(int(readRead));
  Serial.print("G:");
  Serial.println(int(readGreen));
  Serial.print("B:");
  Serial.println(int(readBlue));*/

  delay(50);      //espera de 50 ms como integraciónd de la lectura
  //obtención de los colores en bruto, sin tratamiento
  sensor_color.getRawData(&readRed, &readGreen, &readBlue, &clear);
  //Serial.println("Leyendo colores rgb");
  //Serial.println(readRed);
 // Serial.println(readGreen);
  //Serial.println(readBlue);
 // Serial.println(clear);

  
  //medicion relativa del RGB
  uint32_t sum = clear;
  float r,g,b;
  r=readRed; r/=sum;
  g=readGreen; b/=sum;
  b=readBlue; b/=sum;
 // Serial.println("Valores relativos de las mediciones RGB");
 // Serial.println(r);
 // Serial.println(g);
  //Serial.println(b);
  
  //Escalamiento rgb a bytes
  r *= 256; g *=256; b*=256;

  //Serial.println("Datos escalados");
  //Serial.println(r);
  //Serial.println(g);
 // Serial.println(b);

  //Conversión de RGB A HSV (matiz, saturacion y brillo)
  double hue, saturation, value;
  ColorConverter::RgbToHsv(r,g,b, hue, saturation, value);

  float color_devuelta = hue*360;
  //Serial.print("Matiz del skittle: ");
  //Serial.println((color_devuelta));
  //Serial.print("Saturacion del skittle: ");
  //Serial.println(saturation);
  //Serial.print("Brillo del skittle: ");
  //Serial.println(value);
  return color_devuelta;
}

/*
Funcion usada para la configuraciones de los pines como entradas y salidas
*/
void setup_gpios(void)
{
  pinMode(dirA1_motor, OUTPUT);
  pinMode(dirA2_motor, OUTPUT);
  pinMode(sensor_hall, INPUT);
}

/*
Funcion usada poner en marcha el motor DC de la rueda que posiciona los skittles para su lectura.
*/
void run_motor_wheel(void)
{
  digitalWrite(dirA1_motor, HIGH);
  digitalWrite(dirA2_motor, LOW);
}

/*
Funcion usada detener el motor DC de la rueda que posiciona los skittles para su lectura.
*/
void stop_motor_wheel(void)
{
  digitalWrite(dirA1_motor, HIGH);
  digitalWrite(dirA2_motor, HIGH);
}

/*
Funcion usada para pintar el led NeoPixel del color del skittle leido por el sensor
*/
void setColorPixel(unsigned int color)
{
  switch(color)
  {
    case red:
    {
      pixel_indicator.setPixelColor(0,255,0,0);
      pixel_indicator.show();
      delay(5);
      break;
    }
    case blue:
    {
      pixel_indicator.setPixelColor(0,0,0,255);
      pixel_indicator.show();
      delay(5);
      break;
    }
    case green:
    {
      pixel_indicator.setPixelColor(0,0,255,0);
      pixel_indicator.show();
      delay(5);
      break;
    }
    case yellow:
    {
      pixel_indicator.setPixelColor(0,255,215,0);
      pixel_indicator.show();
      delay(5);
      break;
    }
    case orange:
    {
      pixel_indicator.setPixelColor(0,255,140,0);
      pixel_indicator.show();
      delay(5);
      break;
    }
    case black:
    {
      pixel_indicator.setPixelColor(0,0,0,0);
      pixel_indicator.show();
      delay(5);
      break;
    }
  }
}

/*
Funcion usada para inicializar la LCD y mostrar el mensaje en la pantalla LCD
*/
void setup_lcd(void)
{
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print(" UMG - Privado ");
  lcd.setCursor(0,1);
  lcd.print("Alejandro Lopez");
}
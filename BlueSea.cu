/* Nombre: BrujulaGPSFunciones.ino
   Programa que lee la latitud y la longitud de un GPS usando la libreria TinyGPS+.
   Lee el rumbo al que apunta en el eje X de la brujula HMC5883.
   Utiliza funciones para obtener los datos de los dispositivos y procesarlos,
   haciendo el programa modular.
*/

//ZONA DE LIBRERIAS
#include <TinyGPS++.h>    //Librería para la gestión del GPS (NEO 6)
#include <Wire.h>         //Libreria para leer los datos de la brújula por I2C
#include <MechaQMC5883.h> //Libreria para la gestión de la brújula.

#define INTERVALO 5000   //Intervalo de tiempo de toma de datos del GPS.

TinyGPS gps;

static const double LAT_DEST = 28.110911, LON_DEST = -15.420852;  //Coordenadas destino playa del Poris.
//float flat = 28.096663, flon = -15.711094; //Coordenadas prueba destino.
float flat, flon; //Coordenadas inicio travesía.


unsigned long tempInicio = 0;
unsigned long age;
float difAng;
float rumboDesGPS;      //Rumbo que debe seguir el barco desde donde se encuentra actualmante.
float distanciaDesGPS;  //Distancia que tiene que recorrer desde donde se encuentra actualmente.
float Vel; //Velocidad que tiene conforme la distancia medida entre lecturas en nudos
bool newData;

//Datos para cálcular el rumbo que se ha hecho.
float latInicial, lonInicial; //Posición inicial del barco.
float rumboHecho;             //Rumbo que ha hecho el barco.
float distanciaHecha;         //Distancia que ha recorrido el barco. Solo es información.
//Discrimina el cálculo de la primera posición del GPS.
bool primeraPos = false, calculoRumbo = false;

//Datos para el movimiento del timón (servo).

float Velocidad ()
{
  Vel = ((distanciaHecha) / 5000) / 0.5144;
  Serial.print("Velocidad en nudos: ");
  Serial.println(Vel);
}

float anguloServo, ajuste;

float CalculaRumboHecho (float latIni, float lonIni, float lat, float lon)
{
  float rumbo;
  Serial.println("Datos rumbo hecho.");
  Serial.print("Posición inicial--> ");
  Serial.print ("latInicial "); Serial.print (latIni, 6);
  Serial.print ("\tlonInicial "); Serial.println (lonIni, 6);
  Serial.print("Posición actual --> ");
  Serial.print ("flat "); Serial.print (lat, 6);
  Serial.print ("\tflon "); Serial.println (lon, 6);
  rumbo = gps.course_to (latIni, lonIni, lat, lon);
  return rumbo;
}

float CalculaDistanciaHecha (float latIni, float lonIni, float lat, float lon)
{
  float distancia;
  distancia = gps.distance_between(latIni, lonIni, lat, lon);
  return distancia;
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - start < ms);
  newData = true;
}

void setup()
{
  Wire.begin(9600);
  Serial.begin(9600);   //Comunicación con la placa.
  Serial3.begin(9600);    //Comunicación con GPS.
}

void loop()
{
  // Lectura del GPS. Se obtienen datos cada 5sg.
  smartdelay(5000);

  if (newData)
  {
    //Se almacena la posición anterior.
    if (primeraPos)//La primera vez no se ejecuta.
    {
      latInicial = flat; lonInicial = flon;
      calculoRumbo = true; //Activa el cálculo del rumboHecho y movimiento del timón.
    }

    //Se obtiene la nueva posición.
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print("\tLON=");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //Cálcular el rumbo al destino desde la posición en la que se encuentra el barco.
    Serial.print("Rumbo destino: ");
    rumboDesGPS = gps.course_to(flat, flon, LAT_DEST, LON_DEST);
    Serial.println(rumboDesGPS);
    Serial.print("Distancia destino: ");
    distanciaDesGPS = gps.distance_between(flat, flon, LAT_DEST, LON_DEST) / 1000; // dividir por 1000 para Km.
    Serial.println(distanciaDesGPS);
    primeraPos = true; //Prepara para guardar la segunda posición.
  }

  if (calculoRumbo)
  {
    //Actualizar datos para el cálculo del rumbo seguido.
    rumboHecho = CalculaRumboHecho(latInicial, lonInicial, flat, flon);
    Serial.print ("Rumbo que se ha hecho: "); Serial.println (rumboHecho);
    distanciaHecha = CalculaDistanciaHecha (latInicial, lonInicial, flat, flon);
    Serial.print ("Distancia recorrida: "); Serial.println (distanciaHecha);
    Velocidad();
  }

  //Movimiento del timón según los datos de los rumbos.
  if (rumboDesGPS >= rumboHecho)
  {
    difAng = rumboDesGPS - rumboHecho;
    Serial.print("DifAng: "); Serial.println(difAng);
    if (difAng <= 180)  //Giro a la derecha
    {
      if (difAng > 45)
        anguloServo = 45;
      else
        anguloServo = map (difAng, 0, 45, 90, 45);

      if (anguloServo == 90)
      {
        Serial.print("Recto. Angulo servo: ");
        Serial.print(anguloServo);
      }
      else
      {
        Serial.print("Giro a la derecha.");
        Serial.print("  AnguloServo: "); Serial.println(anguloServo);
      }
    }
    else //Giro a la izquierda
    {
      ajuste = 360 - rumboDesGPS + rumboHecho;  //Correción en la cercania de 0 grados
      Serial.print ("Ajuste: "); Serial.println (ajuste);
      if (ajuste < 45)
        anguloServo = map (ajuste, 0, 45, 90, 135);
      else if (difAng > 45)
        anguloServo = 135;

      if (anguloServo == 90)
      {
        Serial.print("Recto. Angulo servo: ");
        Serial.println(anguloServo);
      }
      else
      {
        Serial.print("Giro a la izquierda.");
        Serial.print("  AnguloServo: "); Serial.println(anguloServo);
      }
    }
  }

  if (rumboDesGPS < rumboHecho)
  {
    difAng = rumboHecho - rumboDesGPS;
    Serial.print("difAng: "); Serial.println(difAng);
    if (difAng <= 180)  //Giro a la izquierda
    {
      if (difAng > 45)
        anguloServo = 135;
      else
        anguloServo = map (difAng, 0, 45, 90, 135);

      if (anguloServo == 90)
      {
        Serial.print("Recto. Angulo servo: ");
        Serial.println(anguloServo);
      }
      else
      {
        Serial.print("Giro a la izquierda.");
        Serial.print("  AnguloServo: "); Serial.println(anguloServo);
      }
    }
    else //Giro a la derecha
    {
      ajuste = 360 - rumboHecho + rumboDesGPS; //Correción en la cercania de 0 grados
      Serial.print ("Ajuste: "); Serial.println (ajuste);
      if (ajuste < 45)
        anguloServo = map (ajuste, 0, 45, 90, 45);
      else if (difAng > 45)
        anguloServo = 45;
      if (anguloServo == 90)
      {
        Serial.print("Recto. Angulo servo: ");
        Serial.println(anguloServo);
      }
      else
      {
        Serial.print("Giro a la derecha.");
        Serial.print("  AnguloServo: "); Serial.println(anguloServo);
      }
    }
  }
  Serial.println(); Serial.println();
}


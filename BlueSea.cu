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

//ZONA DE CONSTANTES
const double DESTINO_LAT = 28.161945;     //Latitud y longitud del destino.
const double DESTINO_LNG = -16.427232;    //Playa del Poris de Abona en Tenerife.

//ZONA DE VARIABLES
MechaQMC5883 brujula;           //Objeto brújula electrónica.
float rumboBrujula;             //Rumbo al que marca la brújula.
float difAngulo;
float ajusteAng;


TinyGPSPlus gps;                //Objeto GPS.
double latitud, longitud;       //Posición del GPS.
double distanciaKm, rumboGPS;   //Distancia y rumbo al destino.
float anguloTimon;
float CorrecionRumbo;
void setup()
{
  Wire.begin();
  brujula.init();

  Serial.begin(9600);         //Comunicación con el dispositivo receptor.
  Serial3.begin(9600);        //Comunicación con el GPS.
}

void loop()
{
  rumboBrujula = RumboBrujula();
  Serial.print("Rumbo brujula: ");
  Serial.println(rumboBrujula);
  Serial.println ();

  ObtenerLatLonGPS (&latitud, &longitud);
  Serial.print("Latitud: "); Serial.print(latitud, 6);
  Serial.print("\tLongitud: "); Serial.println(longitud, 6);

  distanciaKm = ObtenerDistancia(latitud, longitud, DESTINO_LAT, DESTINO_LNG);
  Serial.print("DISTANCIA: "); Serial.print(distanciaKm); Serial.println(" Km");

  rumboGPS = RumboGPS(latitud, longitud, DESTINO_LAT, DESTINO_LNG);
  Serial.print("Rumbo GPS: "); Serial.print(rumboGPS); Serial.println( "º");

  CorrecionRumbo = Correcionrumbo ();
  Serial.println ();
  Serial.print("dif de angulo: ");
  Serial.println(difAngulo);
  Serial.print("Ajuste de angulo: ");
  Serial.println(ajusteAng);
  Serial.print("Angulo del timon:  ");
  Serial.println(anguloTimon);
  Serial.println();
  delay(2000);

  
}

float RumboBrujula()
{ //Función que devuelve le rumbo de la brújula
  int x, y, z;
  int azimuth;

  brujula.read(&x, &y, &z, &azimuth);
  return (float) azimuth;
}

bool ObtenerLatLonGPS (double *lat, double *lon)
{ //Obtiene la pasición (lat y lon) desde una objeto llamado gps de la libreria TinyGPS+)
  bool newData = false;
  //Intenta recibir secuencia durante un segundo
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial3.available())
    {
      if (gps.encode(Serial3.read())) // Nueva secuencia recibida
        newData = true;
    }
  }
  if (newData)
  {
    *lat = gps.location.lat();
    *lon = gps.location.lng();
  }
  return newData;
}

double ObtenerDistancia (double lat, double lon, double destLat, double destLon)
{ //Devuelve la distancia en Km desde el origen (lat y lon) al destino (destLat y destLon)
  double distancia;
  distancia = gps.distanceBetween(lat, lon, destLat, destLon) / 1000.0;
  return (float) distancia;
}

float RumboGPS (double lat, double lon, double destLat, double destLon)
{ //Devuelve el rumbo a seguir desde el origen (lat y lon) al destino (destLat y destLon)
  float rumbo;
  rumbo = gps.courseTo(lat, lon, destLat, destLon);
  return rumbo;
}

float Correcionrumbo ()
{
  if (rumboGPS >= rumboBrujula)
  {
    difAngulo = rumboGPS - rumboBrujula;

    if (difAngulo <= 180)
    { //Giro a la derecha
      if (difAngulo > 45)
        anguloTimon = 45;
      else
        anguloTimon = map(difAngulo, 0, 45, 90, 45);
    }
    if (difAngulo > 180)
    { //Giro a la izquierda.
      ajusteAng = 360 - rumboGPS + rumboBrujula;
      if (ajusteAng > 45)
        anguloTimon = 45;
      else
        anguloTimon = map(ajusteAng, 0, 45, 90, 135);
    }
  }
  if (rumboGPS < rumboBrujula)
  {
    difAngulo =  rumboBrujula - rumboGPS;

    if (difAngulo <= 180)
    { //Giro a la izquierda
      if (difAngulo > 45)
        anguloTimon = 135;
      else
        anguloTimon = map(difAngulo, 0, 45, 90, 135);
    }
    if (difAngulo > 180)
    { //Giro a la derecha.
      ajusteAng = 360 - rumboBrujula + rumboGPS;
      if (ajusteAng > 45)
        anguloTimon = 45;
      else
        anguloTimon = map(ajusteAng, 0, 45, 90, 45);
    }
  }
}

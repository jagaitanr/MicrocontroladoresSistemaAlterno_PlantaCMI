
//Librerías
//#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xAA
};
IPAddress ip(192, 168, 5, 100);
IPAddress subnet(255, 255, 255, 0);

unsigned int localPort = 7124;      // local port to listen on

// buffers for receiving and sending data
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char packetBuffer[100];
char  ReplyBuffer[] = "acknowledged";       // a string to send back
int k=0;
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetServer server(80);


//int CursorVertical = 0;

/* valores solicitados por el programa*/
        char BandaAlimentadoraEntrada ="0";
        char ZarandaEntrada ="0";
        char BandaColectoraEntrada ="0";
        char PesoPruebaBandaAlimentadoraEntrada ="0";
        char BandaAgregados1Entrada ="0";
        char BandaAgregados2Entrada ="0";
        char BandaAgregados3Entrada ="0";
        char BandaAgregados4Entrada ="0";
        char BandaInclinadaRecicladoEntrada ="0";
        char BandaAgregadoRecicladoEntrada ="0";
        char MotorTrituradoRecicladoEntrada ="0";
        char PesoPruebaBandaRecicladoEntrada ="0";
        char CompuertaRecicladoEntrada ="0";
        char MotorBlowerBHEntrada ="0";
        char MotorVannerBHEntrada ="0";
        char SinFinColector1BHEntrada ="0";
        char SinFinColector2BHEntrada ="0";
        char SinFinTransversalBHEntrada ="0";
        char CompresorEntrada ="0";
        char BombaAsfaltoAdelanteEntrada ="0";
        char BombaAsfaltoAtrasEntrada ="0";
        char Extractor1BHEntrada ="0";
        char Extractor2BHEntrada ="0";
        char BlowerQuemadorEntrada ="0";
        char ElevadorEntrada ="0";
        char CompuertaRechazoEntrada ="0";
        char SecadorEntrada ="0";
        char SlingerAdelanteEntrada ="0";
        char SlingerAtrasEntrada ="0";
        char BombaCombustibleEntrada ="0";
        char DamperEntrada ="0";
        char PilotoValvulaEntrada ="0";
        char MainLlamaEntrada ="0";
        char Rotor1BHEntrada ="0";
        char Rotor2BHEntrada ="0";
        char Rotor3BHEntrada ="0";
        char AlimentacionSiloEntrada ="0";
        char ValvulaAceiteEntrada ="0";
        char ValvulaGasEntrada ="0";
        char CompuertaGansoEntrada ="0";
        char R8090ControlLLamaEntrada ="0";
        double PresionCasadeBolsas=0;
        double TemperaturaCasadeBolsas=0;
        double TemperaturaMezcla=0;
        

  

 /*valores entregados por el microcontrolador Esclavo1 correspondiente a lo que lee
  * del estado de los motores,  provenientes de los relevos 110VAC
  */
        char BandaAlimentadoraEstado ="0";
        char ZarandaEstado ="0";
        char BandaColectoraEstado ="0";
        char PesoPruebaBandaAlimentadoraEstado ="0";
        char BandaAgregados1Estado ="0";
        char BandaAgregados2Estado ="0";
        char BandaAgregados3Estado ="0";
        char BandaAgregados4Estado ="0";
        char BandaInclinadaRecicladoEstado ="0";
        char BandaAgregadoRecicladoEstado ="0";
        char MotorTrituradoRecicladoEstado ="0";
        char PesoPruebaBandaRecicladoEstado ="0";
        char CompuertaRecicladoEstado ="0";
        char MotorBlowerBHEstado ="0";
        char MotorVannerBHEstado ="0";
        char SinFinColector1BHEstado ="0";
        char SinFinColector2BHEstado ="0";
        char SinFinTransversalBHEstado ="0";
        char CompresorEstado ="0";
        char BombaAsfaltoAdelanteEstado ="0";
        char BombaAsfaltoAtrasEstado ="0";
        char Extractor1BHEstado ="0";
        char Extractor2BHEstado ="0";
        char BlowerQuemadorEstado ="0";
        char ElevadorEstado ="0";
        char CompuertaRechazoEstado ="0";
        char SecadorEstado ="0";
        char SlingerAdelanteEstado ="0";
        char SlingerAtrasEstado ="0";
        char BombaCombustibleEstado ="0";
        char DamperEstado ="0";
        char PilotoValvulaEstado ="0";
        char MainLlamaEstado ="0";
        char Rotor1BHEstado ="0";
        char Rotor2BHEstado ="0";
        char Rotor3BHEstado ="0";
        char AlimentacionSiloEstado ="0";
        char ValvulaAceiteEstado ="0";
        char ValvulaGasEstado ="0";
        char CompuertaGansoEstado ="0";
        char R8090ControlLLamaEstado ="0";
        int BytesEnBuffer=0;

        int lectura;
        char lectura2;
        char Lectura ="";
        int posDatoLeido=0;
        int pos = 0;
        char DatoLeido[100];
        char Dato_a_enviar[100];
        char DatoAnalogo_a_enviar[100];
        int a;//valor del dato que viene en char y se pasa a entero
        int b=0;
        char str[4];
        char Dato_de_Maestro[100];
        char Dato_de_Maestro_para_Esclavo2[100]; // dato recibido del maestro para esclavo2 operar motores
        

 
int xPin = A1;
int yPin = A0;
int buttonPin = 53;

int xPosition = 0;
int yPosition = 0;
int buttonState = 0;
bool cambiar_dato=false;

const int EnTxPin = 2;// High: Tx y Low Rx
const int EstadoBandaAlimentadora =3;
const int EstadoZaranda =4;
const int EstadoBandaColectora =5;
const int EstadoPesoPruebaBandaAlimentadora =6;
const int EstadoBandaAgregados1 =7;
const int EstadoBandaAgregados2 =8;
const int EstadoBandaAgregados3 =9;
const int EstadoBandaAgregados4 =10;
const int EstadoBandaInclinadaReciclado =11;
const int EstadoBandaAgregadoReciclado =12;
const int EstadoMotorTrituradoReciclado =13;
const int EstadoPesoPruebaBandaReciclado =22;
const int EstadoCompuertaReciclado =23;
const int EstadoMotorBlowerBH =24;
const int EstadoMotorVannerBH =25;
const int EstadoSinFinColector1BH =26;
const int EstadoSinFinColector2BH =27;
const int EstadoSinFinTransversalBH =28;
const int EstadoCompresor =29;
const int EstadoBombaAsfaltoAdelante =30;
const int EstadoBombaAsfaltoAtras =31;
const int EstadoExtractor1BH =32;
const int EstadoExtractor2BH =33;
const int EstadoBlowerQuemador =34;
const int EstadoElevador =35;
const int EstadoCompuertaRechazo =36;
const int EstadoSecador =37;
const int EstadoSlingerAdelante =38;
const int EstadoSlingerAtras =39;
const int EstadoBombaCombustible =40;
const int EstadoDamper =41;
const int EstadoPilotoValvula =42;
const int EstadoMainLlama =43;
const int EstadoRotor1BH =44;
const int EstadoRotor2BH =45;
const int EstadoRotor3BH =46;
const int EstadoAlimentacionSilo =47;
const int EstadoValvulaAceite =48;
const int EstadoCompuertaGanso =49;
const int EstadoValvulaGas =50;
int lecturaInt=0;
   int numero = 5;
  char letra = 'f';
  char c;


void setup() 
{ 
pinMode( EstadoBandaAlimentadora, INPUT_PULLUP);
pinMode( EstadoZaranda, INPUT_PULLUP);
pinMode( EstadoBandaColectora, INPUT_PULLUP);
pinMode( EstadoPesoPruebaBandaAlimentadora, INPUT_PULLUP);
pinMode( EstadoBandaAgregados1, INPUT_PULLUP);
pinMode( EstadoBandaAgregados2, INPUT_PULLUP);
pinMode( EstadoBandaAgregados3, INPUT_PULLUP);
pinMode( EstadoBandaAgregados4, INPUT_PULLUP);
pinMode( EstadoBandaInclinadaReciclado, INPUT_PULLUP);
pinMode( EstadoBandaAgregadoReciclado, INPUT_PULLUP);
pinMode( EstadoMotorTrituradoReciclado, INPUT_PULLUP);
pinMode( EstadoPesoPruebaBandaReciclado, INPUT_PULLUP);
pinMode( EstadoCompuertaReciclado, INPUT_PULLUP);
pinMode( EstadoMotorBlowerBH, INPUT_PULLUP);
pinMode( EstadoMotorVannerBH, INPUT_PULLUP);
pinMode( EstadoSinFinColector1BH, INPUT_PULLUP);
pinMode( EstadoSinFinColector2BH, INPUT_PULLUP);
pinMode( EstadoSinFinTransversalBH, INPUT_PULLUP);
pinMode( EstadoCompresor, INPUT_PULLUP);
pinMode( EstadoBombaAsfaltoAdelante, INPUT_PULLUP);
pinMode( EstadoBombaAsfaltoAtras, INPUT_PULLUP);
pinMode( EstadoExtractor1BH, INPUT_PULLUP);
pinMode( EstadoExtractor2BH, INPUT_PULLUP);
pinMode( EstadoBlowerQuemador, INPUT_PULLUP);
pinMode( EstadoElevador, INPUT_PULLUP);
pinMode( EstadoCompuertaRechazo, INPUT_PULLUP);
pinMode( EstadoSecador, INPUT_PULLUP);
pinMode( EstadoSlingerAdelante, INPUT_PULLUP);
pinMode( EstadoSlingerAtras, INPUT_PULLUP);
pinMode( EstadoBombaCombustible, INPUT_PULLUP);
pinMode( EstadoDamper, INPUT_PULLUP);
pinMode( EstadoPilotoValvula, INPUT_PULLUP);
pinMode( EstadoMainLlama, INPUT_PULLUP);
pinMode( EstadoRotor1BH, INPUT_PULLUP);
pinMode( EstadoRotor2BH, INPUT_PULLUP);
pinMode( EstadoRotor3BH, INPUT_PULLUP);
pinMode( EstadoAlimentacionSilo, INPUT_PULLUP);
pinMode( EstadoValvulaAceite, INPUT_PULLUP); 

pinMode( EstadoValvulaGas, INPUT_PULLUP);
pinMode( EstadoCompuertaGanso, INPUT_PULLUP);

#define SERIAL1_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256
#define SERIAL2_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256
 // start the Ethernet and UDP:
  Ethernet.begin(mac, ip);
  server.begin();
  Udp.begin(localPort);

  Serial.begin(9600);// monitor del arduino en PC


}

 void loop()
  {
   LecturaPuertos();
   Envio_a_Maestro();
   delay (100);
   Solicitud_servidor();
  }

/*
ActualizarLecturadeMaestro, verifica el dato que entro por SerialEven1
si es ME1, entonces asigna Dato_de_Maestro a Dato_de_Maestro_Para_
Esclavo2,  y de una vez pasa a otra subrutina para enviarlo por
SPI antecedido por E1E2 que significa de Esclavo1 a Esclavo2


*//*
  void ActualizarLecturadeMaestro()
  {
    if (Dato_de_Maestro[0]=='M' && Dato_de_Maestro[1]=='E' && Dato_de_Maestro[2]=='1') //dato que será enviado a Esclavo2
    {  
     int a = 0;
     while(a<=40)
    {
      Dato_de_Maestro_para_Esclavo2[a]=Dato_de_Maestro[a+3];
      a++;
    }
   // Envio_a_Esclavo2();  //envío por SPI
    }
     
  }

*/

void LecturaPuertos()
{
  if (digitalRead (EstadoBandaAlimentadora) == LOW)
  BandaAlimentadoraEstado = '1';
  else
  BandaAlimentadoraEstado = '0';
  
  if (digitalRead (EstadoZaranda) == LOW)
  ZarandaEstado = '1';
  else
  ZarandaEstado = '0';  

  
  if (digitalRead (EstadoBandaColectora) == LOW)
  BandaColectoraEstado = '1';
  else
  BandaColectoraEstado = '0';

  if (digitalRead (EstadoPesoPruebaBandaAlimentadora) == LOW)
  PesoPruebaBandaAlimentadoraEstado = '1';
  else
  PesoPruebaBandaAlimentadoraEstado = '0';

  if (digitalRead (EstadoBandaAgregados1) == LOW)
  BandaAgregados1Estado = '1';
  else
  BandaAgregados1Estado = '0';

  if (digitalRead (EstadoBandaAgregados2) == LOW)
  BandaAgregados2Estado = '1';
  else
  BandaAgregados2Estado = '0';

  if (digitalRead (EstadoBandaAgregados3) == LOW)
  BandaAgregados3Estado = '1';
  else
  BandaAgregados3Estado = '0';


  if (digitalRead (EstadoBandaAgregados4) == LOW)
  BandaAgregados4Estado = '1';
  else
  BandaAgregados4Estado = '0';

 if (digitalRead (EstadoBandaInclinadaReciclado) == LOW)
  BandaInclinadaRecicladoEstado = '1';
  else
  BandaInclinadaRecicladoEstado = '0';

  if (digitalRead (EstadoBandaAgregadoReciclado) == LOW)
  BandaAgregadoRecicladoEstado = '1';
  else
  BandaAgregadoRecicladoEstado = '0';

  if (digitalRead (EstadoMotorTrituradoReciclado) == LOW)
  MotorTrituradoRecicladoEstado = '1';
  else
  MotorTrituradoRecicladoEstado = '0';

  if (digitalRead (EstadoPesoPruebaBandaReciclado) == LOW)
  PesoPruebaBandaRecicladoEstado = '1';
  else
  PesoPruebaBandaRecicladoEstado = '0';

  if (digitalRead (EstadoCompuertaReciclado) == LOW)
  CompuertaRecicladoEstado = '1';
  else
  CompuertaRecicladoEstado = '0';

  
  if (digitalRead (EstadoMotorBlowerBH) == LOW)
  MotorBlowerBHEstado = '1';
  else
  MotorBlowerBHEstado = '0';

  if (digitalRead (EstadoMotorVannerBH) == LOW)
  MotorVannerBHEstado = '1';
  else
  MotorVannerBHEstado = '0';
 
  if (digitalRead (EstadoSinFinColector1BH) == LOW)
  SinFinColector1BHEstado = '1';
  else
  SinFinColector1BHEstado = '0';

  if (digitalRead (EstadoSinFinColector2BH) == LOW)
  SinFinColector2BHEstado = '1';
  else
  SinFinColector2BHEstado = '0';

  if (digitalRead (EstadoSinFinTransversalBH) == LOW)
  SinFinTransversalBHEstado = '1';
  else
  SinFinTransversalBHEstado = '0';

  if (digitalRead (EstadoCompresor) == LOW)
  CompresorEstado = '1';
  else
  CompresorEstado = '0';

  if (digitalRead (EstadoBombaAsfaltoAdelante) == LOW)
  BombaAsfaltoAdelanteEstado = '1';
  else
  BombaAsfaltoAdelanteEstado = '0';

  if (digitalRead (EstadoBombaAsfaltoAtras) == LOW)
  BombaAsfaltoAtrasEstado = '1';
  else
  BombaAsfaltoAtrasEstado = '0';

  if (digitalRead (EstadoExtractor1BH) == LOW)
  Extractor1BHEstado = '1';
  else
  Extractor1BHEstado = '0';

  if (digitalRead (EstadoExtractor2BH) == LOW)
  Extractor2BHEstado = '1';
  else
  Extractor2BHEstado = '0';

  if (digitalRead (EstadoBlowerQuemador) == LOW)
  BlowerQuemadorEstado = '1';
  else
  BlowerQuemadorEstado = '0';

  if (digitalRead (EstadoElevador) == LOW)
  ElevadorEstado = '1';
  else
  ElevadorEstado = '0';

  if (digitalRead (EstadoCompuertaRechazo) == LOW)
  CompuertaRechazoEstado = '1';
  else
  CompuertaRechazoEstado = '0';

  if (digitalRead (EstadoSecador) == LOW)
  SecadorEstado = '1';
  else
  SecadorEstado = '0';

  if (digitalRead (EstadoSlingerAdelante) == LOW)
  SlingerAdelanteEstado = '1';
  else
  SlingerAdelanteEstado = '0';

  if (digitalRead (EstadoSlingerAtras) == LOW)
  SlingerAtrasEstado = '1';
  else
  SlingerAtrasEstado = '0';

 if (digitalRead (EstadoBombaCombustible) == LOW)
  BombaCombustibleEstado = '1';
  else
  BombaCombustibleEstado = '0';

  if (digitalRead (EstadoDamper) == LOW)
  DamperEstado = '1';
  else
  DamperEstado = '0';

  if (digitalRead (EstadoPilotoValvula) == LOW)
  PilotoValvulaEstado = '1';
  else
  PilotoValvulaEstado = '0';

  if (digitalRead (EstadoMainLlama) == LOW)
  MainLlamaEstado = '1';
  else
  MainLlamaEstado = '0';

  if (digitalRead (EstadoRotor1BH) == LOW)
  Rotor1BHEstado = '1';
  else
  Rotor1BHEstado = '0';

  if (digitalRead (EstadoRotor2BH) == LOW)
  Rotor2BHEstado = '1';
  else
  Rotor2BHEstado = '0';

  if (digitalRead (EstadoRotor3BH) == LOW)
  Rotor3BHEstado = '1';
  else
  Rotor3BHEstado = '0';

  if (digitalRead (EstadoAlimentacionSilo) == LOW)
  AlimentacionSiloEstado = '1';
  else
  AlimentacionSiloEstado = '0';

  if (digitalRead (EstadoValvulaAceite) == LOW)
  ValvulaAceiteEstado = '1';
  else
  ValvulaAceiteEstado = '0';

  if (digitalRead (EstadoValvulaGas) == LOW)
  ValvulaGasEstado = '1';
  else
  ValvulaGasEstado = '0';

  if (digitalRead (EstadoCompuertaGanso) == LOW)
  CompuertaGansoEstado = '1';
  else
  CompuertaGansoEstado = '0';
  //*/
  }
  
  void Envio_a_Maestro()
      {
        Dato_a_enviar[0]= 'E';
        Dato_a_enviar[1]= '1';
        Dato_a_enviar[2]= 'M';                                                               
        Dato_a_enviar[3]= '1';
        Dato_a_enviar[4]= '2';// envío de señales digitales, estado de motores
        Dato_a_enviar[5] = BandaAlimentadoraEstado;
        Dato_a_enviar[6] = ZarandaEstado;
        Dato_a_enviar[7] = BandaColectoraEstado;
        Dato_a_enviar[8] = PesoPruebaBandaAlimentadoraEstado;
        Dato_a_enviar[9] = BandaAgregados1Estado;
        Dato_a_enviar[10] = BandaAgregados2Estado;
        Dato_a_enviar[11] = BandaAgregados3Estado;
        Dato_a_enviar[12] = BandaAgregados4Estado;
        Dato_a_enviar[13] = BandaInclinadaRecicladoEstado;
        Dato_a_enviar[14] = BandaAgregadoRecicladoEstado;
        Dato_a_enviar[15] = MotorTrituradoRecicladoEstado;
        Dato_a_enviar[16] = PesoPruebaBandaRecicladoEstado;
        Dato_a_enviar[17] = CompuertaRecicladoEstado;
        Dato_a_enviar[18] = MotorBlowerBHEstado;
        Dato_a_enviar[19] = MotorVannerBHEstado;
        Dato_a_enviar[20] = SinFinColector1BHEstado;
        Dato_a_enviar[21] = SinFinColector2BHEstado;
        Dato_a_enviar[22] = SinFinTransversalBHEstado;
        Dato_a_enviar[23] = CompresorEstado;
        Dato_a_enviar[24] = BombaAsfaltoAdelanteEstado;
        Dato_a_enviar[25] = BombaAsfaltoAtrasEstado;
        Dato_a_enviar[26] = Extractor1BHEstado;
        Dato_a_enviar[27] = Extractor2BHEstado;
        Dato_a_enviar[28] = BlowerQuemadorEstado;
        Dato_a_enviar[29] = ElevadorEstado;
        Dato_a_enviar[30] = CompuertaRechazoEstado;
        Dato_a_enviar[31] = SecadorEstado;
        Dato_a_enviar[32] = SlingerAdelanteEstado;
        Dato_a_enviar[33] = SlingerAtrasEstado;
        Dato_a_enviar[34] = BombaCombustibleEstado;
        Dato_a_enviar[35] = DamperEstado;
        Dato_a_enviar[36] = PilotoValvulaEstado;
        Dato_a_enviar[37] = MainLlamaEstado;
        Dato_a_enviar[38] = Rotor1BHEstado;
        Dato_a_enviar[39] = Rotor2BHEstado;
        Dato_a_enviar[40] = Rotor3BHEstado;
        Dato_a_enviar[41] = AlimentacionSiloEstado;
        Dato_a_enviar[42] = ValvulaAceiteEstado;
        Dato_a_enviar[43] = ValvulaGasEstado;
        Dato_a_enviar[44] = CompuertaGansoEstado;
        envio_maestroUDP();
        }
/*
 * No requerimos leer paquetes ya que este microcontrolador solo
 * envia el estado de los motores
 */

   /*
void leer_paquetes()
{
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, 100);
    Serial.println("Contents:");
    Serial.println(packetBuffer);

  }
}
*/
/*
 * en esta rutina enviamos  vía ethernet el dato del estado de los
 * motores especificando el destinatario que es la IP y puerto del 
 * microcontrolador maestro
 */

void envio_maestroUDP ()
{
  IPAddress MaestroIP(192, 168, 5, 200);//200 termina en 200 la IP del maestro
   unsigned int MaestroPort = 8888; // puerto 8888 del maestro
   Udp.beginPacket(MaestroIP, MaestroPort);
   Udp.write(Dato_a_enviar);
   Udp.endPacket();
  //también se envía al monitor serial la información en la misma
  //rutina cuando envía al maestro
  Serial.println("el estado de los motores es");
  Serial.println(Dato_a_enviar);
             
  }
/*
En esta rutina atiende una solicitud de un servidor, es decir desde una pagina web
colocando la dirección IP (192.168.5.100),  mostrará información acerca del
esclavo 1, es decir el estado de los motores
*/
void Solicitud_servidor()
{
  
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          
          client.println("Esclavo1: ");
          Serial.println("entro solicitud servidor"); //estando en modo monitor se observa si hay solicitud
          //de una pagina web
          client.println("el estado de los motores es");
          
          client.println(Dato_a_enviar);
        


              client.println("<style type='text/css'>");
              client.println("#wrap {");
              client.println("width:600px;");
              client.println("margin:0 auto;");
              client.println("}");
              client.println("#left_col {");
              client.println("float:left;");
              client.println("width:300px;");
              client.println(" }");
              client.println(" #right_col {");
              client.println(" float:right;");
              client.println(" width:300px;");
              client.println(" }");
              client.println("</style>");
          
          client.println("<div id='wrap'>");

              client.println("<div id='left_col'>"); 
              client.println("<br> Banda Inclinada : ");
              client.println( Dato_a_enviar[3] );       
              client.println("<br> Zaranda  : " );
              client.println(Dato_a_enviar[4]);
              client.println("<br> BandaColectora  : ");
              client.println(Dato_a_enviar[5]);
              client.println("<br> PesoPruebaBandaAlimentadora  : ");
              client.println(Dato_a_enviar[6]);
              client.println("<br> BandaAgregados1  : ");
              client.println(Dato_a_enviar[7]);
              client.println("<br> BandaAgregados2  : ");
              client.println(Dato_a_enviar[8]);
              client.println("<br> BandaAgregados3  : ");
              client.println(Dato_a_enviar[9]);
              client.println("<br> BandaAgregados4  : ");
              client.println(Dato_a_enviar[10]);
              client.println("<br> BandaInclinadaReciclado  : ");
              client.println(Dato_a_enviar[11]);
              client.println("<br> BandaAgregadoReciclado  : ");
              client.println(Dato_a_enviar[12]);
              client.println("<br> MotorTrituradoReciclado  : ");
              client.println(Dato_a_enviar[13]);
              client.println("<br> PesoPruebaBandaReciclado  : ");
              client.println(Dato_a_enviar[14]);
              client.println("<br> CompuertaReciclado  : ");
              client.println(Dato_a_enviar[15]);
              client.println("<br> MotorBlowerBH  : ");
              client.println(Dato_a_enviar[16]);
              client.println("<br> MotorVannerBH  : ");
              client.println(Dato_a_enviar[17]);
              client.println("<br> SinFinColector1BH  : ");
              client.println(Dato_a_enviar[18]);
              client.println("<br> SinFinColector2BH  : ");
              client.println(Dato_a_enviar[19]);
              client.println("<br> SinFinTransversalBH  : ");
              client.println(Dato_a_enviar[20]);
              client.println("<br> Compresor  : ");
              client.println(Dato_a_enviar[21]);
              client.println("<br> BombaAsfaltoAdelante : ");
              client.println(Dato_a_enviar[22]);
              client.println("<br> BombaAsfaltoAtras : ");
              client.println(Dato_a_enviar[23]);
              client.println("<br> Extractor1BH : ");
              client.println(Dato_a_enviar[24]);
              client.println("</div>");

          client.println("<div id='right_col'>");
        
              client.println("<br> Extractor2BH : ");
              client.println(Dato_a_enviar[25]);
              client.println("<br> BlowerQuemador : ");
              client.println(Dato_a_enviar[26]);
              client.println("<br> Elevador : ");
              client.println(Dato_a_enviar[27]);
              client.println("<br> CompuertaRechazo : ");
              client.println(Dato_a_enviar[28]);
              client.println("<br> Secador : ");
              client.println(Dato_a_enviar[29]);
              client.println("<br> SlingerAdelante : ");
              client.println(Dato_a_enviar[30]);
              client.println("<br> SlingerAtras : ");
              client.println(Dato_a_enviar[31]);
              client.println("<br> BombaCombustible : ");
              client.println(Dato_a_enviar[32]);
              client.println("<br> Damper : ");
              client.println(Dato_a_enviar[33]);
              client.println("<br> PilotoValvula : ");
              client.println(Dato_a_enviar[34]);
              client.println("<br> MainLlama : ");
              client.println(Dato_a_enviar[35]);
              client.println("<br> Rotor1BH : " );
              client.println(Dato_a_enviar[36]);
              client.println("<br> Rotor2BH : " );
              client.println(Dato_a_enviar[37]);
              client.println("<br> Rotor3BH : " );
              client.println(Dato_a_enviar[38]);
              client.println("<br> AlimentacionSilo : ");
              client.println(Dato_a_enviar[39]);
              client.println("<br> ValvulaAceite : ");
              client.println(Dato_a_enviar[40]);
              client.println("<br> CompuertaGanso : ");
              client.println(Dato_a_enviar[41]);
              
              client.println("</div>");
        
           client.println("</div>");

          
          
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}


      /*  void serialEvent1()
        {
          while (Serial1.available())
          {
            lectura=Serial1.read();
          }
           Serial.print(lectura);
            
          if (lectura == 'e')
            {
            digitalWrite(EnTxPin, HIGH); //Habilitamos transmisión
            Serial1.println(Dato_a_enviar);
            Serial1.flush(); //esperamos hasta que se envíe todo el dato 
           // Serial.println(Dato_a_enviar);//delay (1000);
            digitalWrite(EnTxPin, LOW); //apagamos el canal
            }
          if (lectura == 'f')
            {
            posDatoLeido=0;


            
              while (Serial1.available()>0)
              {
              lectura2=Serial1.read();
              Dato_de_Maestro[posDatoLeido]=lectura2;
              posDatoLeido++;
              }
              Serial.println("Dato_de_Maestro");
              posDatoLeido=0;
              while (posDatoLeido <=50)
              {
              Serial.print(Dato_de_Maestro[posDatoLeido]);  
               posDatoLeido++;
                }
              
              Serial.flush();
           // Serial.println(Dato_de_Maestro);//mostrar en monitor
            }
     
     }
     

      
     //*/





//Librerías
//#include <LiquidCrystal_I2C.h>
#include <SPI.h>


//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#define BUFFER_SIZE 64 //El tamaño del buffer.


#include <Ethernet.h>
#include <EthernetUdp.h>

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xBB
};
IPAddress ip(192, 168, 5, 145);
IPAddress subnet(255, 255, 255, 0);

unsigned int localPort = 7145;      // local port to listen on

// buffers for receiving and sending data
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char packetBuffer[100];
char  ReplyBuffer[] = "acknowledged";       // a string to send back
int k=0;
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetServer server(80);
int CursorVertical = 0;

/* valores solicitados por el programa*/
        char BandaAlimentadoraEntrada ='0';
        char ZarandaEntrada ='0';
        char BandaColectoraEntrada ='0';
        char PesoPruebaBandaAlimentadoraEntrada ='0';
        char BandaAgregados1Entrada ='0';
        char BandaAgregados2Entrada ='0';
        char BandaAgregados3Entrada ='0';
        char BandaAgregados4Entrada ='0'; //cambiar ya que presenta problemas con rutina leer_paquetes
        char BandaInclinadaRecicladoEntrada ='0';
        char BandaAgregadoRecicladoEntrada ='0';  
        char MotorTrituradoRecicladoEntrada ='0';
        char PesoPruebaBandaRecicladoEntrada ='0';
        char CompuertaRecicladoEntrada ='0';
        char MotorBlowerBHEntrada ='0';
        char MotorVannerBHEntrada ='0';
        char SinFinColector1BHEntrada ='0';
        char SinFinColector2BHEntrada ='0';
        char SinFinTransversalBHEntrada ='0';
        char CompresorEntrada ='0';
        char BombaAsfaltoAdelanteEntrada ='0';
        char BombaAsfaltoAtrasEntrada ='0';
        char Extractor1BHEntrada ='0';
        char Extractor2BHEntrada ='0';
        char BlowerQuemadorEntrada ='0';
        char ElevadorEntrada ='0';
        char CompuertaRechazoEntrada ='0';
        char SecadorEntrada ='0';
        char SlingerAdelanteEntrada ='0';
        char SlingerAtrasEntrada ='0';
        char BombaCombustibleEntrada ='0';
        char DamperEntrada ='0';
        char PilotoValvulaEntrada ='0';
        char MainLlamaEntrada ='0';
        char Rotor1BHEntrada ='0';
        char Rotor2BHEntrada ='0';
        char Rotor3BHEntrada ='0';
        char AlimentacionSiloEntrada ='0';
        char ValvulaAceiteEntrada ='0';
        char ValvulaGasEntrada ='0';
        char CompuertaGansoEntrada ='0';
        char R8090ControlLLamaEntrada ='0';
        double PresionCasadeBolsas=0;
        double TemperaturaCasadeBolsas=0;
        double TemperaturaMezcla=0;
        

  
 char SolicitudBandaAlimentadora ='0';
 char SolicitudZaranda ='0';
 char SolicitudBandaColectora ='0';
 char SolicitudPesoPruebaBandaAlimentadora ='0';
 char SolicitudBandaAgregados1 ='0';
 char SolicitudBandaAgregados2 ='0';
 char SolicitudBandaAgregados3 ='0';
 char SolicitudBandaAgregados4 ='0';
 char SolicitudBandaInclinadaReciclado ='0';
 char SolicitudBandaAgregadoReciclado ='0';
 char SolicitudMotorTrituradoReciclado ='0';
 char SolicitudPesoPruebaBandaReciclado ='0';
 char SolicitudCompuertaReciclado ='0';
 char SolicitudMotorBlowerBH ='0';
 char SolicitudMotorVannerBH ='0';
 char SolicitudSinFinColector1BH ='0';
 char SolicitudSinFinColector2BH ='0';
 char SolicitudSinFinTransversalBH ='0';
 char SolicitudCompresor ='0';
 char SolicitudBombaAsfaltoAdelante ='0';
 char SolicitudBombaAsfaltoAtras ='0';
 char SolicitudExtractor1BH ='0';
 char SolicitudExtractor2BH ='0';
 char SolicitudBlowerQuemador ='0';
 char SolicitudElevador ='0';
 char SolicitudCompuertaRechazo ='0';
 char SolicitudSecador ='0';
 char SolicitudSlingerAdelante ='0';
 char SolicitudSlingerAtras ='0';
 char SolicitudBombaCombustible ='0';
 char SolicitudDamper ='0';
 char SolicitudPilotoValvula ='0';
 char SolicitudMainLlama ='0';
 char SolicitudRotor1BH ='0';
 char SolicitudRotor2BH ='0';
 char SolicitudRotor3BH ='0';
 char SolicitudAlimentacionSilo ='0';
 char SolicitudValvulaAceite ='0';
 char SolicitudValvulaGas ='0';
 char SolicitudCompuertaGanso ='0';
 char SolicitudOpenDamperBH = '0';
 char SolicitudCloseDamperBH =  '0';
 char SolicitudIncrementarDampQuemador = '0';
 char SolicitudDecrementarDampQuemador =  '0';
 char SolicitudRecycleBypass =  '0';
        
 char SolicitudValBascAsfalto ='0';
 int reinicio = 0;

        char DatoLeido[100];
         int a;//valor del dato que viene en char y se pasa a entero
        int b=0;
        

  

 
const int EnTxPin = 2;// High: Tx y Low Rx
const int SalidaBandaAlimentadora =3;
const int SalidaZaranda =4;
const int SalidaBandaColectora =5;
const int SalidaPesoPruebaBandaAlimentadora =6;
const int SalidaBandaAgregados1 =7;
const int SalidaBandaAgregados2 =8;
const int SalidaBandaAgregados3 =9;
const int SalidaBandaAgregados4 =10;
const int SalidaBandaInclinadaReciclado =11;
const int SalidaBandaAgregadoReciclado =12;
const int SalidaMotorTrituradoReciclado =13;
const int SalidaPesoPruebaBandaReciclado =22;
const int SalidaCompuertaReciclado =23;
const int SalidaMotorBlowerBH =24;
const int SalidaMotorVannerBH =25;
const int SalidaSinFinColector1BH =26;
const int SalidaSinFinColector2BH =27;
const int SalidaSinFinTransversalBH =28;
const int SalidaCompresor =29;
const int SalidaBombaAsfaltoAdelante =30;  //era el 30, solo para probar pin
const int SalidaBombaAsfaltoAtras =31;
const int SalidaExtractor1BH =32;
const int SalidaExtractor2BH =33;
const int SalidaBlowerQuemador =34;
const int SalidaElevador =35;
const int SalidaCompuertaRechazo =36;
const int SalidaSecador =37;
const int SalidaSlingerAdelante =38;
const int SalidaSlingerAtras =39;
const int SalidaBombaCombustible =40;
const int SalidaDamper =41;  // 814 SLAM DAMPER
const int SalidaPilotoValvula =42; 
const int SalidaMainLlama =43;
const int SalidaRotor1BH =44;
const int SalidaRotor2BH =45;
const int SalidaRotor3BH =46;
const int SalidaAlimentacionSilo =47;
const int SalidaValvulaAceite =48;
const int SalidaCompuertaGanso =68;
const int SalidaValvulaGas =67;   //MAIN GAS   S/V 505
const int SalidaValBascAsfalto = 58;  //607 PLC
const int SalidaIncrementarDampQuemador = 55;  //A0
const int SalidaDecrementarDampQuemador = 54;  //A1
const int SalidaOpenDamperBH = 56;    //A2
const int SalidaCloseDamperBH = 57;   //A3




char Buf [100]; 
void setup() 
{ 

  pinMode( SalidaBandaAlimentadora, OUTPUT);
  pinMode( SalidaZaranda, OUTPUT);
  pinMode( SalidaBandaColectora, OUTPUT);
  pinMode( SalidaPesoPruebaBandaAlimentadora, OUTPUT);
  pinMode( SalidaBandaAgregados1, OUTPUT);
  pinMode( SalidaBandaAgregados2, OUTPUT);
  pinMode( SalidaBandaAgregados3, OUTPUT);
  pinMode( SalidaBandaAgregados4, OUTPUT);
  pinMode( SalidaBandaInclinadaReciclado, OUTPUT);
  pinMode( SalidaBandaAgregadoReciclado, OUTPUT);
  pinMode( SalidaMotorTrituradoReciclado, OUTPUT);
  pinMode( SalidaPesoPruebaBandaReciclado, OUTPUT);
  pinMode( SalidaCompuertaReciclado, OUTPUT);
  pinMode( SalidaMotorBlowerBH, OUTPUT);
  pinMode( SalidaMotorVannerBH, OUTPUT);
  pinMode( SalidaSinFinColector1BH, OUTPUT);
  pinMode( SalidaSinFinColector2BH, OUTPUT);
  pinMode( SalidaSinFinTransversalBH, OUTPUT);
  pinMode( SalidaCompresor, OUTPUT);
  pinMode( SalidaBombaAsfaltoAdelante, OUTPUT);
  pinMode( SalidaBombaAsfaltoAtras, OUTPUT);
  pinMode( SalidaExtractor1BH, OUTPUT);
  pinMode( SalidaExtractor2BH, OUTPUT);
  pinMode( SalidaBlowerQuemador, OUTPUT);
  pinMode( SalidaElevador, OUTPUT);
  pinMode( SalidaCompuertaRechazo, OUTPUT);
  pinMode( SalidaSecador, OUTPUT);
  pinMode( SalidaSlingerAdelante, OUTPUT);
  pinMode( SalidaSlingerAtras, OUTPUT);
  pinMode( SalidaBombaCombustible, OUTPUT);
  pinMode( SalidaDamper, OUTPUT);
  pinMode( SalidaPilotoValvula, OUTPUT);
  pinMode( SalidaMainLlama, OUTPUT);
  pinMode( SalidaRotor1BH, OUTPUT);
  pinMode( SalidaRotor2BH, OUTPUT);
  pinMode( SalidaRotor3BH, OUTPUT);
  pinMode( SalidaAlimentacionSilo, OUTPUT);
  pinMode( SalidaValvulaAceite, OUTPUT);
  pinMode( SalidaValvulaGas, OUTPUT);
  pinMode (SalidaValBascAsfalto, OUTPUT);
  pinMode (SalidaIncrementarDampQuemador, OUTPUT);
  pinMode (SalidaDecrementarDampQuemador, OUTPUT);
  pinMode (SalidaOpenDamperBH, OUTPUT);
  pinMode (SalidaCloseDamperBH, OUTPUT);
  
  pinMode (54, OUTPUT);
  pinMode (55, OUTPUT);
  pinMode (56, OUTPUT);
  pinMode (57, OUTPUT);
  pinMode (58, OUTPUT);
  pinMode (59, OUTPUT);
  pinMode (60, OUTPUT);
  pinMode (61, OUTPUT);
  pinMode (62, OUTPUT);
  pinMode (63, OUTPUT);
  pinMode (64, OUTPUT);
  pinMode (65, OUTPUT);
  pinMode (66, OUTPUT);
  pinMode (67, OUTPUT);
  pinMode (68, OUTPUT);
  pinMode (69, OUTPUT);
  
  
  
  //pinMode( EstadoCompuertaGanso, OUTPUT);
  pinMode (EnTxPin, OUTPUT); // pin en modo salida
  Serial.begin(9600);//visualizar en el scope datos
  
   #define SERIAL2_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256
   #define SERIAL1_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256

Ethernet.begin(mac, ip);
server.begin();
Udp.begin(localPort);

apagartodos();


}



void loop()
  {


   EscrituraSalidas();    
   Modificar_motores();
    delay (10);                                                                                                                                        
   leer_paquetes();
    Solicitud_servidor(); 

    
    //apagartodos();
    //recorrido_relevos();  //rutina para prender uno a uno cada relevo intervalo de  un segundo
    //  prendertodos();
  }

// INTERRUPCIÓN por puerto SPI
/*
 ISR (SPI_STC_vect)
  {
    byte c = SPDR;
    Buf [pos] = c;
    pos ++;
    if (c== '\n')
    {
    LongBuf = pos;
    recibidoSPI= true;
    }
  }
  */
  /*
void leer_esclavoSPI()
  {
   if (recibidoSPI)
   {
      for (pos2 = 0; pos2<LongBuf ; pos2 ++)
      {
       // Serial.print (Buf[pos2]);
        DatoLeido[pos2] = Buf[pos2];
      }
    }
    recibidoSPI = false;
    pos=0;
  }
*/
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
   for (int i=0; i<=99; i++)
   {
    DatoLeido[i]= packetBuffer[i]; //dato recibido por ethernet
    }
  }
}


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
          
          client.println("Esclavo2: ");
          Serial.println("entro solicitud servidor"); //estando en modo monitor se observa si hay solicitud
          //de una pagina web
          client.println("la solicitud para prender motores es: ");
          client.println(DatoLeido);
          client.println("<br>");
   
          
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
          client.println( DatoLeido[3] );       
          client.println("<br> Zaranda  : " );
          client.println(DatoLeido[4]);
          client.println("<br> BandaColectora  : ");
          client.println(DatoLeido[5]);
        client.println("<br> PesoPruebaBandaAlimentadora  : ");
        client.println(DatoLeido[6]);
          client.println("<br> BandaAgregados1  : ");
        client.println(DatoLeido[7]);
          client.println("<br> BandaAgregados2  : ");
        client.println(DatoLeido[8]);
          client.println("<br> BandaAgregados3  : ");
        client.println(DatoLeido[9]);
          client.println("<br> BandaAgregados4  : ");
        client.println(DatoLeido[10]);
          client.println("<br> BandaInclinadaReciclado  : ");
        client.println(DatoLeido[11]);
          client.println("<br> BandaAgregadoReciclado  : ");
        client.println(DatoLeido[12]);
          client.println("<br> MotorTrituradoReciclado  : ");
        client.println(DatoLeido[13]);
          client.println("<br> PesoPruebaBandaReciclado  : ");
        client.println(DatoLeido[14]);
          client.println("<br> CompuertaReciclado  : ");
        client.println(DatoLeido[15]);
          client.println("<br> MotorBlowerBH  : ");
        client.println(DatoLeido[16]);
          client.println("<br> MotorVannerBH  : ");
        client.println(DatoLeido[17]);
          client.println("<br> SinFinColector1BH  : ");
        client.println(DatoLeido[18]);
          client.println("<br> SinFinColector2BH  : ");
        client.println(DatoLeido[19]);
          client.println("<br> SinFinTransversalBH  : ");
        client.println(DatoLeido[20]);
          client.println("<br> Compresor  : ");
        client.println(DatoLeido[21]);
          client.println("<br> BombaAsfaltoAdelante : ");
        client.println(DatoLeido[22]);
          client.println("<br> BombaAsfaltoAtras : ");
        client.println(DatoLeido[23]);
          client.println("<br> Extractor1BH : ");
        client.println(DatoLeido[24]);
        client.println("</div>");

        client.println("<div id='right_col'>");
        
          client.println("<br> Extractor2BH : ");
        client.println(DatoLeido[25]);
          client.println("<br> BlowerQuemador : ");
        client.println(DatoLeido[26]);
          client.println("<br> Elevador : ");
        client.println(DatoLeido[27]);
          client.println("<br> CompuertaRechazo : ");
        client.println(DatoLeido[28]);
        client.println("<br> Secador : ");
        client.println(DatoLeido[29]);
          client.println("<br> SlingerAdelante : ");
        client.println(DatoLeido[30]);
          client.println("<br> SlingerAtras : ");
        client.println(DatoLeido[31]);
          client.println("<br> BombaCombustible : ");
        client.println(DatoLeido[32]);
          client.println("<br> Damper : ");
        client.println(DatoLeido[33]);
          client.println("<br> PilotoValvula : ");
        client.println(DatoLeido[34]);
          client.println("<br> MainLlama : ");
        client.println(DatoLeido[35]);
          client.println("<br> Rotor1BH : " );
        client.println(DatoLeido[36]);
          client.println("<br> Rotor2BH : " );
        client.println(DatoLeido[37]);
          client.println("<br> Rotor3BH : " );
        client.println(DatoLeido[38]);
          client.println("<br> AlimentacionSilo : ");
         client.println(DatoLeido[39]);
          client.println("<br> ValvulaAceite : ");
        client.println(DatoLeido[40]);
          client.println("<br> CompuertaGanso : ");
        client.println(DatoLeido[41]);
          client.println("<br> ValvulaGas: ");
        client.println(DatoLeido[42]);
          client.println("<br> OpenDamperBH : ");
        client.println(DatoLeido[43]);
          client.println("<br> CloseDamperBH : ");
        client.println(DatoLeido[44]);
          client.println("<br> IncrementarDampQuemador : ");
        client.println(DatoLeido[45]);
          client.println("<br> DecrementarDampQuemador : ");
        client.println(DatoLeido[46]);
          client.println("<br> RecycleBypass : ");
        client.println(DatoLeido[47]);
          client.println("<br> ValBascAsfalto : ");
        client.println(DatoLeido[48]);
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


    
    void Modificar_motores() 
    
     { Serial.println("Modificando motores: ");
        Serial.println(DatoLeido);
       

      if (DatoLeido[0]== 'M' && DatoLeido[1]== 'E' && DatoLeido[2]== '2')
      {
        Serial.println("Confirmado de Maestro a Esclavo 2: ");
        Serial.println(DatoLeido);
        SolicitudBandaAlimentadora  = DatoLeido[3];                // ya que DatoLeido[3+1] equivalente a 3 será para las analogas
        SolicitudZaranda  = DatoLeido[4];
        SolicitudBandaColectora  = DatoLeido[5];
        SolicitudPesoPruebaBandaAlimentadora  = DatoLeido[6];
        SolicitudBandaAgregados1  = DatoLeido[7];
        SolicitudBandaAgregados2  = DatoLeido[8];
        SolicitudBandaAgregados3  = DatoLeido[9];
        SolicitudBandaAgregados4  = DatoLeido[10];
        SolicitudBandaInclinadaReciclado  = DatoLeido[11];
        SolicitudBandaAgregadoReciclado  = DatoLeido[12];
        SolicitudMotorTrituradoReciclado  = DatoLeido[13];
        SolicitudPesoPruebaBandaReciclado  = DatoLeido[14];
        SolicitudCompuertaReciclado  = DatoLeido[15];
        SolicitudMotorBlowerBH  = DatoLeido[16];
        SolicitudMotorVannerBH  = DatoLeido[17];
        SolicitudSinFinColector1BH  = DatoLeido[18];
        SolicitudSinFinColector2BH  = DatoLeido[19];
        SolicitudSinFinTransversalBH  = DatoLeido[20];
        SolicitudCompresor  = DatoLeido[21];
        SolicitudBombaAsfaltoAdelante = DatoLeido[22];
        SolicitudBombaAsfaltoAtras =  DatoLeido[23];
        SolicitudExtractor1BH =  DatoLeido[24];
        SolicitudExtractor2BH =  DatoLeido[25];
        SolicitudBlowerQuemador =  DatoLeido[26];
        SolicitudElevador =  DatoLeido[27];
        SolicitudCompuertaRechazo =  DatoLeido[28];
        SolicitudSecador =  DatoLeido[29];
        SolicitudSlingerAdelante =  DatoLeido[30];
        SolicitudSlingerAtras =  DatoLeido[31];
        SolicitudBombaCombustible =  DatoLeido[32];
        SolicitudDamper =  DatoLeido[33];
        SolicitudPilotoValvula =  DatoLeido[34];
        SolicitudMainLlama =  DatoLeido[35];
        SolicitudRotor1BH =  DatoLeido[36];
        SolicitudRotor2BH =  DatoLeido[37];
        SolicitudRotor3BH =  DatoLeido[38];
        SolicitudAlimentacionSilo =  DatoLeido[39];
        SolicitudValvulaAceite =  DatoLeido[40];
        SolicitudCompuertaGanso =  DatoLeido[41];
        SolicitudValvulaGas=  DatoLeido[42];
        SolicitudOpenDamperBH =  DatoLeido[43];
        SolicitudCloseDamperBH =  DatoLeido[44];
        SolicitudIncrementarDampQuemador =  DatoLeido[45];
        SolicitudDecrementarDampQuemador =  DatoLeido[46];
        SolicitudRecycleBypass =  DatoLeido[47];
        SolicitudValBascAsfalto =  DatoLeido[48];
        
      }  
     }


     

     

void prendertodos()

{
  // delay(1000);                  
digitalWrite( SalidaBandaAlimentadora, LOW);
digitalWrite( SalidaZaranda, LOW);
digitalWrite( SalidaBandaColectora, LOW);
digitalWrite( SalidaPesoPruebaBandaAlimentadora, LOW);
digitalWrite( SalidaBandaAgregados1, LOW);
digitalWrite( SalidaBandaAgregados2, LOW);
digitalWrite( SalidaBandaAgregados3, LOW);
digitalWrite( SalidaBandaAgregados4, LOW);
digitalWrite( SalidaBandaInclinadaReciclado, LOW);
digitalWrite( SalidaBandaAgregadoReciclado, LOW);
digitalWrite( SalidaMotorTrituradoReciclado, LOW);
digitalWrite( SalidaPesoPruebaBandaReciclado, LOW);
digitalWrite( SalidaCompuertaReciclado, LOW);
digitalWrite( SalidaMotorBlowerBH, LOW);
digitalWrite( SalidaMotorVannerBH, LOW);
digitalWrite( SalidaSinFinColector1BH, LOW);
digitalWrite( SalidaSinFinColector2BH, LOW);
digitalWrite( SalidaSinFinTransversalBH, LOW);
digitalWrite( SalidaCompresor, LOW);
digitalWrite( SalidaBombaAsfaltoAdelante, LOW);
digitalWrite( SalidaBombaAsfaltoAtras, LOW);
digitalWrite( SalidaExtractor1BH, LOW);
digitalWrite( SalidaExtractor2BH, LOW);
digitalWrite( SalidaBlowerQuemador, LOW);
digitalWrite( SalidaElevador, LOW);
digitalWrite( SalidaCompuertaRechazo, LOW);
digitalWrite( SalidaSecador, LOW);
digitalWrite( SalidaSlingerAdelante, LOW);
digitalWrite( SalidaSlingerAtras, LOW);
digitalWrite( SalidaBombaCombustible, LOW);
digitalWrite( SalidaDamper, LOW);
digitalWrite( SalidaPilotoValvula, LOW);
digitalWrite( SalidaMainLlama, LOW);
digitalWrite( SalidaRotor1BH, LOW);
digitalWrite( SalidaRotor2BH, LOW);
digitalWrite( SalidaRotor3BH, LOW);
digitalWrite( SalidaAlimentacionSilo, LOW);
digitalWrite( SalidaValvulaAceite, LOW);
digitalWrite( SalidaValvulaGas, LOW);
//digitalWrite( EstadoCompuertaGanso, LOW);
digitalWrite (EnTxPin, LOW); // pin en modo salida
/*
delay (1000);
digitalWrite( SalidaBandaAlimentadora, LOW);
digitalWrite( SalidaZaranda, HIGH);
digitalWrite( SalidaBandaColectora, HIGH);
digitalWrite( SalidaPesoPruebaBandaAlimentadora, HIGH);
digitalWrite( SalidaBandaAgregados1, LOW);
digitalWrite( SalidaBandaAgregados2, HIGH);
digitalWrite( SalidaBandaAgregados3, LOW);
digitalWrite( SalidaBandaAgregados4, HIGH);
digitalWrite( SalidaBandaInclinadaReciclado, LOW);
digitalWrite( SalidaBandaAgregadoReciclado, HIGH);
digitalWrite( SalidaMotorTrituradoReciclado, LOW);
digitalWrite( SalidaPesoPruebaBandaReciclado, HIGH);
digitalWrite( SalidaCompuertaReciclado, LOW);
digitalWrite( SalidaMotorBlowerBH, HIGH);
digitalWrite( SalidaMotorVannerBH, LOW);
digitalWrite( SalidaSinFinColector1BH, HIGH);
digitalWrite( SalidaSinFinColector2BH, LOW);
digitalWrite( SalidaSinFinTransversalBH, HIGH);
digitalWrite( SalidaCompresor, HIGH);
digitalWrite( SalidaBombaAsfaltoAdelante, HIGH);
digitalWrite( SalidaBombaAsfaltoAtras, HIGH);
digitalWrite( SalidaExtractor1BH, HIGH);
digitalWrite( SalidaExtractor2BH, HIGH);
digitalWrite( SalidaBlowerQuemador, HIGH);
digitalWrite( SalidaElevador, HIGH);
digitalWrite( SalidaCompuertaRechazo, HIGH);
digitalWrite( SalidaSecador, HIGH);
digitalWrite( SalidaSlingerAdelante, HIGH);
digitalWrite( SalidaSlingerAtras, HIGH);
digitalWrite( SalidaBombaCombustible, HIGH);
digitalWrite( SalidaDamper, HIGH);
digitalWrite( SalidaPilotoValvula, HIGH);
digitalWrite( SalidaMainLlama, HIGH);
digitalWrite( SalidaRotor1BH, HIGH);
digitalWrite( SalidaRotor2BH, HIGH);
digitalWrite( SalidaRotor3BH, HIGH);
digitalWrite( SalidaAlimentacionSilo, HIGH);
digitalWrite( SalidaValvulaAceite, LOW);
digitalWrite( SalidaValvulaGas, HIGH);
//digitalWrite( EstadoCompuertaGanso, HIGH);
digitalWrite (EnTxPin, HIGH); // pin en modo salida*/
}

void apagartodos()

{
  // delay(1000);                  
digitalWrite( SalidaBandaAlimentadora, HIGH);
digitalWrite( SalidaZaranda, HIGH);
digitalWrite( SalidaBandaColectora, HIGH);
digitalWrite( SalidaPesoPruebaBandaAlimentadora, HIGH);
digitalWrite( SalidaBandaAgregados1, HIGH);
digitalWrite( SalidaBandaAgregados2, HIGH);
digitalWrite( SalidaBandaAgregados3, HIGH);
digitalWrite( SalidaBandaAgregados4, HIGH);
digitalWrite( SalidaBandaInclinadaReciclado, HIGH);
digitalWrite( SalidaBandaAgregadoReciclado, HIGH);
digitalWrite( SalidaMotorTrituradoReciclado, HIGH);
digitalWrite( SalidaPesoPruebaBandaReciclado, HIGH);
digitalWrite( SalidaCompuertaReciclado, HIGH);
digitalWrite( SalidaMotorBlowerBH, HIGH);
digitalWrite( SalidaMotorVannerBH, HIGH);
digitalWrite( SalidaSinFinColector1BH, HIGH);
digitalWrite( SalidaSinFinColector2BH, HIGH);
digitalWrite( SalidaSinFinTransversalBH, HIGH);
digitalWrite( SalidaCompresor, HIGH);
digitalWrite( SalidaBombaAsfaltoAdelante, HIGH);
digitalWrite( SalidaBombaAsfaltoAtras, HIGH);
digitalWrite( SalidaExtractor1BH, HIGH);
digitalWrite( SalidaExtractor2BH, HIGH);
digitalWrite( SalidaBlowerQuemador, HIGH);
digitalWrite( SalidaElevador, HIGH);
digitalWrite( SalidaCompuertaRechazo, HIGH);
digitalWrite( SalidaSecador, HIGH);
digitalWrite( SalidaSlingerAdelante, HIGH);
digitalWrite( SalidaSlingerAtras, HIGH);
digitalWrite( SalidaBombaCombustible, HIGH);
digitalWrite( SalidaDamper, HIGH);
digitalWrite( SalidaPilotoValvula, HIGH);
digitalWrite( SalidaMainLlama, HIGH);
digitalWrite( SalidaRotor1BH, HIGH);
digitalWrite( SalidaRotor2BH, HIGH);
digitalWrite( SalidaRotor3BH, HIGH);
digitalWrite( SalidaAlimentacionSilo, HIGH);
digitalWrite( SalidaValvulaAceite, HIGH);
digitalWrite( SalidaValvulaGas, HIGH);
digitalWrite (EnTxPin, HIGH); // pin en modo salida
digitalWrite (54, HIGH); // pin en modo salida
digitalWrite (55, HIGH); // pin en modo salida
digitalWrite (56, HIGH); // pin en modo salida
digitalWrite (57, HIGH); // pin en modo salida
digitalWrite (58, HIGH); // pin en modo salida
digitalWrite (59, HIGH); // pin en modo salida
digitalWrite (60, HIGH); // pin en modo salida
digitalWrite (65, HIGH); // pin en modo salida
digitalWrite (66, HIGH); // pin en modo salida
digitalWrite (67, LOW); // pin en modo salida
digitalWrite (68, HIGH); // pin en modo salida
digitalWrite (69, HIGH); // pin en modo salida

}



void EscrituraSalidas()
{
  if (SolicitudBandaAlimentadora == '1')
  digitalWrite( SalidaBandaAlimentadora, LOW);
  else
  digitalWrite( SalidaBandaAlimentadora, HIGH);

  if ( SolicitudZaranda == '1')
  digitalWrite(SalidaZaranda, LOW);
  else
  digitalWrite(SalidaZaranda, HIGH);  

  
  if (SolicitudBandaColectora == '1')
  digitalWrite (SalidaBandaColectora, LOW);
  else
  digitalWrite (SalidaBandaColectora, HIGH);

  if (SolicitudPesoPruebaBandaAlimentadora == '1')
  digitalWrite (SalidaPesoPruebaBandaAlimentadora, LOW);
  else
  digitalWrite (SalidaPesoPruebaBandaAlimentadora, HIGH);

  if (SolicitudBandaAgregados1 == '1')
  digitalWrite (SalidaBandaAgregados1, LOW);
  else
  digitalWrite (SalidaBandaAgregados1, HIGH);

  if (SolicitudBandaAgregados2 == '1')
  digitalWrite (SalidaBandaAgregados2, LOW);
  else
  digitalWrite (SalidaBandaAgregados2, HIGH);

  if (SolicitudBandaAgregados3 == '1')
  digitalWrite (SalidaBandaAgregados3, LOW);
  else
  digitalWrite (SalidaBandaAgregados3, HIGH);


  if (SolicitudBandaAgregados4 == '1')
  digitalWrite (SalidaBandaAgregados4, LOW);
  else
  digitalWrite (SalidaBandaAgregados4, HIGH);

 if (SolicitudBandaInclinadaReciclado == '1')
  digitalWrite (SalidaBandaInclinadaReciclado, LOW);
  else
  digitalWrite (SalidaBandaInclinadaReciclado, HIGH);

  if (SolicitudBandaAgregadoReciclado == '1')
  digitalWrite (SalidaBandaAgregadoReciclado, LOW);
  else
  digitalWrite (SalidaBandaAgregadoReciclado, HIGH);

  if (SolicitudMotorTrituradoReciclado == '1')
  digitalWrite (SalidaMotorTrituradoReciclado, LOW);
  else
  digitalWrite (SalidaMotorTrituradoReciclado, HIGH);

  if (SolicitudPesoPruebaBandaReciclado == '1')
  digitalWrite (SalidaPesoPruebaBandaReciclado, LOW);
  else
  digitalWrite (SalidaPesoPruebaBandaReciclado, HIGH);

  if (SolicitudCompuertaReciclado == '1')
  digitalWrite (SalidaCompuertaReciclado, LOW);
  else
  digitalWrite (SalidaCompuertaReciclado, HIGH);

  
  if (SolicitudMotorBlowerBH == '1')
  digitalWrite (SalidaMotorBlowerBH, LOW);
  else
  digitalWrite (SalidaMotorBlowerBH, HIGH);

  if (SolicitudMotorVannerBH == '1')
  digitalWrite (SalidaMotorVannerBH, LOW);
  else
  digitalWrite (SalidaMotorVannerBH, HIGH);
 
  if (SolicitudSinFinColector1BH == '1')
  digitalWrite (SalidaSinFinColector1BH, LOW);
  else
  digitalWrite (SalidaSinFinColector1BH, HIGH);

  if (SolicitudSinFinColector2BH == '1')
  digitalWrite (SalidaSinFinColector2BH, LOW);
  else
  digitalWrite (SalidaSinFinColector2BH, HIGH);

  if (SolicitudSinFinTransversalBH == '1')
  digitalWrite (SalidaSinFinTransversalBH, LOW);
  else
  digitalWrite (SalidaSinFinTransversalBH, HIGH);

  if (SolicitudCompresor == '1')
  digitalWrite (SalidaCompresor, LOW);
  else
  digitalWrite (SalidaCompresor, HIGH);

  if (SolicitudBombaAsfaltoAdelante == '1')
  digitalWrite (SalidaBombaAsfaltoAdelante, LOW);
  else
  digitalWrite (SalidaBombaAsfaltoAdelante, HIGH);

  if (SolicitudBombaAsfaltoAtras == '1')
  digitalWrite (SalidaBombaAsfaltoAtras, LOW);
  else
  digitalWrite (SalidaBombaAsfaltoAtras, HIGH);
  
  
  if (SolicitudExtractor1BH == '1')
  digitalWrite (SalidaExtractor1BH, LOW);
  else
  digitalWrite (SalidaExtractor1BH, HIGH);

  if (SolicitudExtractor2BH == '1')
  digitalWrite (SalidaExtractor2BH, LOW);
  else
  digitalWrite (SalidaExtractor2BH, HIGH);

  if (SolicitudBlowerQuemador == '1')
  digitalWrite (SalidaBlowerQuemador, LOW);
  else
  digitalWrite (SalidaBlowerQuemador, HIGH);

  if (SolicitudElevador == '1')
  digitalWrite (SalidaElevador, LOW);
  else
  digitalWrite (SalidaElevador, HIGH);

  if (SolicitudCompuertaRechazo == '1')
  digitalWrite (SalidaCompuertaRechazo, LOW);
  else
  digitalWrite (SalidaCompuertaRechazo, HIGH);

  if (SolicitudSecador == '1')
  digitalWrite (SalidaSecador, LOW);
  else
  digitalWrite (SalidaSecador, HIGH);

  if (SolicitudSlingerAdelante == '1')
  digitalWrite (SalidaSlingerAdelante, LOW);
  else
  digitalWrite (SalidaSlingerAdelante, HIGH);

  if (SolicitudSlingerAtras == '1')
  digitalWrite (SalidaSlingerAtras, LOW);
  else
  digitalWrite (SalidaSlingerAtras, HIGH);

  if (SolicitudBombaCombustible == '1')
  digitalWrite (SalidaBombaCombustible, LOW);
  else
  digitalWrite (SalidaBombaCombustible, HIGH);

  if (SolicitudDamper == '1')
  digitalWrite (SalidaDamper, LOW);
  else
  digitalWrite (SalidaDamper, HIGH);

  if (SolicitudPilotoValvula == '1')
  digitalWrite (SalidaPilotoValvula, LOW);
  else
  digitalWrite (SalidaPilotoValvula, HIGH);

  if (SolicitudMainLlama == '1')
  digitalWrite (SalidaMainLlama, LOW);
  else
  digitalWrite (SalidaMainLlama, HIGH);

  if (SolicitudRotor1BH == '1')
  digitalWrite (SalidaRotor1BH, LOW);
  else
  digitalWrite (SalidaRotor1BH, HIGH);

  if (SolicitudRotor2BH == '1')
  digitalWrite (SalidaRotor2BH, LOW);
  else
  digitalWrite (SalidaRotor2BH, HIGH);

  if (SolicitudRotor3BH == '1')
  digitalWrite (SalidaRotor3BH, LOW);
  else
  digitalWrite (SalidaRotor3BH, HIGH);

  if (SolicitudAlimentacionSilo == '1')
  digitalWrite (SalidaAlimentacionSilo, LOW);
  else
  digitalWrite (SalidaAlimentacionSilo, HIGH);

  if (SolicitudValvulaAceite== '1')
  digitalWrite (SalidaValvulaAceite, LOW);
  else
  digitalWrite (SalidaValvulaAceite, HIGH);

  if (SolicitudValvulaGas == '1')
  digitalWrite (SalidaValvulaGas, LOW);
  else
  digitalWrite (SalidaValvulaGas, HIGH);

if (SolicitudOpenDamperBH == '1')
  digitalWrite (SalidaOpenDamperBH, LOW);
  else
  digitalWrite (SalidaOpenDamperBH, HIGH);

if (SolicitudCloseDamperBH == '1')
  digitalWrite (SalidaCloseDamperBH, LOW);
  else
  digitalWrite (SalidaCloseDamperBH, HIGH);
  

  if (SolicitudIncrementarDampQuemador == '1')
  digitalWrite (SalidaIncrementarDampQuemador, LOW);
  else
  digitalWrite (SalidaIncrementarDampQuemador, HIGH);

if (SolicitudDecrementarDampQuemador == '1')
  digitalWrite (SalidaDecrementarDampQuemador, LOW);
  else
  digitalWrite (SalidaDecrementarDampQuemador, HIGH);

  if (SolicitudCompuertaGanso == '1')
  digitalWrite (SalidaCompuertaGanso, LOW);
  else
  digitalWrite (SalidaCompuertaGanso, HIGH);


if (SolicitudValBascAsfalto == '1')
  digitalWrite (SalidaValBascAsfalto, LOW);
  else
  digitalWrite (SalidaValBascAsfalto, HIGH);


 
 }
  

void recorrido_relevos()
{

digitalWrite( SalidaBandaAlimentadora, LOW); //PIN MICRO 03
delay(1000);
digitalWrite( SalidaBandaAlimentadora, HIGH);//PIN MICRO 03
digitalWrite( SalidaZaranda, LOW); //PIN MICRO 04
delay(1000);
digitalWrite( SalidaZaranda, HIGH); //PIN MICRO 04
digitalWrite( SalidaBandaColectora, LOW); //PIN MICRO 05
delay(1000);
digitalWrite( SalidaBandaColectora, HIGH);//PIN MICRO 05
digitalWrite( SalidaPesoPruebaBandaAlimentadora, LOW); //PIN MICRO 06
delay(1000);
digitalWrite( SalidaPesoPruebaBandaAlimentadora, HIGH); //PIN MICRO 06
digitalWrite( SalidaBandaAgregados1, LOW); //PIN MICRO 07
delay(1000);
digitalWrite( SalidaBandaAgregados1, HIGH); //PIN MICRO 07
digitalWrite( SalidaBandaAgregados2, LOW); //PIN MICRO 08
delay(1000);
digitalWrite( SalidaBandaAgregados2, HIGH); //PIN MICRO 08
digitalWrite( SalidaBandaAgregados3, LOW); //PIN MICRO 9
delay(1000);
digitalWrite( SalidaBandaAgregados3, HIGH); //PIN MICRO 9
digitalWrite( SalidaBandaAgregados4, LOW); //PIN MICRO 10
delay(1000);
digitalWrite( SalidaBandaAgregados4, HIGH); //PIN MICRO 10
digitalWrite( SalidaBandaInclinadaReciclado, LOW); //PIN MICRO 11
delay(1000);
digitalWrite( SalidaBandaInclinadaReciclado, HIGH); //PIN MICRO 11
digitalWrite( SalidaBandaAgregadoReciclado, LOW); //PIN MICRO 12
delay(1000);
digitalWrite( SalidaBandaAgregadoReciclado, HIGH); //PIN MICRO 12
digitalWrite( SalidaMotorTrituradoReciclado, LOW); //PIN MICRO 13
delay(1000);
digitalWrite( SalidaMotorTrituradoReciclado, HIGH); //PIN MICRO 13
digitalWrite( SalidaPesoPruebaBandaReciclado, LOW); //PIN MICRO 22
delay(1000);
digitalWrite( SalidaPesoPruebaBandaReciclado, HIGH); //PIN MICRO 22
digitalWrite( SalidaCompuertaReciclado, LOW); //PIN MICRO 23
delay(1000);
digitalWrite( SalidaCompuertaReciclado, HIGH); //PIN MICRO 23
digitalWrite( SalidaMotorBlowerBH, LOW); //PIN MICRO 24
delay(1000);
digitalWrite( SalidaMotorBlowerBH, HIGH); //PIN MICRO 24
digitalWrite( SalidaMotorVannerBH, LOW); //PIN MICRO 25
delay(1000);
digitalWrite( SalidaMotorVannerBH, HIGH); //PIN MICRO 25
digitalWrite( SalidaSinFinColector1BH, LOW); //PIN MICRO 26
delay(1000);
digitalWrite( SalidaSinFinColector1BH, HIGH); //PIN MICRO 26
digitalWrite( SalidaSinFinColector2BH, LOW);  //PIN MICRO 27
delay(1000);
digitalWrite( SalidaSinFinColector2BH, HIGH);  //PIN MICRO 27
digitalWrite( SalidaSinFinTransversalBH, LOW); //PIN MICRO 28
delay(1000);
digitalWrite( SalidaSinFinTransversalBH, HIGH); //PIN MICRO 28 
digitalWrite( SalidaCompresor, LOW); //PIN MICRO 29
delay(1000);
digitalWrite( SalidaCompresor, HIGH); //PIN MICRO 29
digitalWrite( SalidaBombaAsfaltoAdelante, LOW); //PIN MICRO 30
delay(1000);
digitalWrite( SalidaBombaAsfaltoAdelante, HIGH); //PIN MICRO 30
digitalWrite( SalidaBombaAsfaltoAtras, LOW); //PIN MICRO 31
delay(1000);
digitalWrite( SalidaBombaAsfaltoAtras, HIGH); //PIN MICRO 31
digitalWrite( SalidaExtractor1BH, LOW); //PIN MICRO 32
delay(1000);
digitalWrite( SalidaExtractor1BH, HIGH); //PIN MICRO 32
digitalWrite( SalidaExtractor2BH, LOW); //PIN MICRO 33
delay(1000);
digitalWrite( SalidaExtractor2BH, HIGH);  //PIN MICRO 33
digitalWrite( SalidaBlowerQuemador, LOW); //PIN MICRO 34
delay(1000);
digitalWrite( SalidaBlowerQuemador, HIGH); //PIN MICRO 34
digitalWrite( SalidaElevador, LOW); //PIN MICRO 35
delay(1000);
digitalWrite( SalidaElevador, HIGH);  //PIN MICRO 35
digitalWrite( SalidaCompuertaRechazo, LOW); //PIN MICRO 36
delay(1000);
digitalWrite( SalidaCompuertaRechazo, HIGH); //PIN MICRO 36
digitalWrite( SalidaSecador, LOW);  //PIN MICRO 37
delay(1000);
digitalWrite( SalidaSecador, HIGH);  //PIN MICRO 37
digitalWrite( SalidaSlingerAdelante, LOW); //PIN MICRO 38
delay(1000);
digitalWrite( SalidaSlingerAdelante, HIGH); //PIN MICRO 38
digitalWrite( SalidaSlingerAtras, LOW); //PIN MICRO 39
delay(1000);
digitalWrite( SalidaSlingerAtras, HIGH); //PIN MICRO 39
digitalWrite( SalidaBombaCombustible, LOW); //PIN MICRO 40
delay(1000);
digitalWrite( SalidaBombaCombustible, HIGH); //PIN MICRO 40
digitalWrite( SalidaDamper, LOW); //PIN MICRO 41
delay(1000);
digitalWrite( SalidaDamper, HIGH);       //PIN MICRO 41
digitalWrite( SalidaPilotoValvula, LOW); //PIN MICRO 42
delay(1000);
digitalWrite( SalidaPilotoValvula, HIGH); //PIN MICRO 42
digitalWrite( SalidaMainLlama, LOW);  //PIN MICRO 43
delay(1000);
digitalWrite( SalidaMainLlama, HIGH); //PIN MICRO 43
digitalWrite( SalidaRotor1BH, LOW); //PIN MICRO 44
delay(1000);
digitalWrite( SalidaRotor1BH, HIGH); //PIN MICRO 44
digitalWrite( SalidaRotor2BH, LOW); //PIN MICRO 45
delay(1000);
digitalWrite( SalidaRotor2BH, HIGH); //PIN MICRO 45
digitalWrite( SalidaRotor3BH, LOW);//PIN MICRO 46
delay(1000);
digitalWrite( SalidaRotor3BH, HIGH); //PIN MICRO 46

/*

const int SalidaMainLlama =43;
const int SalidaRotor1BH =44;
const int SalidaRotor2BH =45;
const int SalidaRotor3BH =46;
const int SalidaAlimentacionSilo =47;
const int SalidaValvulaAceite =48;
const int SalidaCompuertaGanso =49;
const int SalidaValvulaGas =50;
const int SalidaValBascAsfalto = 68;
const int SalidaIncDamperQuemador = 67;
const int SalidaDecDamperQuemador = 66;
const int SalidaOpenDamperBH = 65;
const int SalidaCloseDamperBH = 64

*/


digitalWrite( SalidaAlimentacionSilo, LOW); //PIN MICRO 47
delay(1000);
digitalWrite( SalidaAlimentacionSilo, HIGH); //PIN MICRO 47
digitalWrite( SalidaValvulaAceite, LOW);    //PIN MICRO 48
delay(1000);
digitalWrite( SalidaValvulaAceite, HIGH); //PIN MICRO 48

digitalWrite( 68, LOW);
delay(1000);
digitalWrite( 68, HIGH);

digitalWrite( 69, LOW);
delay(1000);
digitalWrite( 69, HIGH);

digitalWrite( SalidaIncrementarDampQuemador, LOW);  //PIN MICRO 54
delay(1000);
digitalWrite( SalidaIncrementarDampQuemador, HIGH); //PIN MICRO 54

digitalWrite( SalidaDecrementarDampQuemador, LOW); //PIN MICRO 55
delay(1000);
digitalWrite( SalidaDecrementarDampQuemador, HIGH); //PIN MICRO 55

digitalWrite( SalidaCloseDamperBH, LOW); //PIN MICRO 56
delay(1000);
digitalWrite( SalidaCloseDamperBH, HIGH); //PIN MICRO 56

digitalWrite( SalidaOpenDamperBH, LOW); //PIN MICRO 57
delay(1000);
digitalWrite( SalidaOpenDamperBH, HIGH); //PIN MICRO 57

digitalWrite( SalidaValBascAsfalto, LOW);  //PIN MICRO 58
delay(1000);
digitalWrite( SalidaValBascAsfalto, HIGH); //PIN MICRO 58

digitalWrite( 59, LOW);
delay(1000);
digitalWrite( 59, HIGH);

/*digitalWrite( 60, LOW);
delay(1000);
digitalWrite( 60, HIGH);

digitalWrite( 61, LOW);
delay(1000);
digitalWrite( 61, HIGH);

digitalWrite( 62, LOW);
delay(1000);
digitalWrite( 62, HIGH);

digitalWrite( 63, LOW);
delay(1000);
digitalWrite( 63, HIGH);

digitalWrite( 64, LOW);
delay(1000);
digitalWrite( 64, HIGH);
*/
digitalWrite( 65, LOW);
delay(1000);
digitalWrite( 65, HIGH);

digitalWrite( 66, LOW);
delay(1000);
digitalWrite( 66, HIGH);
/*
digitalWrite( 67, LOW);
delay(1000);
digitalWrite( 67, HIGH);
*/




/*

digitalWrite( SalidaCompuertaGanso, LOW);
delay(1000);
digitalWrite( SalidaCompuertaGanso, HIGH);
digitalWrite( SalidaValvulaGas, LOW);
delay(1000);
digitalWrite( SalidaValvulaGas, HIGH);
digitalWrite( SalidaValBascAsfalto, LOW);
delay(1000);
digitalWrite( SalidaValBascAsfalto, HIGH);


digitalWrite( SalidaIncDamperQuemador, LOW);
delay(1000);
digitalWrite( SalidaIncDamperQuemador, HIGH);
digitalWrite( SalidaDecDamperQuemador, LOW);
delay(1000);
digitalWrite( SalidaDecDamperQuemador, HIGH);
digitalWrite( SalidaOpenDamperBH, LOW);
delay(1000);
digitalWrite( SalidaOpenDamperBH, HIGH);
digitalWrite( SalidaCloseDamperBH, LOW);
delay(1000);
digitalWrite( SalidaCloseDamperBH, HIGH);
  
*/




}

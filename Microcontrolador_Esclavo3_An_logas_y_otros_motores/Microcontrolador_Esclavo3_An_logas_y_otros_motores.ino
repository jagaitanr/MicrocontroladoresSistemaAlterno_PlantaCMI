



//Librerías
//#include <LiquidCrystal_I2C.h>
#include <SPI.h>

#define BUFFER_SIZE 64 //El tamaño del buffer.

#include <Ethernet.h>
#include <EthernetUdp.h>

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xCC
};
IPAddress ip(192, 168, 5, 181);
IPAddress subnet(255, 255, 255, 0);

unsigned int localPort = 7185;      // local port to listen on


// buffers for receiving and sending data
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char packetBuffer[100];
char  ReplyBuffer[] = "acknowledged";       // a string to send back
int k=0;
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetServer server(80);




double PresionCasadeBolsas=0;
double TemperaturaCasadeBolsas=0;
double TemperaturadeMezcla=0;
double TemperaturadeAsfalto=0;
double TemperaturaExaustDrum=0;
double PosicionDamperBH=0;
double PosicionValvulaCombustible=0;
double PosicionDamperBHSolicitado=0;   //dato solicitado por el usuario
double PosicionDamperQuemadorSolicitado=0; //dato solicitado por el usuario


char PressBH[10];   //To hold . and null
char TempeBH[10];
char TempeMz[10];
char TempeAs[10];
char TempeExaustDrum[10];
char PosDamp[10];
char PosQuem[10];

 

const int SalidaValvulaGas = 22;
const int SalidaIncDamperQuemador = 24;
const int SalidaDecDamperQuemador = 23;
const int SalidaOpenDamperBH = 25;
const int SalidaCloseDamperBH = 26;

int PressBHPin=A14;
int TempBHPin=A8;
int TempMzPin=A11;
int TempAsfPin=A9;
int PosDamBHPin=A15;
int PosValvFuelPin=A13;
int TempExaustDrumPin=A10;

int EstadoIncDamperQuemador = 0;
int EstadoDecDamperQuemador = 0;
int EstadoOpenDamperBH = 0;
int EstadoCloseDamperBH = 0;

String BufSPIrx;

volatile boolean recibidoSPI; // variable booleana de recibido SPI
volatile byte pos;
char BufSPI [100]; 
char BufSPI2 [100];
char DatoLeido2[100];
char Dato_a_enviar[100];
int LongBuf = 0;  //lee el largo de la caden Buf recibida (SPI)
int pos2=0;  
int posSPI=0;
int VelocidadAgregado1Hz = 0; 
int VelocidadAgregado2Hz = 0; 
int VelocidadAgregado3Hz = 0; 
int VelocidadAgregado4Hz = 0; 
int VelocidadAgregadoRecicladoHz = 0; 
int VelocidadAsfaltoHz = 0; 
int posDatoLeido = 0;
volatile byte command = 0;

void ss_falling()
{command = 0;}//fin del servicio de rotina de la interrupcion (ISR) ss_falling


void setup() 
{ 

  Serial.begin(9600);//visualizar en el scope datos
  
   #define SERIAL2_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256
   #define SERIAL1_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256
  Ethernet.begin(mac, ip);
  server.begin();
  Udp.begin(localPort);

  pinMode( SalidaIncDamperQuemador, OUTPUT);
  pinMode( SalidaDecDamperQuemador, OUTPUT);

  pinMode ( SalidaOpenDamperBH, OUTPUT);
  pinMode ( SalidaCloseDamperBH, OUTPUT);


  }


void loop()
  {
    leer_paquetes();
    Solicitud_servidor();
    delay(100);
    //Mostrar_dato_Variadores();
    
    
//   leer_esclavoSPI();

    
   ActualizarVariadores();
   AplicarPWM();
  //  ActualizarMotores();
   LecturaAnalogas();
   EnvioAnalogas();
   moverDamperes();   
   //  delay(1000);
   /* Serial.print("datosdigitales");
    Serial.print(EstadoIncDamperQuemador);
    Serial.print(EstadoDecDamperQuemador);
    Serial.print(EstadoOpenDamperBH);
    Serial.println(EstadoCloseDamperBH);
    */

  
  }

void envio_maestroUDP ()
{
  IPAddress MaestroIP(192, 168, 5, 200);//200 termina en 200 la IP del maestro
   unsigned int MaestroPort = 8888; // puerto 8888 del maestro
   Udp.beginPacket(MaestroIP, MaestroPort);
   Udp.write(Dato_a_enviar);
   Udp.endPacket();
  //también se envía al monitor serial la información en la misma
  //rutina cuando envía al maestro
  Serial.println("las señales analogas a enviar son: ");
  Serial.println(Dato_a_enviar);
             
  }


  void LecturaAnalogas ()

  {
    int PressBHPin=A8;
 PresionCasadeBolsas= analogRead(PressBHPin)*0.009766; // presión va de 0 a 10miliBares
 TemperaturaCasadeBolsas=(analogRead(TempBHPin)-197)*0.5086263;// temperatura de termocupla de 0-500°C  (Vmin = 0.96v =>0°C, max 4.8v=500°C)
 TemperaturadeMezcla=(analogRead(TempMzPin)-197)*0.5086263;// temperatura de termocupla de 0-500°C;
 TemperaturadeAsfalto=(analogRead(TempAsfPin)-197)*0.5086263;// temperatura de termocupla de 0-500°C;
 TemperaturaExaustDrum=(analogRead(TempExaustDrumPin)-197)*0.5086263;//temperatura salida secador
 PosicionDamperBH=analogRead(PosDamBHPin)*0.09766; // posición hasta el 100%
 PosicionValvulaCombustible=analogRead(PosValvFuelPin)*0.09766;//posición hasta el 100%
 
/* PresionCasadeBolsas = 0.3578945845;  
 TemperaturaCasadeBolsas = 45.8954585;  
 TemperaturadeMezcla=165.468745212;
 TemperaturadeAsfalto=5.1564865121;
 PosicionDamperBH=100.1;
 PosicionValvulaCombustible=5.4564215;
*/ 
 
 dtostrf(PresionCasadeBolsas,5,2,PressBH);  //convierte el valor de Presion cada de bolsas en un arreglos, con 5 numeros en total y los dos ultimos son decimales
 dtostrf(TemperaturaCasadeBolsas,5,2,TempeBH);
 dtostrf(TemperaturadeMezcla,5,2,TempeMz);
 dtostrf(TemperaturadeAsfalto,5,2,TempeAs);
 dtostrf(TemperaturaExaustDrum,5,2,TempeExaustDrum);
 dtostrf(PosicionDamperBH,5,2,PosDamp);
 dtostrf(PosicionDamperBH,5,2,PosDamp);
 dtostrf(PosicionValvulaCombustible,5,2,PosQuem);
 
    
  }

   void EnvioAnalogas()

   {
   Dato_a_enviar[0]='E';
   Dato_a_enviar[1]='3';
   Dato_a_enviar[2]='M';
   Dato_a_enviar[3]='P';
   Dato_a_enviar[4]='B';
   Dato_a_enviar[5]=PressBH[0];
   Dato_a_enviar[6]=PressBH[1];
   Dato_a_enviar[7]=PressBH[2];
   Dato_a_enviar[8]=PressBH[3];
   Dato_a_enviar[9]=PressBH[4];
   Dato_a_enviar[10]='T';
   Dato_a_enviar[11]='B';
   Dato_a_enviar[12]=TempeBH[0];
   Dato_a_enviar[13]=TempeBH[1];
   Dato_a_enviar[14]=TempeBH[2];
   Dato_a_enviar[15]=TempeBH[3];
   Dato_a_enviar[16]=TempeBH[4];
   Dato_a_enviar[17]='T';
   Dato_a_enviar[18]='M';
   Dato_a_enviar[19]=TempeMz[0];
   Dato_a_enviar[20]=TempeMz[1];
   Dato_a_enviar[21]=TempeMz[2];
   Dato_a_enviar[22]=TempeMz[3];
   Dato_a_enviar[23]=TempeMz[4];
   Dato_a_enviar[24]='T';
   Dato_a_enviar[25]='A';
   Dato_a_enviar[26]=TempeAs[0];
   Dato_a_enviar[27]=TempeAs[1];
   Dato_a_enviar[28]=TempeAs[2];
   Dato_a_enviar[29]=TempeAs[3];
   Dato_a_enviar[30]=TempeAs[4];
   Dato_a_enviar[31]='T';
   Dato_a_enviar[32]='D';
   Dato_a_enviar[33]=TempeExaustDrum[0];
   Dato_a_enviar[34]=TempeExaustDrum[1];
   Dato_a_enviar[35]=TempeExaustDrum[2];
   Dato_a_enviar[36]=TempeExaustDrum[3];
   Dato_a_enviar[37]=TempeExaustDrum[4];
   Dato_a_enviar[38]='P';
   Dato_a_enviar[39]='D';
   Dato_a_enviar[40]=PosDamp[0];
   Dato_a_enviar[41]=PosDamp[1];
   Dato_a_enviar[42]=PosDamp[2];
   Dato_a_enviar[43]=PosDamp[3];
   Dato_a_enviar[44]=PosDamp[4];
   Dato_a_enviar[45]='P';
   Dato_a_enviar[46]='Q';
   Dato_a_enviar[47]=PosQuem[0];
   Dato_a_enviar[48]=PosQuem[1];
   Dato_a_enviar[49]=PosQuem[2];
   Dato_a_enviar[50]=PosQuem[3];
   Dato_a_enviar[51]=PosQuem[4];
   Dato_a_enviar[52]='F';
   Dato_a_enviar[53]='I';
   Dato_a_enviar[54]='N';
   envio_maestroUDP(); 
    }

/*
  void Mostrar_dato_Variadores()
  {
    Serial.println ("Dato Leido del Maestro");
    for (int a = 0; a<=50; a++)
    {
      
    Serial.print (DatoLeido2[a]);
    }
    Serial.println (int (VelocidadAgregado1Hz*0.0425));
    Serial.println (VelocidadAgregado2Hz);
    Serial.println (VelocidadAgregado3Hz);
    Serial.println (VelocidadAgregado4Hz);
    Serial.println (VelocidadAgregadoRecicladoHz);
    Serial.println (VelocidadAsfaltoHz);
   }

*/
void leer_paquetes()
{
  Serial.print ("entre a rutina de leer paquetes, paquetes recibidos: ");
  int packetSize = Udp.parsePacket();
  Serial.println (packetSize);
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
    DatoLeido2[i]= packetBuffer[i]; //dato recibido por ethernet
    }
  }
  else
  {
    Serial.println ("no hay packetsize");
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
          
          client.println("Esclavo3: ");
          Serial.println("entro solicitud servidor"); //estando en modo monitor se observa si hay solicitud
          //de una pagina web
          client.println("los datos para los variadores es:");
          client.println(DatoLeido2);
          client.println("<p>");
          client.println("la velocidad para el agregado 1 es:");
          client.println(VelocidadAgregado1Hz);
          client.println("</p>");
          client.println("<p>");
          client.println("la velocidad para el agregado 2 es:");
          client.println(VelocidadAgregado2Hz);
          client.println("</p>");
          client.println("<p>");
          client.println("la velocidad para el agregado 3 es:");
          client.println(VelocidadAgregado3Hz);
          client.println("</p>");
          client.println("<p>");
          client.println("la velocidad para el agregado 4 es:");
          client.println(VelocidadAgregado4Hz);
          client.println("</p>");
          client.println("<p>");
          client.println("la velocidad para el asfalto es:");
          client.println(VelocidadAsfaltoHz);
          client.println("</p>");
          client.println("<p>");
          client.println("la velocidad para el reciclado es:");
          client.println(VelocidadAgregadoRecicladoHz);
          client.println("</p>");
          client.println("<p>");
          client.println("la temperatura de la Mezcla es::");
          client.println(TemperaturadeMezcla);
          client.println("</p>");
          client.println("<p>");
          client.println("la temperatura de Salida Casa de Bolsas es::");
          client.println(TemperaturaCasadeBolsas);
          client.println("</p>");
          client.println("<p>");
          client.println("la temperatura del exaust Drum es:");
          client.println(TemperaturaExaustDrum);
          client.println("</p>");
          client.println("<p>");
          client.println("la temperatura del asfalto es:");
          client.println(TemperaturadeAsfalto);
          client.println("</p>");
          
          
          client.println("<p>");
          client.println("la posicion del damper de la casa de Bolsas solicitada por el usuario es: ");
          client.println(PosicionDamperBHSolicitado);
          client.println("</p>");
          client.println("<p>");
          client.println("la posicion del damper del quemador solicitado es: ");
          client.println(PosicionDamperQuemadorSolicitado);
          client.println("</p>");
          
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


// INTERRUPCIÓN por puerto SPI

 ISR (SPI_STC_vect)
  {
  byte c = SPDR;
  BufSPI[posSPI]=c;
   
  if (BufSPI[0] == '%')
  {
   if (BufSPI[posSPI]!='\0') //elimina posiciones vaciós en el arreglo recibido
   {posSPI++;}
   if (BufSPI[posSPI]=='@') // caracter que envía el Esclavo1(maestroSPI) para finalizar la comunicación
   {LongBuf=posSPI;
   posSPI=0;}


if (BufSPI[1]=='1')
    {command=1;}
    else
      {
      if (BufSPI[1]=='2')
      {command=2;}
      else
        {
        if (BufSPI[1]=='3')
        {command=3;}
        else
        {
          if (BufSPI[1]=='4')
          {command=4;}
          else
          {
            if (BufSPI[1]=='5')
            {command=5;}
            else
            {
              if (BufSPI[1]=='6')
              {command=6;}
              else
              {command = 0;}
            }
          }
         }
        }  
       }
  



  switch (command)
    {
    // no command? then this is the command
    case 1:
      SPDR =PressBH[posSPI] ;
      break;

    // add to incoming byte, return result
    case 2:
      SPDR = TempeBH[posSPI];  // add 15
      break;

    // subtract from incoming byte, return result
     case 3:
      SPDR = TempeMz[posSPI];  // subtract 8
      break;
     case 4:
      SPDR = TempeAs[posSPI];  // subtract 8
      break;
     case 5:
      SPDR = PosDamp[posSPI];  // subtract 8
      break;
     case 6:
      SPDR = PosQuem[posSPI];  // subtract 8
      break;

    } // end of switch
     recibidoSPI=true;   
 
  }  // end of interrupt service routine (ISR) SPI_STC_vect
  }

  /*
void leer_esclavoSPI()
  {
    for (int i = 1; i<=LongBuf; i++)
    {BufSPI2[i-1]=BufSPI[i];}
   if (recibidoSPI)
   {
    
      for (int pos2 = 0; pos2<=LongBuf ; pos2 ++)
      {
        Serial.print (char (BufSPI[pos2]));
        Serial.print(pos2);
      }
      Serial.println();
      
      BufSPIrx=String(BufSPI2);
      Serial.println(BufSPIrx);
      if (BufSPIrx=="PressBH@")
      {Serial.print("es igual:"); Serial.print(PressBH[0]);Serial.print(PressBH[6]);}    }
    recibidoSPI = false;
    posSPI=0;
    Serial.print("command: ");
  Serial.print(command);
  Serial.print(" pressBH: ");
  Serial.print(PressBH);
  Serial.print(" TempeBH: ");
  Serial.println(TempeBH);
  
    
  }
*/
void ActualizarVariadores()


{
  posDatoLeido = 0;

         while (posDatoLeido <=50)
          {
         if (DatoLeido2[posDatoLeido]=='c')
          {
          if (DatoLeido2[posDatoLeido-5]=='b')
          VelocidadAgregado1Hz= (float)((((int(DatoLeido2[posDatoLeido-4]-48)))*1000)+(((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-4]=='b')
          VelocidadAgregado1Hz= (float)((((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-3]=='b')
          VelocidadAgregado1Hz= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='b')
          VelocidadAgregado1Hz= (float)( (int (DatoLeido2[posDatoLeido-1]-48))) ;
          } 
       
        if (DatoLeido2[posDatoLeido]=='x')
          {
          if (DatoLeido2[posDatoLeido-5]=='c')
          VelocidadAgregado2Hz= (float)((((int(DatoLeido2[posDatoLeido-4]-48)))*1000)+(((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-4]=='c')
          VelocidadAgregado2Hz= (float)((((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-3]=='c')
          VelocidadAgregado2Hz= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='c')
          VelocidadAgregado2Hz= (float)( (int (DatoLeido2[posDatoLeido-1]-48))) ;
          } 
       
        if (DatoLeido2[posDatoLeido]=='e')
          {
          if (DatoLeido2[posDatoLeido-5]=='x')
          VelocidadAgregado3Hz= (float)((((int(DatoLeido2[posDatoLeido-4]-48)))*1000)+(((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-4]=='x')
          VelocidadAgregado3Hz= (float)((((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-3]=='x')
          VelocidadAgregado3Hz= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='x')
          VelocidadAgregado3Hz= (float)( (int (DatoLeido2[posDatoLeido-1]-48))) ;
          } 
       
        if (DatoLeido2[posDatoLeido]=='f')
          
          {
          if (DatoLeido2[posDatoLeido-5]=='e')
          VelocidadAgregado4Hz= (float)((((int(DatoLeido2[posDatoLeido-4]-48)))*1000)+(((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-4]=='e')
          VelocidadAgregado4Hz= (float)((((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-3]=='e')
          VelocidadAgregado4Hz= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='e')
          VelocidadAgregado4Hz= (float)( (int (DatoLeido2[posDatoLeido-1]-48))) ;
          } 
       
         if (DatoLeido2[posDatoLeido]=='g')
          {
          if (DatoLeido2[posDatoLeido-5]=='f')
          VelocidadAgregadoRecicladoHz= (float)((((int(DatoLeido2[posDatoLeido-4]-48)))*1000)+(((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48)));
          if (DatoLeido2[posDatoLeido-4]=='f')
          VelocidadAgregadoRecicladoHz= (float)((((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-3]=='f')
          VelocidadAgregadoRecicladoHz= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='f')
          VelocidadAgregadoRecicladoHz= (float)( (int (DatoLeido2[posDatoLeido-1]-48))) ;
          } 
       
        if (DatoLeido2[posDatoLeido]=='h')
          {
          if (DatoLeido2[posDatoLeido-5]=='g')
          VelocidadAsfaltoHz= (float)((((int(DatoLeido2[posDatoLeido-4]-48)))*1000)+(((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-4]=='g')
          VelocidadAsfaltoHz= (float)((((int(DatoLeido2[posDatoLeido-3]-48)))*100) + ( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-3]=='g')
          VelocidadAsfaltoHz= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='g')
          VelocidadAsfaltoHz= (float)( (int (DatoLeido2[posDatoLeido-1]-48))) ;
          }
        
          if (DatoLeido2[posDatoLeido]=='i')
          {
          if (DatoLeido2[posDatoLeido-3]=='h')
          PosicionDamperBHSolicitado= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='h')
          PosicionDamperBHSolicitado= (float) (int (DatoLeido2[posDatoLeido-1]-48)) ;
          }

        if (DatoLeido2[posDatoLeido]=='j')
          {
          if (DatoLeido2[posDatoLeido-3]=='i')
          PosicionDamperQuemadorSolicitado= (float) (( (int (DatoLeido2[posDatoLeido-2]-48))*10) + (int (DatoLeido2[posDatoLeido-1]-48))) ;
          if (DatoLeido2[posDatoLeido-2]=='i')
          PosicionDamperQuemadorSolicitado= (float) (int (DatoLeido2[posDatoLeido-1]-48)) ;
          }

          
          
          posDatoLeido++;  
        }   
      }

  void AplicarPWM()

  {/*
    analogWrite(2,40);
    analogWrite(3,80);
    analogWrite(4,120);
    analogWrite(5,160);
    analogWrite(6,200);
    analogWrite(7,240);
     */
    analogWrite(2,int (VelocidadAgregado1Hz*0.425)); // la velocidadAgregado1Hz vendrá de 0 a 600
    analogWrite(3,int(VelocidadAgregado2Hz*0.425)); // simulando la frecuencia de 0 a 60.00 
    analogWrite(4,int(VelocidadAgregado3Hz*0.425));
    //analogWrite(5,int(VelocidadAgregado4Hz*0.425));
    analogWrite(6,int(VelocidadAgregadoRecicladoHz*0.425));
    analogWrite(5,int(VelocidadAsfaltoHz*0.425));
    
    
    }

 void ActualizarMotores()

  {
    for (int u = 0; u<=40; u++)
    {
      if ((DatoLeido2[u-4])== 'k')
        {
          EstadoIncDamperQuemador  = int (DatoLeido2[u-3]-48);
          EstadoDecDamperQuemador  = int (DatoLeido2[u-2]-48);
          EstadoOpenDamperBH = int (DatoLeido2[u-1]-48);
          EstadoCloseDamperBH = int (DatoLeido2[u]-48);
          Serial.println("entro a subrutina");
        }
      }

      if (EstadoIncDamperQuemador == 1)
      {
      digitalWrite (SalidaIncDamperQuemador, LOW);
      Serial.println("Damper 0 quemador");}
      else
      {
        if (EstadoIncDamperQuemador == 0)
        {
          digitalWrite (SalidaIncDamperQuemador, HIGH);
          Serial.println("Damper 1 quemador");
        }
      }
      
      if (EstadoDecDamperQuemador == 1)
      digitalWrite (SalidaDecDamperQuemador, LOW);
      else
      {
      if (EstadoDecDamperQuemador == 0)
      digitalWrite (SalidaDecDamperQuemador, HIGH);
      }

      if (EstadoOpenDamperBH == 1)
      digitalWrite (SalidaOpenDamperBH, LOW);
      else
      {
      if (EstadoOpenDamperBH == 0)
      digitalWrite (SalidaOpenDamperBH, HIGH);
      }
      
      if (EstadoCloseDamperBH == 1)
      digitalWrite (SalidaCloseDamperBH, LOW);
      else
      {
        if (EstadoCloseDamperBH==0)
        digitalWrite (SalidaCloseDamperBH, HIGH);
      }

  }

  void moverDamperes(){
    if (PosicionDamperBH<=(PosicionDamperBHSolicitado-5))  //una histéresis de 2 para darle tiempo
       digitalWrite(SalidaOpenDamperBH, LOW);  //activar relevo para abrir damper 
    else
       digitalWrite(SalidaOpenDamperBH, HIGH);  // desactivar relevo para abrir damper
    if (PosicionDamperBH>=(PosicionDamperBHSolicitado+5))  //una histéresis de 2 para darle tiempo
       digitalWrite(SalidaCloseDamperBH, LOW);  //activar relevo para cerrar damper 
    else
       digitalWrite(SalidaCloseDamperBH, HIGH);  // desactivar relevo para abrir damper
       


    if (PosicionValvulaCombustible<=(PosicionDamperQuemadorSolicitado-5))  //una histéresis de 2 para darle tiempo
       digitalWrite(SalidaIncDamperQuemador, LOW);  //activar relevo para abrir damper 
      else
       digitalWrite(SalidaIncDamperQuemador, HIGH);  // desactivar relevo para abrir damper
      if (PosicionValvulaCombustible>=(PosicionDamperQuemadorSolicitado+5))  //una histéresis de 2 para darle tiempo
       digitalWrite(SalidaDecDamperQuemador, LOW);  //activar relevo para cerrar damper 
      else
       digitalWrite(SalidaDecDamperQuemador, HIGH);  // desactivar relevo para abrir damper
    
      }
  
 

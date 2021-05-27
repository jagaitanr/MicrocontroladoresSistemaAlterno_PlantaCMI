
//Librerías
#include <LiquidCrystal_I2C.h>
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

byte mac[] = {   //configuración mac tarjeta shield internet
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(169, 254, 215, 200); //asignacion de una IP, 

unsigned int localPort = 8888;      // asignación de un puerto

// buffers for receiving and sending data
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char packetBuffer[100];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetServer server(80);


LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

        int CursorVertical = 0;

/* valores solicitados por el programa*/
        int BandaAlimentadoraEntrada =0;
        int ZarandaEntrada =0;
        int BandaColectoraEntrada =0;
        int PesoPruebaBandaAlimentadoraEntrada =0;
        int BandaAgregados1Entrada =0;
        int BandaAgregados2Entrada =0;
        int BandaAgregados3Entrada =0;
        int BandaAgregados4Entrada =0;
        int BandaInclinadaRecicladoEntrada =0;
        int BandaAgregadoRecicladoEntrada =0;
        int MotorTrituradoRecicladoEntrada =0;
        int PesoPruebaBandaRecicladoEntrada =0;
        int CompuertaRecicladoEntrada =0;
        int MotorBlowerBHEntrada =0;
        int MotorVannerBHEntrada =0;
        int SinFinColector1BHEntrada =0;
        int SinFinColector2BHEntrada =0;
        int SinFinTransversalBHEntrada =0;
        int CompresorEntrada =0;
        int BombaAsfaltoAdelanteEntrada =0;
        int BombaAsfaltoAtrasEntrada =0;
        int Extractor1BHEntrada =0;
        int Extractor2BHEntrada =0;
        int BlowerQuemadorEntrada =0;
        int ElevadorEntrada =0;
        int CompuertaRechazoEntrada =0;
        int SecadorEntrada =0;
        int SlingerAdelanteEntrada =0;
        int SlingerAtrasEntrada =0;
        int BombaCombustibleEntrada =0;
        int DamperEntrada =0;
        int PilotoValvulaEntrada =0;
        int MainLlamaEntrada =0;
        int Rotor1BHEntrada =0;
        int Rotor2BHEntrada =0;
        int Rotor3BHEntrada =0;
        int AlimentacionSiloEntrada =0;
        int ValvulaAceiteEntrada =0;
        int CompuertaGansoEntrada =0;
        int ValvulaGasEntrada =0;
        int OpenDamperBHEntrada = 0;
        int CloseDamperBHEntrada = 0;
        int IncrementarDampQuemadorEntrada = 0;
        int DecrementarDampQuemadorEntrada = 0;
        int RecycleBypassEntrada = 0;
        int R8090ControlLLamaEntrada =0;
        int IncDamperQuemador = 0; //estado recibido del maestro pero solo para prender la electroválvula no hay retroalimentación
        int DecDamperQuemador = 0;
        int OpenDamperBH = 0;
        int CloseDamperBH = 0;
        double PresionCasadeBolsas=0;
        double TemperaturaCasadeBolsas=0;
        double TemperaturaMezcla=0;
        double PosicionDamperQuemador = 0;
        double PosicionDamperExtractorBH = 0;
        double VelocidadAgregado1Hz = 0;
        double VelocidadAgregado2Hz = 0;
        double VelocidadAgregado3Hz = 0;
        double VelocidadAgregado4Hz = 0;
        double VelocidadAgregadoRecicladoHz = 0;
        double VelocidadAsfaltoHz = 0;
        

  

 /*valores entregados por el microcontrolador Esclavo1 sobre el estado
 de los motores*/
        int BandaAlimentadoraSalida =0;
        int ZarandaSalida =0;
        int BandaColectoraSalida =0;
        int PesoPruebaBandaAlimentadoraSalida =0;
        int BandaAgregados1Salida =0;
        int BandaAgregados2Salida =0;
        int BandaAgregados3Salida =0;
        int BandaAgregados4Salida =0;
        int BandaInclinadaRecicladoSalida =0;
        int BandaAgregadoRecicladoSalida =0;
        int MotorTrituradoRecicladoSalida =0;
        int PesoPruebaBandaRecicladoSalida =0;
        int CompuertaRecicladoSalida =0;
        int MotorBlowerBHSalida =0;
        int MotorVannerBHSalida =0;
        int SinFinColector1BHSalida =0;
        int SinFinColector2BHSalida =0;
        int SinFinTransversalBHSalida =0;
        int CompresorSalida =0;
        int BombaAsfaltoAdelanteSalida =0;
        int BombaAsfaltoAtrasSalida =0;
        int Extractor1BHSalida =0;
        int Extractor2BHSalida =0;
        int BlowerQuemadorSalida =0;
        int ElevadorSalida =0;
        int CompuertaRechazoSalida =0;
        int SecadorSalida =0;
        int SlingerAdelanteSalida =0;
        int SlingerAtrasSalida =0;
        int BombaCombustibleSalida =0;
        int DamperSalida =0;
        int PilotoValvulaSalida =0;
        int MainLlamaSalida =0;
        int Rotor1BHSalida =0;
        int Rotor2BHSalida =0;
        int Rotor3BHSalida =0;
        int AlimentacionSiloSalida =0;
        int ValvulaAceiteSalida =0;
        int CompuertaGansoSalida =0;
        int ValvulaGasSalida =0;
        int R8090ControlLLamaSalida =0;
        int BytesEnBuffer=0;
        int tiempo = 0;
        int tiempo1 = 0;

        int lectura=0;
        char Lectura2[4];
        char Lectura=" ";
        char Lectura1 ="";
        int posDatoLeido=0;
        int posDatoLeido2=0;
        char DatoLeido[100]; //valor recibido por puerto serie
        char DatoLeido1[100]; //valor asignado para prender y apagar motores
        char DatoLeido2[100]; // valor asignado para variadores
        int DatoLeido3[100];
        char DatoLeido4[100];
        char DatoLeido5[100];
        char DatoLeido6[100];
        char Dato_a_enviar[100];
        char DatoaEsclavo1[100];  // Dato enviado a Esclavo1
        int  DatoaEsclavo3[100];
        char Dato_de_Esclavo[100];  // Dato recibido por esclavo1
        char DatoAnalogo_a_enviar[100];
        int a;//valor del dato que viene en char y se pasa a entero
        int b=0;
        

  

 
int xPin = A14;
int yPin = A15;
int FlameEye = A0;   //señal análoga de intensidad de llama
int IndAgregados = A1; // señal análoga proveniente del indicador de agregados
int IndAsfalto = A2; // señal análoga proveniente del indicador de asfalto
int IndReciclado = A3;// señal análoga proveniente del indicador de reciclado
int Valor_FlameEye = 0;
int Valor_IndAgregados = 0;  // variable donde se guardará el valor de 0 a 1023 del indicador de agregados
int Valor_IndAsfalto = 0;  // variable donde se guardará el valor de 0 a 1023 del indicador de asfalto
int Valor_IndReciclado = 0;  // variable donde se guardará el valor de 0 a 1023 del sensor de la llama
int FreqAgregado1 = 0; // frecuencia recibida del maestro para variador agregado1

int buttonPin = 53;

int xPosition = 0;
int yPosition = 0;
int buttonState = 0;
bool cambiar_dato=false;

const int EnTxPin = 2;// High: Tx y Low Rx

void setup() 
{ 
  
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(EnTxPin, OUTPUT);

  //activar resistencia pull-up en el pin pulsador 
  pinMode(buttonPin, INPUT_PULLUP); 
  lcd.begin(16,2); //inicializar LCD
  Serial2.begin(9600);  //comunicación rs485 con Arduino Mega1
  
  Serial1.begin(9600);  //comunicación rs485 con Arduino Mega1
  Serial1.setTimeout(100); //tiempo de espera de 100ms
  digitalWrite (EnTxPin, LOW);
  
  // Para las versiones anteriores a 1.0.1 Arduino 
   pinMode (buttonPin, INPUT); 
   digitalWrite (buttonPin, HIGH);
  #define SERIAL_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256
  #define SERIAL1_BUFFER_SIZE 256 //aumentar la capacidad del buffer a 256

DatoaEsclavo1[0]= '0';
DatoaEsclavo1[1]= '2';
DatoaEsclavo1[2]= '1';
DatoaEsclavo1[3]= '1';
DatoaEsclavo1[4]= '1';
DatoaEsclavo1[5]= '1';
DatoaEsclavo1[6]= '1';
DatoaEsclavo1[7]= '1';

// start the Ethernet and UDP:
  Ethernet.begin(mac, ip);
  server.begin();
  Udp.begin(localPort);
  Serial.begin(9600);
  }


void loop2 ()
{
  Serial2.print("numero1: ");
  Serial2.flush();
  Serial2.write (49);
  Serial2.flush();
  
  }

void loop()
  {
    digitalWrite (EnTxPin, HIGH);
    LecturaEsclavo1(); //envia una 'e' para luego recibir el estado de motores
    EnvioEsclavo1(); //envía al microcontrolador Esclavo1 el estado que requiere poner en los motores
    //RevisarSolicitudProgramaPrincipal();
    ActualizarMotores();
    ActualizarVariadores();
    EnvioEstadoMotoresProgramaPrincipal();// se aprovecha para enviar
    EnvioAnalogasProgramaPrincipal();
        
    delay(50);
    LecturaJoystick();
    VisualizarLCD();
    ModificarAnalogas();
    //Serial.println(Dato_de_Esclavo);
   // Serial.print("→");
   // Serial.println(DatoLeido2);
    LecturaAnalogas();  //lee los puertos análogos 
   EnvioEsclavo1Variadores(); // envía el dato de las frecuencias de cada variador
     
//Serial.println("feqAgg1");
  //Serial.println(FreqAgregado1);
      
      //if (tiempo == 1)
  /*  {tiempo1=micros();
    tiempo = 1;}
  */
  
    
    
}

////////////***RevisarSolicitudProgramaPrincipal*********\\\\\\\\\\\\\\\\\
//revisa si ha pedido un dato para enviárselo

void RevisarSolicitudProgramaPrincipal()

  {
    if (DatoLeido[0]=='d' && DatoLeido[1]=='a' && DatoLeido[2]=='t' && DatoLeido[3]=='o' && DatoLeido[4]=='2' ) //verifica información de entrada para hacer cambios
    {
    //  Serial.print ("enviando a programaPrincipal");
    ActualizarMotores(); // después de leer en la interrupción del puerto serie
     // y verificar "dato2"  actualizar para modificar los 
    // los motores y enviar a Esclavo2 
    
    
    EnvioEstadoMotoresProgramaPrincipal();// se aprovecha para enviar
    //el estado de los motores
    }
    else
    {
      if (DatoLeido[0] == 'b')
      ActualizarVariadores();
      EnvioAnalogasProgramaPrincipal();
    
    }
    DatoLeido[0]= '0';
      }


/////////////////**********Lectura Interfaz*******************\\\\\\\\\\\\\\\\\\\\\

//void LecturaTipo2() //Recibe los datos de visual para
//poner determinado motor en on o off

void serialEvent()
{
   posDatoLeido=0;
   posDatoLeido2=0;
   Lectura = '-';
   while (Serial.available()>0 )
      {
        Lectura=Serial.read();
        DatoLeido[posDatoLeido]=Lectura;
        DatoaEsclavo1[posDatoLeido]=Lectura;
        if (DatoLeido[0] == 'b')  // solo si Lectura[0] = 'b' entonces actualiza DatoLeido2 que es para los variadores
        {
          DatoLeido2[posDatoLeido]= Lectura;
        
        }
        else
        {
        if (DatoLeido[0] == 'd'&& DatoLeido[1] == 'a' && DatoLeido[2] == 't'&& DatoLeido[3] == 'o'&& DatoLeido[4] == '2')
        {
          DatoLeido1[0] = 'd';
          DatoLeido1[1] = 'a';
          DatoLeido1[2] = 't';
          DatoLeido1[3] = 'o';
          DatoLeido1[4] = '2';
          DatoLeido1[posDatoLeido]=Lectura; // si es diferente de 'b' se refiere al dato principal para actualizar motores
          }
          }
          posDatoLeido++;
      
        
        }
      }
        
     void ActualizarMotores()

        {
          
        BandaAlimentadoraEntrada  = int (DatoLeido1[5]-48);                // ya que DatoLeido1[4] equivalente a 2 será para las analogas
        ZarandaEntrada  = int (DatoLeido1[6]-48);
        BandaColectoraEntrada  = int (DatoLeido1[7]-48);
        PesoPruebaBandaAlimentadoraEntrada  = int (DatoLeido1[8]-48);
        BandaAgregados1Entrada  = int (DatoLeido1[9]-48);
        BandaAgregados2Entrada  = int (DatoLeido1[10]-48);
        BandaAgregados3Entrada  = int (DatoLeido1[11]-48);
        BandaAgregados4Entrada  = int (DatoLeido1[12]-48);
        BandaInclinadaRecicladoEntrada  = int (DatoLeido1[13]-48);
        BandaAgregadoRecicladoEntrada  = int (DatoLeido1[14]-48);
        MotorTrituradoRecicladoEntrada  = int (DatoLeido1[15]-48);
        PesoPruebaBandaRecicladoEntrada  = int (DatoLeido1[16]-48);
        CompuertaRecicladoEntrada  = int (DatoLeido1[17]-48);
        MotorBlowerBHEntrada  = int (DatoLeido1[18]-48);
        MotorVannerBHEntrada  = int (DatoLeido1[19]-48);
        SinFinColector1BHEntrada  = int (DatoLeido1[20]-48);
        SinFinColector2BHEntrada  = int (DatoLeido1[21]-48);
        SinFinTransversalBHEntrada  = int (DatoLeido1[22]-48);
        CompresorEntrada  = int (DatoLeido1[23]-48);
         BombaAsfaltoAdelanteEntrada = int (DatoLeido1[24]-48);
         BombaAsfaltoAtrasEntrada =  int (DatoLeido1[25]-48);
         Extractor1BHEntrada =  int (DatoLeido1[26]-48);
         Extractor2BHEntrada =  int (DatoLeido1[27]-48);
         BlowerQuemadorEntrada =  int (DatoLeido1[28]-48);
         ElevadorEntrada =  int (DatoLeido1[29]-48);
         CompuertaRechazoEntrada =  int (DatoLeido1[30]-48);
         SecadorEntrada =  int (DatoLeido1[31]-48);
         SlingerAdelanteEntrada =  int (DatoLeido1[32]-48);
         SlingerAtrasEntrada =  int (DatoLeido1[33]-48);
         BombaCombustibleEntrada =  int (DatoLeido1[34]-48);
         DamperEntrada =  int (DatoLeido1[35]-48);
         PilotoValvulaEntrada =  int (DatoLeido1[36]-48);
         MainLlamaEntrada =  int (DatoLeido1[37]-48);
         Rotor1BHEntrada =  int (DatoLeido1[38]-48);
         Rotor2BHEntrada =  int (DatoLeido1[39]-48);
         Rotor3BHEntrada =  int (DatoLeido1[40]-48);
         AlimentacionSiloEntrada =  int (DatoLeido1[41]-48);
         ValvulaAceiteEntrada =  int (DatoLeido1[42]-48);
         CompuertaGansoEntrada = int (DatoLeido1[43]-48);
         ValvulaGasEntrada =  int (DatoLeido1[44]-48);
         OpenDamperBHEntrada = int (DatoLeido1[45]-48);
         CloseDamperBHEntrada = int (DatoLeido1[46]-48);
         IncrementarDampQuemadorEntrada = int (DatoLeido1[47]-48);
         DecrementarDampQuemadorEntrada = int (DatoLeido1[48]-48);
         RecycleBypassEntrada = int (DatoLeido1[49]-48);
        
       }
      void EnvioEstadoMotoresProgramaPrincipal()
      {
        Dato_a_enviar[0]= 'd';
        Dato_a_enviar[1]= 'a';
        Dato_a_enviar[2]= 't';
        Dato_a_enviar[3]= 'o';
        Dato_a_enviar[4]= '2';// envío de señales digitales, estado de motores a programa Principal
        Dato_a_enviar[5] = char (BandaAlimentadoraSalida+48);
        Dato_a_enviar[6] = char (ZarandaSalida+48);
        Dato_a_enviar[7] = char (BandaColectoraSalida+48);
        Dato_a_enviar[8] = char (PesoPruebaBandaAlimentadoraSalida+48);
        Dato_a_enviar[9] = char (BandaAgregados1Salida+48);
        Dato_a_enviar[10] = char (BandaAgregados2Salida+48);
        Dato_a_enviar[11] = char (BandaAgregados3Salida+48);
        Dato_a_enviar[12] = char (BandaAgregados4Salida+48);
        Dato_a_enviar[13] = char (BandaInclinadaRecicladoSalida+48);
        Dato_a_enviar[14] = char (BandaAgregadoRecicladoSalida+48);
        Dato_a_enviar[15] = char (MotorTrituradoRecicladoSalida+48);
        Dato_a_enviar[16] = char (PesoPruebaBandaRecicladoSalida+48);
        Dato_a_enviar[17] = char (CompuertaRecicladoSalida+48);
        Dato_a_enviar[18] = char (MotorBlowerBHSalida+48);
        Dato_a_enviar[19] = char (MotorVannerBHSalida+48);
                                               Dato_a_enviar[20] = char (SinFinColector1BHSalida+48);
        Dato_a_enviar[21] = char (SinFinColector2BHSalida+48);
        Dato_a_enviar[22] = char (SinFinTransversalBHSalida+48);
        Dato_a_enviar[23] = char (CompresorSalida+48);
        Dato_a_enviar[24] = char (BombaAsfaltoAdelanteSalida+48);
        Dato_a_enviar[25] = char (BombaAsfaltoAtrasSalida+48);
        Dato_a_enviar[26] = char (Extractor1BHSalida+48);
        Dato_a_enviar[27] = char (Extractor2BHSalida+48);
        Dato_a_enviar[28] = char (BlowerQuemadorSalida+48);
        Dato_a_enviar[29] = char (ElevadorSalida+48);
        Dato_a_enviar[30] = char (CompuertaRechazoSalida+48);
        Dato_a_enviar[31] = char (SecadorSalida+48);
        Dato_a_enviar[32] = char (SlingerAdelanteSalida+48);
        Dato_a_enviar[33] = char (SlingerAtrasSalida+48);
        Dato_a_enviar[34] = char (BombaCombustibleSalida+48);
        Dato_a_enviar[35] = char (DamperSalida+48);
        Dato_a_enviar[36] = char (PilotoValvulaSalida+48);
        Dato_a_enviar[37] = char (MainLlamaSalida+48);
        Dato_a_enviar[38] = char (Rotor1BHSalida+48);
        Dato_a_enviar[39] = char (Rotor2BHSalida+48);
        Dato_a_enviar[40] = char (Rotor3BHSalida+48);
        Dato_a_enviar[41] = char (AlimentacionSiloSalida+48);
        Dato_a_enviar[42] = char (ValvulaAceiteSalida+48);
        Dato_a_enviar[43] = char (CompuertaGansoSalida+48);
        Dato_a_enviar[44] = char (ValvulaGasSalida+48);
        Dato_a_enviar[45] = char (R8090ControlLLamaSalida+48);
        Serial.println(Dato_a_enviar);
        
        }
  
    void ActualizarVariadores()

        {
          posDatoLeido = 0;

         while (posDatoLeido <=30)
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
          posDatoLeido++;  
        }   
      }
     
//////////////////************Lectura Esclavo 1 Arduino entradas del estado de los motores**************\\\\\\\\\\\\\\\\\

/////////////////**********Lectura Interfaz*******************\\\\\\\\\\\\\\\\\\\\\

void LecturaEsclavo1()
    
    {int packetSize = Udp.parsePacket();
        if (packetSize) 
        {     Serial.print("Received packet of size ");
              Serial.println(packetSize);
              Serial.print("From ");
              IPAddress remote = Udp.remoteIP();
              for (int i = 0; i < 4; i++)
              {
              Serial.print(remote[i], DEC);
              if (i < 3) 
              {
              Serial.print(".");
              }
        }
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      // read the packet into packetBufffer
      Udp.read(packetBuffer, 100);
      Serial.println("Contents:");
      Serial.println(packetBuffer);
      for (int i=0;i<=100;i++)
      {Dato_de_Esclavo[i]=packetBuffer[i];} 
      /*
      digitalWrite(EnTxPin, LOW); //módulo RS485 en recepción
      delay(200); //espera 10 segundos para recibir el dato del esclavo1
      digitalWrite(EnTxPin, HIGH); //módulo RS485 en transmisión
      
   /* digitalWrite(EnTxPin, HIGH); //módulo RS485 en transmisión
    Serial1.print('a');
    Serial1.flush();
    
    delay (10);
    */
      if (Dato_de_Esclavo[0]=='E' && Dato_de_Esclavo[1]=='1' && Dato_de_Esclavo[2]=='M' && Dato_de_Esclavo[3]=='1' && Dato_de_Esclavo[4]=='2' ) //verifica información de entrada para hacer cambios
        {                                                                  //el DatoLeido2[4] equivalente a 2 es el primer envio de señales digitales
        BandaAlimentadoraSalida  = int (Dato_de_Esclavo[5]-48);                // ya que Dato_de_Esclavo[4] equivalente a 3 será para las analogas
        ZarandaSalida  = int (Dato_de_Esclavo[6]-48);
        BandaColectoraSalida  = int (Dato_de_Esclavo[7]-48);
        PesoPruebaBandaAlimentadoraSalida  = int (Dato_de_Esclavo[8]-48);
        BandaAgregados1Salida  = int (Dato_de_Esclavo[9]-48);
        BandaAgregados2Salida  = int (Dato_de_Esclavo[10]-48);
        BandaAgregados3Salida  = int (Dato_de_Esclavo[11]-48);
        BandaAgregados4Salida  = int (Dato_de_Esclavo[12]-48);
        BandaInclinadaRecicladoSalida  = int (Dato_de_Esclavo[13]-48);
        BandaAgregadoRecicladoSalida  = int (Dato_de_Esclavo[14]-48);
        MotorTrituradoRecicladoSalida  = int (Dato_de_Esclavo[15]-48);
        PesoPruebaBandaRecicladoSalida  = int (Dato_de_Esclavo[16]-48);
        CompuertaRecicladoSalida  = int (Dato_de_Esclavo[17]-48);
        MotorBlowerBHSalida  = int (Dato_de_Esclavo[18]-48);
        MotorVannerBHSalida  = int (Dato_de_Esclavo[19]-48);
        SinFinColector1BHSalida  = int (Dato_de_Esclavo[20]-48);
        SinFinColector2BHSalida  = int (Dato_de_Esclavo[21]-48);
        SinFinTransversalBHSalida  = int (Dato_de_Esclavo[22]-48);
        CompresorSalida  = int (Dato_de_Esclavo[23]-48);
         BombaAsfaltoAdelanteSalida = int (Dato_de_Esclavo[24]-48);
         BombaAsfaltoAtrasSalida =  int (Dato_de_Esclavo[25]-48);
         Extractor1BHSalida =  int (Dato_de_Esclavo[26]-48);
         Extractor2BHSalida =  int (Dato_de_Esclavo[27]-48);
         BlowerQuemadorSalida =  int (Dato_de_Esclavo[28]-48);
         ElevadorSalida =  int (Dato_de_Esclavo[29]-48);
         CompuertaRechazoSalida =  int (Dato_de_Esclavo[30]-48);
         SecadorSalida =  int (Dato_de_Esclavo[31]-48);
         SlingerAdelanteSalida =  int (Dato_de_Esclavo[32]-48);
         SlingerAtrasSalida =  int (Dato_de_Esclavo[33]-48);
         BombaCombustibleSalida =  int (Dato_de_Esclavo[34]-48);
         DamperSalida =  int (Dato_de_Esclavo[35]-48);
         PilotoValvulaSalida =  int (Dato_de_Esclavo[36]-48);
         MainLlamaSalida =  int (Dato_de_Esclavo[37]-48);
         Rotor1BHSalida =  int (Dato_de_Esclavo[38]-48);
         Rotor2BHSalida =  int (Dato_de_Esclavo[39]-48);
         Rotor3BHSalida =  int (Dato_de_Esclavo[40]-48);
         AlimentacionSiloSalida =  int (Dato_de_Esclavo[41]-48);
         ValvulaAceiteSalida =  int (Dato_de_Esclavo[42]-48);
         CompuertaGansoSalida =  int (Dato_de_Esclavo[43]-48);
         ValvulaGasSalida =  int (Dato_de_Esclavo[44]-48);
         }

    }

    /*void serialEvent1()
      {
        posDatoLeido=0; 
        delay(10); //allows all serial sent to be received together
        while(Serial1.available()>0 )
        {
          lectura = Serial1.read();
          Dato_de_Esclavo[posDatoLeido]=lectura;
          posDatoLeido++;
          
        }
        
       // Serial.print("Dato_de_Esclavo: ");
       // Serial.println(Dato_de_Esclavo);
      }  
  
 */ 
    }
      
 void EnvioEsclavo1()  //datos que deben ser enviados hasta Esclavo 2
  {                     //pasando primero por esclavo 1 para modificar
                        //el estado de los motores
    /*Serial1.flush();
    digitalWrite(EnTxPin, LOW); //módulo RS485 en recepción
    posDatoLeido = 0;
   */ delay(10);
    
   /*
    
    DatoaEsclavo1[0]= 'M';
    DatoaEsclavo1[1]= 'E';
    DatoaEsclavo1[2]= '1';
    DatoaEsclavo1[3]= char ( BandaAlimentadoraEntrada-48);
    DatoaEsclavo1[4]= char (ZarandaEntrada-48);
    DatoaEsclavo1[5]= char ( BandaColectoraEntrada-48);
    DatoaEsclavo1[6]= char ( PesoPruebaBandaAlimentadoraEntrada-48);
    DatoaEsclavo1[7]= char ( BandaAgregados1Entrada);
    DatoaEsclavo1[8]= char (  BandaAgregados2Entrada);
    DatoaEsclavo1[9]= char (  BandaAgregados3Entrada);
    DatoaEsclavo1[10]= char (  BandaAgregados4Entrada);
    DatoaEsclavo1[11]= char (  BandaInclinadaRecicladoEntrada+48);
    DatoaEsclavo1[12]= char (  BandaAgregadoRecicladoEntrada+48);
    DatoaEsclavo1[13]= char ( MotorTrituradoRecicladoEntrada+48);
    DatoaEsclavo1[14]= char ( PesoPruebaBandaRecicladoEntrada+48);
    DatoaEsclavo1[15]=  ( CompuertaRecicladoEntrada+48);
    DatoaEsclavo1[16]=  ( MotorBlowerBHEntrada+48);
    DatoaEsclavo1[17]=  ( MotorVannerBHEntrada+48);
    DatoaEsclavo1[18]=  ( SinFinColector1BHEntrada+48);
    DatoaEsclavo1[19]= char ( SinFinColector2BHEntrada+48);
    DatoaEsclavo1[20]= char ( SinFinTransversalBHEntrada+48);
    DatoaEsclavo1[21]= char ( CompresorEntrada+48);
    DatoaEsclavo1[22]= char ( BombaAsfaltoAdelanteEntrada+48);
    DatoaEsclavo1[23]= char ( BombaAsfaltoAtrasEntrada+48);
    DatoaEsclavo1[24]= char ( Extractor1BHEntrada+48);
    DatoaEsclavo1[25]= char ( Extractor2BHEntrada+48);
    DatoaEsclavo1[26]= char ( BlowerQuemadorEntrada+48);
    DatoaEsclavo1[27]= char ( ElevadorEntrada+48);
    DatoaEsclavo1[28]= char ( CompuertaRechazoEntrada+48);
    DatoaEsclavo1[29]= char ( SecadorEntrada+48);
    DatoaEsclavo1[30]= char ( SlingerAdelanteEntrada+48);
    DatoaEsclavo1[31]= char ( SlingerAtrasEntrada+48);
    DatoaEsclavo1[32]= char ( BombaCombustibleEntrada+48);
    DatoaEsclavo1[33]= char ( DamperEntrada+48);
    DatoaEsclavo1[34]= char ( PilotoValvulaEntrada+48);
    DatoaEsclavo1[35]= char ( MainLlamaEntrada+48);
    DatoaEsclavo1[36]= char ( Rotor1BHEntrada+48);
    DatoaEsclavo1[37]= char ( Rotor2BHEntrada+48);
    DatoaEsclavo1[38]= char ( Rotor3BHEntrada+48);
    DatoaEsclavo1[39]= char ( AlimentacionSiloEntrada+48);
    DatoaEsclavo1[40]= char ( ValvulaAceiteEntrada+48);
    DatoaEsclavo1[41]= char ( CompuertaGansoEntrada+48);
    DatoaEsclavo1[42]= char ( ValvulaGasEntrada+48);
    DatoaEsclavo1[43]= char ( OpenDamperBHEntrada + 48);
    DatoaEsclavo1[44]= char ( CloseDamperBHEntrada + 48);
    DatoaEsclavo1[45]= char ( IncrementarDampQuemadorEntrada + 48);
    DatoaEsclavo1[46]= char ( DecrementarDampQuemadorEntrada + 48);
    DatoaEsclavo1[47]= char ( RecycleBypassEntrada + 48);
    
    */
    
    DatoaEsclavo3[0]=   BandaAlimentadoraEntrada;
    DatoaEsclavo3[1]=   ZarandaEntrada ;
    DatoaEsclavo3[2]=    BandaColectoraEntrada ;
    DatoaEsclavo3[3]=    PesoPruebaBandaAlimentadoraEntrada ;
    DatoaEsclavo3[4]=    BandaAgregados1Entrada ;
    DatoaEsclavo3[5]=     BandaAgregados2Entrada ;
    DatoaEsclavo3[6]=     BandaAgregados3Entrada ;
    DatoaEsclavo3[7]=     BandaAgregados4Entrada ;
    DatoaEsclavo3[8]=     BandaInclinadaRecicladoEntrada ;
    DatoaEsclavo3[9]=     BandaAgregadoRecicladoEntrada ;
    DatoaEsclavo3[10]=    MotorTrituradoRecicladoEntrada ;
    DatoaEsclavo3[11]=    PesoPruebaBandaRecicladoEntrada ;
    DatoaEsclavo3[12]=    CompuertaRecicladoEntrada ;
    DatoaEsclavo3[13]=    MotorBlowerBHEntrada ;
    DatoaEsclavo3[14]=    MotorVannerBHEntrada ;
    DatoaEsclavo3[15]=    SinFinColector1BHEntrada ;
    DatoaEsclavo3[16]=    SinFinColector2BHEntrada ;
    DatoaEsclavo3[17]=    SinFinTransversalBHEntrada ;
    DatoaEsclavo3[18]=    CompresorEntrada ;
    DatoaEsclavo3[19]=    BombaAsfaltoAdelanteEntrada ;
    DatoaEsclavo3[20]=    BombaAsfaltoAtrasEntrada ;
    DatoaEsclavo3[21]=    Extractor1BHEntrada ;
    DatoaEsclavo3[22]=    Extractor2BHEntrada ;
    DatoaEsclavo3[23]=    BlowerQuemadorEntrada ;
    DatoaEsclavo3[24]=    ElevadorEntrada ;
    DatoaEsclavo3[25]=    CompuertaRechazoEntrada ;
    DatoaEsclavo3[26]=    SecadorEntrada ;
    DatoaEsclavo3[27]=    SlingerAdelanteEntrada ;
    DatoaEsclavo3[28]=    SlingerAtrasEntrada ;
    DatoaEsclavo3[29]=    BombaCombustibleEntrada ;
    DatoaEsclavo3[30]=    DamperEntrada ;
    DatoaEsclavo3[31]=    PilotoValvulaEntrada ;
    DatoaEsclavo3[32]=    MainLlamaEntrada ;
    DatoaEsclavo3[33]=    Rotor1BHEntrada ;
    DatoaEsclavo3[34]=    Rotor2BHEntrada ;
    DatoaEsclavo3[35]=    Rotor3BHEntrada ;
    DatoaEsclavo3[36]=    AlimentacionSiloEntrada ;
    DatoaEsclavo3[37]=    ValvulaAceiteEntrada ;
    DatoaEsclavo3[38]=    CompuertaGansoEntrada ;
    DatoaEsclavo3[39]=    ValvulaGasEntrada ;
    DatoaEsclavo3[40]=    OpenDamperBHEntrada ;
    DatoaEsclavo3[41]=    CloseDamperBHEntrada  ;
    DatoaEsclavo3[42]=    IncrementarDampQuemadorEntrada ;
    DatoaEsclavo3[43]=    DecrementarDampQuemadorEntrada;
    DatoaEsclavo3[44]=    RecycleBypassEntrada;
    
    
    //Serial.print("ME1");
    //Serial.flush();
    //Serial.print(DatoaEsclavo1);//dato recibido del PC para modificar motores en Esclavo2
    //Serial.flush();
    //delay (10);
    Serial1.print("ME1");
    for (a= 0; a<=44; a++)
    {
      Serial1.print(DatoaEsclavo3[a]); //dato recibido del PC para modificar motores en Esclavo2
      Serial1.flush();
     }

       // send a reply to the IP address and port that sent us the packet we received
  IPAddress RemIP(169, 254, 215, 100);
  unsigned int RemPort = 7124;
   Udp.beginPacket(RemIP, RemPort);
   Udp.write("ME1");
   Udp.write(DatoaEsclavo1);
   Udp.endPacket();
   delay(400); 
  delay(10);
  //
    } 

  void EnvioEsclavo1Variadores()
  
  {
  digitalWrite(EnTxPin, HIGH);
  Serial1.print(DatoLeido2);
  Serial1.flush();
  digitalWrite(EnTxPin, LOW);
  
  }

    
////////////******************Lectura del Joystick*************\\\\\\\\\\\\\\\\\\\

void ModificarAnalogas()
 {
  if (CursorVertical==41)
  {
    if (xPosition<=300)
    {
      PresionCasadeBolsas=PresionCasadeBolsas+0.1;
      }
    if (xPosition>=600)
    {
      PresionCasadeBolsas=PresionCasadeBolsas-0.1;
      }
     
    }

    if (CursorVertical==42)
  {
    if (xPosition<=300)
    {
      TemperaturaCasadeBolsas=TemperaturaCasadeBolsas+0.5;
      }
    if (xPosition>=600)
    {
      TemperaturaCasadeBolsas=TemperaturaCasadeBolsas-0.5;
      }
     
    }

    if (CursorVertical==43)
  {
    if (xPosition<=300)
    {
      TemperaturaMezcla=TemperaturaMezcla+0.5;
      }
    if (xPosition>=600)
    {
      TemperaturaMezcla=TemperaturaMezcla-0.5;
      }
     
    }


      if (CursorVertical==44)
  {
    if (xPosition<=300)
    {
      PosicionDamperQuemador=PosicionDamperQuemador+1;
      }
    if (xPosition>=600)
    {
      PosicionDamperQuemador=PosicionDamperQuemador-1;
      }
     
    }


      if (CursorVertical==45)
  {
    if (xPosition<=300)
    {
      PosicionDamperExtractorBH=PosicionDamperExtractorBH+1;
      }
    if (xPosition>=600)
    {
      PosicionDamperExtractorBH=PosicionDamperExtractorBH-1;
      }
     
    }
 }


void LecturaJoystick ()

{
    
    xPosition = analogRead(xPin);
    yPosition = analogRead(yPin);
    buttonState = digitalRead(buttonPin);
  

  
  if (yPosition <=300 && CursorVertical<100)
  CursorVertical ++;


  if (yPosition >=600 && CursorVertical >0)
  CursorVertical --;

  if (xPosition <=300) 
  cambiar_dato==true;


  if (xPosition >=600)
  cambiar_dato==true;

  
  if (buttonState== LOW)
  
  cambiar_dato = true;
  else
  cambiar_dato = false;

  




  }
void EnvioAnalogasProgramaPrincipal()

{
  int PresionCasadeBolsasLocal=0;
        PresionCasadeBolsasLocal= (int)(PresionCasadeBolsas);
        DatoAnalogo_a_enviar[0]= 'd';
        DatoAnalogo_a_enviar[1]= 'a';
        DatoAnalogo_a_enviar[2]= 't';
        DatoAnalogo_a_enviar[3]= 'o';
        DatoAnalogo_a_enviar[4]= '3';// envío de señales analogoas
        Serial.println(DatoAnalogo_a_enviar);  //envía 'dato3'
        Serial.println(PresionCasadeBolsas,2);// enviando numero double con 2 cifras decimales
        Serial.println(TemperaturaCasadeBolsas, 2);
        Serial.println(TemperaturaMezcla, 2);
        Serial.println(PosicionDamperQuemador, 2);
        Serial.println(PosicionDamperExtractorBH, 2);
        Serial.println(Valor_FlameEye);
        Serial.println(Valor_IndAgregados);
        Serial.println(Valor_IndAsfalto);
        Serial.println(Valor_IndReciclado);
        
        
        
        
  }


void LecturaAnalogas()
    {
      Valor_FlameEye = analogRead(FlameEye);
      Valor_IndAgregados = analogRead(IndAgregados);
      Valor_IndAsfalto = analogRead(IndAsfalto);
      Valor_IndReciclado = analogRead(IndReciclado);
      
      
      }

      
  




void VisualizarLCD()
  {

    lcd.setCursor (0,1); 
    lcd.print(CursorVertical);
    if (CursorVertical<100)
      {
      lcd.setCursor(0,0);
      lcd.print(CursorVertical+1);
      }
    
    if (CursorVertical<9) //borrar cualquier cero que quede grabado en el LCD
    { 
      lcd.setCursor(1,0);
      lcd.print(" ");
      lcd.setCursor(1,1);
      lcd.print(" ");
      }
    if(CursorVertical==9)
    { 
      lcd.setCursor(1,1);
      lcd.print(" ");
     }
    //delay (150);//espera
    



    
    switch (CursorVertical) {
    case 0:
    lcd.setCursor (3,0);
    lcd.print("B.Aliment");
    lcd.setCursor (13,0);
    lcd.print (BandaAlimentadoraEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaAlimentadoraSalida==0)
      BandaAlimentadoraSalida=1;
      else
      BandaAlimentadoraSalida=0;
    }
   /* lcd.setCursor (15,0);
    lcd.print (BandaAlimentadoraSalida);

    lcd.setCursor (3,1);
    lcd.print(DatoaEsclavo1);
    */
    
    break;
    
    case 1:
    lcd.setCursor (3,0);
    lcd.print("Zaranda  ");
    lcd.setCursor (13,0);
    lcd.print (ZarandaEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (ZarandaSalida==0)
      ZarandaSalida=1;
      else
      ZarandaSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (ZarandaSalida);

    lcd.setCursor (3,1);
    lcd.print("B.Aliment");
    lcd.setCursor (13,1);
    lcd.print (BandaAlimentadoraEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaAlimentadoraSalida);
    break;
    
    
    case 2:
    lcd.setCursor (3,0);
    lcd.print("B.Colecto");
    lcd.setCursor (13,0);
    lcd.print (BandaColectoraEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaColectoraSalida==0)
      BandaColectoraSalida=1;
      else
      BandaColectoraSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BandaColectoraSalida);

    lcd.setCursor (3,1);
    lcd.print("Zaranda  ");
    lcd.setCursor (13,1);
    lcd.print (ZarandaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (ZarandaSalida);
    break;

    case 3:
    lcd.setCursor (3,0);
    lcd.print("PPB.Alime");
    lcd.setCursor (13,0);
    lcd.print (PesoPruebaBandaAlimentadoraEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (PesoPruebaBandaAlimentadoraSalida==0)
      PesoPruebaBandaAlimentadoraSalida=1;
      else
      PesoPruebaBandaAlimentadoraSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (PesoPruebaBandaAlimentadoraSalida);

    lcd.setCursor (3,1);
    lcd.print("B.Colecto");
    lcd.setCursor (13,1);
    lcd.print (BandaColectoraEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaColectoraSalida);
    
    break;
    
    case 4:
    lcd.setCursor (3,0);
    lcd.print("B.Agrega1");
    lcd.setCursor (13,0);
    lcd.print (BandaAgregados1Entrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaAgregados1Salida==0)
      BandaAgregados1Salida=1;
      else
      BandaAgregados1Salida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BandaAgregados1Salida);

    lcd.setCursor (3,1);
    lcd.print("PPB.Alime");
    lcd.setCursor (13,1);
    lcd.print (PesoPruebaBandaAlimentadoraEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (PesoPruebaBandaAlimentadoraSalida);
    break;
    
    case 5:
    lcd.setCursor (3,0);
    lcd.print("B.Agrega2");
    lcd.setCursor (13,0);
    lcd.print (BandaAgregados2Entrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaAgregados2Salida==0)
      BandaAgregados2Salida=1;
      else
      BandaAgregados2Salida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BandaAgregados2Salida);

    lcd.setCursor (3,1);
    lcd.print("B.Agrega1");
    lcd.setCursor (13,1);
    lcd.print (BandaAgregados1Entrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaAgregados1Salida);
    break;
    
    case 6:
    lcd.setCursor (3,0);
    lcd.print("B.Agrega3");
    lcd.setCursor (13,0);
    lcd.print (BandaAgregados3Entrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaAgregados3Salida==0)
      BandaAgregados3Salida=1;
      else
      BandaAgregados3Salida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BandaAgregados3Salida);

    lcd.setCursor (3,1);
    lcd.print("B.Agrega2");
    lcd.setCursor (13,1);
    lcd.print (BandaAgregados2Entrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaAgregados2Salida);
    break;
   
    case 7:
    lcd.setCursor (3,0);
    lcd.print("B.Agrega4");
    lcd.setCursor (13,0);
    lcd.print (BandaAgregados4Entrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaAgregados4Salida==0)
      BandaAgregados4Salida=1;
      else
      BandaAgregados4Salida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BandaAgregados4Salida);

    lcd.setCursor (3,1);
    lcd.print("B.Agrega3");
    lcd.setCursor (13,1);
    lcd.print (BandaAgregados3Entrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaAgregados3Salida);
    break;
    
    case 8:
    lcd.setCursor (3,0);
    lcd.print("B.IncReci");
    lcd.setCursor (13,0);
    lcd.print (BandaInclinadaRecicladoEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaInclinadaRecicladoSalida==0)
      BandaInclinadaRecicladoSalida=1;
      else
      BandaInclinadaRecicladoSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BandaInclinadaRecicladoSalida);

    lcd.setCursor (3,1);
    lcd.print("B.Agrega4");
    lcd.setCursor (13,1);
    lcd.print (BandaAgregados4Entrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaAgregados4Salida);
    break;
    
    case 9:
    lcd.setCursor (3,0);
    lcd.print("B.AggReci");
    lcd.setCursor (13,0);
    lcd.print (BandaAgregadoRecicladoEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BandaAgregadoRecicladoSalida==0)
      BandaAgregadoRecicladoSalida=1;
      else
      BandaAgregadoRecicladoSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BandaAgregadoRecicladoSalida);

    lcd.setCursor (3,1);
    lcd.print("B.IncReci");
    lcd.setCursor (13,1);
    lcd.print (BandaInclinadaRecicladoEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaInclinadaRecicladoSalida);
    break;
    
    case 10:
    lcd.setCursor (3,0);
    lcd.print("Trit.Reci");
    lcd.setCursor (13,0);
    lcd.print (MotorTrituradoRecicladoEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (MotorTrituradoRecicladoSalida==0)
      MotorTrituradoRecicladoSalida=1;
      else
      MotorTrituradoRecicladoSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (MotorTrituradoRecicladoSalida);

    lcd.setCursor (3,1);
    lcd.print("B.AggReci");
    lcd.setCursor (13,1);
    lcd.print (BandaAgregadoRecicladoEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BandaAgregadoRecicladoSalida);
    break;
    
    case 11:
    lcd.setCursor (3,0);
    lcd.print("PPB.Recic");
    lcd.setCursor (13,0);
    lcd.print (PesoPruebaBandaRecicladoEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (PesoPruebaBandaRecicladoSalida==0)
      PesoPruebaBandaRecicladoSalida=1;
      else
      PesoPruebaBandaRecicladoSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (PesoPruebaBandaRecicladoSalida);

    lcd.setCursor (3,1);
    lcd.print("Trit.Reci");
    lcd.setCursor (13,1);
    lcd.print (MotorTrituradoRecicladoEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (MotorTrituradoRecicladoSalida);
    break;
    
    case 12:
    lcd.setCursor (3,0);
    lcd.print("CompRecic");
    lcd.setCursor (13,0);
    lcd.print (CompuertaRecicladoEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (CompuertaRecicladoSalida==0)
      CompuertaRecicladoSalida=1;
      else
      CompuertaRecicladoSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (CompuertaRecicladoSalida);

    lcd.setCursor (3,1);
    lcd.print("PPB.Recic");
    lcd.setCursor (13,1);
    lcd.print (PesoPruebaBandaRecicladoEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (PesoPruebaBandaRecicladoSalida);
    break;
    
    case 13:
    lcd.setCursor (3,0);
    lcd.print("Blower.BH");
    lcd.setCursor (13,0);
    lcd.print (MotorBlowerBHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (MotorBlowerBHSalida==0)
      MotorBlowerBHSalida=1;
      else
      MotorBlowerBHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (MotorBlowerBHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("CompRecic");
    lcd.setCursor (13,1);
    lcd.print (CompuertaRecicladoEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (CompuertaRecicladoSalida);
    break;

    case 14:
    lcd.setCursor (3,0);
    lcd.print("VannerBH ");
    lcd.setCursor (13,0);
    lcd.print (MotorVannerBHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (MotorVannerBHSalida==0)
      MotorVannerBHSalida=1;
      else
      MotorVannerBHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (MotorVannerBHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Blower.BH");
    lcd.setCursor (13,1);
    lcd.print (MotorBlowerBHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (MotorBlowerBHSalida);
    break;

    case 15:
    lcd.setCursor (3,0);
    lcd.print("SF.Colec1");
    lcd.setCursor (13,0);
    lcd.print (SinFinColector1BHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (SinFinColector1BHSalida==0)
      SinFinColector1BHSalida=1;
      else
      SinFinColector1BHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (SinFinColector1BHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("VannerBH ");
    lcd.setCursor (13,1);
    lcd.print (MotorVannerBHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (MotorVannerBHSalida);
    break;
    
    case 16:
    lcd.setCursor (3,0);
    lcd.print("SF.Colec2");
    lcd.setCursor (13,0);
    lcd.print (SinFinColector2BHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (SinFinColector2BHSalida==0)
      SinFinColector2BHSalida=1;
      else
      SinFinColector2BHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (SinFinColector2BHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("SF.Colec1");
    lcd.setCursor (13,1);
    lcd.print (SinFinColector1BHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (SinFinColector1BHSalida);
    break;
    
    case 17:
    lcd.setCursor (3,0);
    lcd.print("SF.Transv");
    lcd.setCursor (13,0);
    lcd.print (SinFinTransversalBHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (SinFinTransversalBHSalida==0)
      SinFinTransversalBHSalida=1;
      else
      SinFinTransversalBHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (SinFinTransversalBHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("SF.Colec2");
    lcd.setCursor (13,1);
    lcd.print (SinFinColector2BHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (SinFinColector2BHSalida);
    break;

    case 18:
    lcd.setCursor (3,0);
    lcd.print("Compresor");
    lcd.setCursor (13,0);
    lcd.print (CompresorEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (CompresorSalida==0)
      CompresorSalida=1;
      else
      CompresorSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (CompresorSalida);
    
    lcd.setCursor (3,1);
    lcd.print("SF.Transv");
    lcd.setCursor (13,1);
    lcd.print (SinFinTransversalBHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (SinFinTransversalBHSalida);
    break;

    case 19:
    lcd.setCursor (3,0);
    lcd.print("B.Asf.Ade");
    lcd.setCursor (13,0);
    lcd.print (BombaAsfaltoAdelanteEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BombaAsfaltoAdelanteSalida==0)
      BombaAsfaltoAdelanteSalida=1;
      else
      BombaAsfaltoAdelanteSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BombaAsfaltoAdelanteSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Compresor");
    lcd.setCursor (13,1);
    lcd.print (CompresorEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (CompresorSalida);
    
    break;

    case 20:
    lcd.setCursor (3,0);
    lcd.print("B.Asf.Atr");
    lcd.setCursor (13,0);
    lcd.print (BombaAsfaltoAtrasEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BombaAsfaltoAtrasSalida==0)
      BombaAsfaltoAtrasSalida=1;
      else
      BombaAsfaltoAtrasSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BombaAsfaltoAtrasSalida);
    
    lcd.setCursor (3,1);
    lcd.print("B.Asf.Ade");
    lcd.setCursor (13,1);
    lcd.print (BombaAsfaltoAdelanteEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BombaAsfaltoAdelanteSalida);
    break;

    case 21:
    lcd.setCursor (3,0);
    lcd.print("ExtracBH1");
    lcd.setCursor (13,0);
    lcd.print (Extractor1BHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (Extractor1BHSalida==0)
      Extractor1BHSalida=1;
      else
      Extractor1BHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (Extractor1BHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("B.Asf.Atr");
    lcd.setCursor (13,1);
    lcd.print (BombaAsfaltoAtrasEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BombaAsfaltoAtrasSalida);
    break;

    case 22:
    lcd.setCursor (3,0);
    lcd.print("ExtracBH2");
    lcd.setCursor (13,0);
    lcd.print (Extractor2BHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (Extractor2BHSalida==0)
      Extractor2BHSalida=1;
      else
      Extractor2BHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (Extractor2BHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("ExtracBH1");
    lcd.setCursor (13,1);
    lcd.print (Extractor1BHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (Extractor1BHSalida);
    break;

    case 23:
    lcd.setCursor (3,0);
    lcd.print("BlwQuemad");
    lcd.setCursor (13,0);
    lcd.print (BlowerQuemadorEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BlowerQuemadorSalida==0)
      BlowerQuemadorSalida=1;
      else
      BlowerQuemadorSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BlowerQuemadorSalida);
    
    lcd.setCursor (3,1);
    lcd.print("ExtracBH2");
    lcd.setCursor (13,1);
    lcd.print (Extractor2BHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (Extractor2BHSalida);
    break;

    case 24:
    lcd.setCursor (3,0);
    lcd.print("Elevador ");
    lcd.setCursor (13,0);
    lcd.print (ElevadorEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (ElevadorSalida==0)
      ElevadorSalida=1;
      else
      ElevadorSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (ElevadorSalida);
    
    lcd.setCursor (3,1);
    lcd.print("BlwQuemad");
    lcd.setCursor (13,1);
    lcd.print (BlowerQuemadorEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BlowerQuemadorSalida);
    break;

    case 25:
    lcd.setCursor (3,0);
    lcd.print("C/Rechazo");
    lcd.setCursor (13,0);
    lcd.print (CompuertaRechazoEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (CompuertaRechazoSalida==0)
      CompuertaRechazoSalida=1;
      else
      CompuertaRechazoSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (CompuertaRechazoSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Elevador ");
    lcd.setCursor (13,1);
    lcd.print (ElevadorEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (ElevadorSalida);
    break;

    case 26:
    lcd.setCursor (3,0);
    lcd.print("Secador  ");
    lcd.setCursor (13,0);
    lcd.print (SecadorEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (SecadorSalida==0)
      SecadorSalida=1;
      else
      SecadorSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (SecadorSalida);
    
    lcd.setCursor (3,1);
    lcd.print("C/Rechazo");
    lcd.setCursor (13,1);
    lcd.print (CompuertaRechazoEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (CompuertaRechazoSalida);
    break;

    case 27:
    lcd.setCursor (3,0);
    lcd.print("SlinAdela");
    lcd.setCursor (13,0);
    lcd.print (SlingerAdelanteEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (SlingerAdelanteSalida==0)
      SlingerAdelanteSalida=1;
      else
      SlingerAdelanteSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (SlingerAdelanteSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Secador  ");
    lcd.setCursor (13,1);
    lcd.print (SecadorEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (SecadorSalida);
    break;

    case 28:
    lcd.setCursor (3,0);
    lcd.print("SlinAtras");
    lcd.setCursor (13,0);
    lcd.print (SlingerAtrasEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (SlingerAtrasSalida==0)
      SlingerAtrasSalida=1;
      else
      SlingerAtrasSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (SlingerAtrasSalida);
    
    lcd.setCursor (3,1);
    lcd.print("SlinAdela");
    lcd.setCursor (13,1);
    lcd.print (SlingerAdelanteEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (SlingerAdelanteSalida);
    break;

    case 29:
    lcd.setCursor (3,0);
    lcd.print("BombCombu");
    lcd.setCursor (13,0);
    lcd.print (BombaCombustibleEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (BombaCombustibleSalida==0)
      BombaCombustibleSalida=1;
      else
      BombaCombustibleSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (BombaCombustibleSalida);
    
    lcd.setCursor (3,1);
    lcd.print("SlinAtras");
    lcd.setCursor (13,1);
    lcd.print (SlingerAtrasEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (SlingerAtrasSalida);
    break;

    case 30:
    lcd.setCursor (3,0);
    lcd.print("Damper    ");
    lcd.setCursor (13,0);
    lcd.print (DamperEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (DamperSalida==0)
      DamperSalida=1;
      else
      DamperSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (DamperSalida);
    
    lcd.setCursor (3,1);
    lcd.print("BombCombu");
    lcd.setCursor (13,1);
    lcd.print (BombaCombustibleEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (BombaCombustibleSalida);
    break;

    case 31:
    lcd.setCursor (3,0);
    lcd.print("Piloto   ");
    lcd.setCursor (13,0);
    lcd.print (PilotoValvulaEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (PilotoValvulaSalida==0)
      PilotoValvulaSalida=1;
      else
      PilotoValvulaSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (PilotoValvulaSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Damper   ");
    lcd.setCursor (13,1);
    lcd.print (DamperEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (DamperSalida);
    break;

    case 32:
    lcd.setCursor (3,0);
    lcd.print("MainValvu");
    lcd.setCursor (13,0);
    lcd.print (MainLlamaEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (MainLlamaSalida==0)
      MainLlamaSalida=1;
      else
      MainLlamaSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (MainLlamaSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Piloto   ");
    lcd.setCursor (13,1);
    lcd.print (PilotoValvulaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (PilotoValvulaSalida);
    break;

    case 33:
    lcd.setCursor (3,0);
    lcd.print("Rotor1.BH");
    lcd.setCursor (13,0);
    lcd.print (Rotor1BHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (Rotor1BHSalida==0)
      Rotor1BHSalida=1;
      else
      Rotor1BHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (Rotor1BHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("MainValvu");
    lcd.setCursor (13,1);
    lcd.print (MainLlamaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (MainLlamaSalida);
    break;

    case 34:
    lcd.setCursor (3,0);
    lcd.print("Rotor.2BH");
    lcd.setCursor (13,0);
    lcd.print (Rotor2BHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (Rotor2BHSalida==0)
      Rotor2BHSalida=1;
      else
      Rotor2BHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (Rotor2BHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Rotor.1BH");
    lcd.setCursor (13,1);
    lcd.print (Rotor1BHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (Rotor1BHSalida);
    break;

    case 35:
    lcd.setCursor (3,0);
    lcd.print("Rotor.3BH");
    lcd.setCursor (13,0);
    lcd.print (Rotor3BHEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (Rotor3BHSalida==0)
      Rotor3BHSalida=1;
      else
      Rotor3BHSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (Rotor3BHSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Rotor.2BH");
    lcd.setCursor (13,1);
    lcd.print (Rotor2BHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (Rotor2BHSalida);
    break;

    
    case 36:
    lcd.setCursor (3,0);
    lcd.print("Alim/Silo");
    lcd.setCursor (13,0);
    lcd.print (AlimentacionSiloEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (AlimentacionSiloSalida==0)
      AlimentacionSiloSalida=1;
      else
      AlimentacionSiloSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (AlimentacionSiloSalida);
    
    lcd.setCursor (3,10);
    lcd.print("Rotor3.BH");
    lcd.setCursor (13,1);
    lcd.print (Rotor3BHEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (Rotor3BHSalida);
    break;

    case 37:
    lcd.setCursor (3,0);
    lcd.print("Valv/Acei");
    lcd.setCursor (13,0);
    lcd.print (ValvulaAceiteEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (ValvulaAceiteSalida==0)
      ValvulaAceiteSalida=1;
      else
      ValvulaAceiteSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (ValvulaAceiteSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Alim/Silo");
    lcd.setCursor (13,1);
    lcd.print (AlimentacionSiloEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (AlimentacionSiloSalida);
    break;

    case 38:
    lcd.setCursor (3,0);
    lcd.print("Valvu/gas");
    lcd.setCursor (13,0);
    lcd.print (ValvulaGasEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (ValvulaGasSalida==0)
      ValvulaGasSalida=1;
      else
      ValvulaGasSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (ValvulaGasSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Valv/Acei");
    lcd.setCursor (13,1);
    lcd.print (ValvulaAceiteEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (ValvulaAceiteSalida);
    break;

    case 39:
    lcd.setCursor (3,0);
    lcd.print("Comp/Gans");
    lcd.setCursor (13,0);
    lcd.print (CompuertaGansoEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (CompuertaGansoSalida==0)
      CompuertaGansoSalida=1;
      else
      CompuertaGansoSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (CompuertaGansoSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Valvu/gas");
    lcd.setCursor (13,1);
    lcd.print (ValvulaGasEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (ValvulaGasSalida);
    break;

    case 40:
    lcd.setCursor (3,0);
    lcd.print("R8090.On ");
    lcd.setCursor (13,0);
    lcd.print (R8090ControlLLamaEntrada);
    lcd.setCursor (14,0);
    lcd.print (" ");
    if (cambiar_dato == true)
    {
      cambiar_dato=false;
      if (R8090ControlLLamaSalida==0)
      R8090ControlLLamaSalida=1;
      else
      R8090ControlLLamaSalida=0;
    }
    lcd.setCursor (15,0);
    lcd.print (R8090ControlLLamaSalida);
    
    lcd.setCursor (3,1);
    lcd.print("Comp/Gans");
    lcd.setCursor (13,1);
    lcd.print (CompuertaGansoEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (CompuertaGansoSalida);
    break;

    case 41:
    lcd.setCursor (3,0);
    lcd.print("PresionBH");
    lcd.setCursor (12,0);
    lcd.print (PresionCasadeBolsas);
    
    lcd.setCursor (3,1);
    lcd.print("R8090.On ");
    lcd.setCursor (13,1);
    lcd.print (R8090ControlLLamaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (R8090ControlLLamaSalida);
    break;

    case 42:
    lcd.setCursor (3,0);
    lcd.print("IncDampQue");
    lcd.setCursor (12,0);
    lcd.print (PresionCasadeBolsas);
    
    lcd.setCursor (3,1);
    lcd.print("R8090.On ");
    lcd.setCursor (13,1);
    lcd.print (R8090ControlLLamaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (R8090ControlLLamaSalida);
    break;
    
    case 43:
    lcd.setCursor (3,0);
    lcd.print("PresionBH");
    lcd.setCursor (12,0);
    lcd.print (PresionCasadeBolsas);
    
    lcd.setCursor (3,1);
    lcd.print("R8090.On ");
    lcd.setCursor (13,1);
    lcd.print (R8090ControlLLamaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (R8090ControlLLamaSalida);
    break;
  
    case 44:
    lcd.setCursor (3,0);
    lcd.print("PresionBH");
    lcd.setCursor (12,0);
    lcd.print (PresionCasadeBolsas);
    
    lcd.setCursor (3,1);
    lcd.print("R8090.On ");
    lcd.setCursor (13,1);
    lcd.print (R8090ControlLLamaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (R8090ControlLLamaSalida);
    break;
    
    case 45:
    lcd.setCursor (3,0);
    lcd.print("PresionBH");
    lcd.setCursor (12,0);
    lcd.print (PresionCasadeBolsas);
    
    lcd.setCursor (3,1);
    lcd.print("R8090.On ");
    lcd.setCursor (13,1);
    lcd.print (R8090ControlLLamaEntrada);
    lcd.setCursor (14,1);
    lcd.print (" ");
    lcd.setCursor (15,1);
    lcd.print (R8090ControlLLamaSalida);
    break;

    
    case 46:
    lcd.setCursor (3,0);
    lcd.print("TemperaBH");
    lcd.setCursor (12,0);
    lcd.print (TemperaturaCasadeBolsas);
    
    lcd.setCursor (3,1);
    lcd.print("PresionBH");
    lcd.setCursor (12,1);
    lcd.print (PresionCasadeBolsas);
    break;

    case 47:
    lcd.setCursor (3,0);
    lcd.print("TmpMezcla");
    lcd.setCursor (12,0);
    lcd.print (TemperaturaMezcla);
    
    lcd.setCursor (3,1);
    lcd.print("TemperaBH");
    lcd.setCursor (12,1);
    lcd.print (TemperaturaCasadeBolsas);
    break;

    case 48:
    lcd.setCursor (3,0);
    lcd.print("PosDamQue    ");
    lcd.setCursor (13,0);
    lcd.print (PosicionDamperQuemador);
    
    lcd.setCursor (3,1);
    lcd.print("TmpMezcla    ");
    lcd.setCursor (13,1);
    lcd.print (TemperaturaMezcla);
    break;

    case 49:
    lcd.setCursor (3,0);
    lcd.print("PosDamExt    ");
    lcd.setCursor (13,0);
   lcd.print (PosicionDamperExtractorBH);
    
    lcd.setCursor (3,1);
    lcd.print("PosDamQue    ");
    lcd.setCursor (13,1);
    lcd.print (PosicionDamperQuemador);
    break;

    case 50:
    lcd.setCursor (3,0);
    lcd.print("VA1 ");
    lcd.setCursor (12,0);
    lcd.print (VelocidadAgregado1Hz);
    
    lcd.setCursor (3,1);
    lcd.print("PosDamExt    ");
    lcd.setCursor (13,1);
    lcd.print (PosicionDamperExtractorBH);
    break;

    case 51:
    lcd.setCursor (3,0);
    lcd.print("VelAgg2Hz    ");
    lcd.setCursor (12,0);
    lcd.print (VelocidadAgregado2Hz);
    
    lcd.setCursor (3,1);
    lcd.print("VelAgg1Hz    ");
    lcd.setCursor (12,1);
    lcd.print (VelocidadAgregado1Hz);
    break;

    case 52:
    lcd.setCursor (3,0);
    lcd.print("VelAgg3Hz    ");
    lcd.setCursor (12,0);
    lcd.print (VelocidadAgregado3Hz);
    
    lcd.setCursor (3,1);
    lcd.print("VelAgg2Hz    ");
    lcd.setCursor (12,1);
    lcd.print (VelocidadAgregado2Hz);
    break;

    case 53:
    lcd.setCursor (3,0);
    lcd.print("VelAgg4Hz    ");
    lcd.setCursor (12,0);
    lcd.print (VelocidadAgregado4Hz);
    
    lcd.setCursor (3,1);
    lcd.print("VelAgg3Hz    ");
    lcd.setCursor (12,1);
    lcd.print (VelocidadAgregado3Hz);
    break;

    case 54:
    lcd.setCursor (3,0);
    lcd.print("VelAggRec    ");
    lcd.setCursor (12,0);
    lcd.print (VelocidadAgregadoRecicladoHz);
    
    lcd.setCursor (3,1);
    lcd.print("VelAgg4Hz    ");
    lcd.setCursor (12,1);
    lcd.print (VelocidadAgregado4Hz);
    break;

    case 55:
    lcd.setCursor (3,0);
    lcd.print("VelAsfHz    ");
    lcd.setCursor (12,0);
    lcd.print (VelocidadAsfaltoHz);
    
    lcd.setCursor (3,1);
    lcd.print("VelAggRec    ");
    lcd.setCursor (12,1);
    lcd.print (VelocidadAgregadoRecicladoHz);
    break;

    case 56:
    lcd.setCursor (3,0);
    lcd.print("tiempo            ");
    lcd.setCursor (12,0);
    lcd.print (tiempo1);
    
    lcd.setCursor (3,1);
    lcd.print("VelAsfHz    ");
    lcd.setCursor (12,1);
    lcd.print (VelocidadAsfaltoHz);
    break;

    case 57:
    lcd.setCursor (3,0);
    lcd.print(DatoLeido1[0]);
    lcd.print(DatoLeido1[1]);
    lcd.print(DatoLeido1[2]);
    lcd.print(DatoLeido1[3]);
    lcd.print(DatoLeido1[4]);
    lcd.print(DatoLeido1[5]);
    lcd.print(DatoLeido1[6]);
    lcd.print(DatoLeido1[7]);
    lcd.print(DatoLeido1[8]);
    lcd.print(DatoLeido1[9]);
    lcd.print(DatoLeido1[10]);
    lcd.print(DatoLeido1[11]);
    lcd.print(DatoLeido1[12]);
    lcd.print(DatoLeido1[13]);
    
    lcd.setCursor (3,1);
    lcd.print("tiempo    ");
    lcd.setCursor (12,1);
    lcd.print (tiempo1);
    break;

    case 58:
    lcd.setCursor (3,0);
    lcd.print(DatoLeido2[0]);
    lcd.print(DatoLeido2[1]);
    lcd.print(DatoLeido2[2]);
    lcd.print(DatoLeido2[3]);
    lcd.print(DatoLeido2[4]);
    lcd.print(DatoLeido2[5]);
    lcd.print(DatoLeido2[6]);
    lcd.print(DatoLeido2[7]);
    lcd.print(DatoLeido2[8]);
    lcd.print(DatoLeido2[9]);
    lcd.print(DatoLeido2[10]);
    lcd.print(DatoLeido2[11]);
    lcd.print(DatoLeido2[12]);
    lcd.print(DatoLeido2[13]);
    
    lcd.setCursor (3,1);
    lcd.print(DatoLeido1[0]);
    lcd.print(DatoLeido1[1]);
    lcd.print(DatoLeido1[2]);
    lcd.print(DatoLeido1[3]);
    lcd.print(DatoLeido1[4]);
    lcd.print(DatoLeido1[5]);
    lcd.print(DatoLeido1[6]);
    lcd.print(DatoLeido1[7]);
    lcd.print(DatoLeido1[8]);
    lcd.print(DatoLeido1[9]);
    lcd.print(DatoLeido1[10]);
    lcd.print(DatoLeido1[11]);
    lcd.print(DatoLeido1[12]);
    lcd.print(DatoLeido1[13]);
    break;

    case 59:
    lcd.setCursor (3,0);
    lcd.print("            ");
    lcd.setCursor (3,1);
    lcd.setCursor (3,0);
    lcd.print(DatoLeido2[0]);
    lcd.print(DatoLeido2[1]);
    lcd.print(DatoLeido2[2]);
    lcd.print(DatoLeido2[3]);
    lcd.print(DatoLeido2[4]);
    lcd.print(DatoLeido2[5]);
    lcd.print(DatoLeido2[6]);
    lcd.print(DatoLeido2[7]);
    lcd.print(DatoLeido2[8]);
    lcd.print(DatoLeido2[9]);
    lcd.print(DatoLeido2[10]);
    lcd.print(DatoLeido2[11]);
    lcd.print(DatoLeido2[12]);
    lcd.print(DatoLeido2[13]);
    break;
   
    default: 
    lcd.clear();
      // if nothing else matches, do the default
      // default is optional
    break;
  
    }
  }

     //*/

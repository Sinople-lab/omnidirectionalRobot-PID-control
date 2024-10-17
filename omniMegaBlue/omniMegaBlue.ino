/* Control PID para robot Omnidireccional 
   Martin Carballo Flores
   02/sep/2019
*/
// -------------------------------------------------------------------------------------------
int enc[4]={31,33,35,37};   // Pines para canal A de los encoders
float pulses[4]={0,0,0,0};  // Arreglo de contadores de encoders
int R=0;                    // variable para tamaño de lado
int est[4];                 // Arreglo de estados actuales de los encoders
int last[4];                // Arreglo de estados anteriores de los encoders
// -------------------------------------------------------------------------------------------
float t0=0, tf=0, dt=0, ft;             // Variables de tiempo inicial, final y derivada
float puls0[4]={0,0,0,0};               // Arreglo de contadores en t0 de encoders
float pulsf[4]={0,0,0,0};               // Arreglo de contadores en tf de encoders
float dpuls[4]={0,0,0,0};               // Arreglo de derivada de contadores obtenidos
float kp[4] = {3, 3, 3, 3};             // + = mas rapidomenos preciso
float ki[4] = {3.3, 2.7, 2.6, 2.7};     // + = mas rapido y ligeramente mayor oscilacion
float kd[4] = {0.05, 0.05, 0.05, 0.04}; // + = mas lento y mas inestable
float error[4]={0,0,0,0}, cumError[4]={0,0,0,0};     // Arreglo de error y de suma de error
float rateError[4]={0,0,0,0}, lastError[4]={0,0,0,0};// Arreglo para derivar error
//float want[4]={33, 32, 31.5, 32};//*/                // contadores deseados por tiempo de muestreo
float want[4]={41.8, 41.3, 41.6, 41.7};// *? [2]
//--------------------------------------------------------------------------------------------
int motores[4]={4,5,6,7};         // pines de pwm
//float pwms[4]={86,56,52,58};    // anchos a las que los motores tienen aprox. la msima vel.
float pwms[4]={0,0,0,0};          // Anchos iniciales
String incoming;                  // Variable para recibir los movimientos a realizar
String direcciones;               // Variable para almacenar los movimientos a realizar
void setup(){                     // FUncion de inicializacion
  Serial1.begin(9600);            // Velocidad de 9600 baudios para modulo BT en Serial1(RX19, TX18)
  DDRA |= B11111111;              // Configuracion de pines de direcion de motores como salidas
  /*
     El comando de registros DDRx, donde x es el registro a manipular, configura los pines como
     salidas si los bits se ponen en 1 y como entradas si se ponen en 0, los valores iniciales
     son 0, por lo tanto no es necesario configurar entradas. Al colocar los 1 y 0 se inicia
     colocando B. El orden de los bits en AVRATmega2560 es [0,1,2,3,4,5,6,7]. Se debe
     revisar a que pines de arduino corresponden los pines del micro que se quieran configurar.
  */
  for(int i=0;i<4; i++){         // Para cada sensor
    est[i]=digitalRead(enc[i]);  // Se lee el estado actual de los motores
    last[i]=est[i];              // Las lecturas se establecen como el estado anterior
  }
  DDRG |= B00000100;             // -Configuracion de los pines pwm
  DDRE |= B00010000;             // PG5, PE3, PH3 y PH4 como salidas,
  DDRH |= B00011000;             // es decir pines 4,5,6,7 de arduino.
  PORTG &= B11111011;            // -Poner en bajolos pines
  PORTE &= B11101111;            // 4,5,6,7 de arduino, es decir
  PORTH &= B11100111;            // PG5, PE3, PH3, PH4 del micro.
  /*
     El comando de registros PORTx, donde x es el registro a manipular, se usa para poner
      los pines en 1 o 0 usando compuerta and u or
  */
}
void loop(){                           // Funcion principal
  if(Serial1.available()>0){           // Si hay conexiones entrantes
    incoming = Serial1.readString();   // Se lee una linea
    direcciones=incoming;              // Se guarda la cadena leida
    pulse();                           // Se ejecutan los movimientos
  }
}
void pwm(float pwm0, float pwm1, float pwm2, float pwm3){// Funcion para enviar pwm a motores
  float out[4]={pwm0, pwm1, pwm2, pwm3};// Se toman las entradas de la funcion
  for(int i=0;i<4;i++){              // para cada pin pwm
    analogWrite(motores[i], out[i]); // Enviar el pulso de ancho=out[i] a motor[i]
  }
}
void off(){                          // Funcion para apagar los motores
  for(int i=0;i<4;i++){              // para cada pin pwm
    analogWrite(motores[i],0);       // Enviar pwm=0
  }
}
void encoders(){                     // Funcion para leer los encoders
  for(int t=0;t<4;t++){              // Para cada encoder
    est[t]=digitalRead(enc[t]);      // Leer el estadoa ctual del encoder
    if(est[t]!=last[t]){             // Si es diferente al estado anterior
      pulses[t]+=1;                  // Aumentar el contador del encoder
      last[t]=est[t];                // El edo. actual sera el edo. anterior
    }
  }
}
void pid(int x0, int x1, int x2, int x3){        // Funcion de control PID
  int enabPID[4]={x0, x1, x2, x3};               // Guardar la configuracion de PID
  tf=millis();                                   // Tomar el tiempo actual
  dt=tf-t0;                                      // Obtener la derivada del tiempo
  if(dt>25){                                     // Si la derivada del tiempo es mayor a 25 milli
    dt=dt/1000;                                  // Transforma dt a segundos
    for(int i=0; i<4;i++){                       // Para cada pwm
      if(enabPID[i]==1){                         // Si el PID para el motor i esta habilitado
        pulsf[i]=pulses[i];                      // Obtener el numero de conteos actual
        dpuls[i]=pulsf[i]-puls0[i];              // Obtener los conteos desde la ultima iteracion
        error[i]=want[i]-dpuls[i];               /* Restar los conteos realizados a los conteos 
                                                    deseados por tiempo de muestreo. */
        cumError[i] += (error[i]*dt);            // Integra los errores-> sum(error[i]*dt)
        rateError[i] = (error[i]-lastError[i]);  // Derivar el error -> errorActual - errorAnterior 
        pwms[i]=kp[i]*error[i]+ki[i]*cumError[i]+38.46*kd[i]*rateError[i];// PID
                                                 /*
                                                    En la parte de kd*rateError se multiplica por
                                                    38.46, porque es el resultado de 1/dt, ya que
                                                    por errores en el ide de esta instalacion
                                                    no se pudo escribir el calculo de la
                                                    derivada del error directamente:
                                                    (error[i]-lastError[i])/dt
                                                 */
        lastError[i]=error[i];                   // Error actual como error anterior para la siguiente
        puls0[i]=pulsf[i];                       // Pulsos finales como pulsos iniciales para la
      }                                          // Siguiente iteracion. 
    }
    t0=tf;                                     // Tiempo actual sera el tiempo inicial
  }                                            // de la siguiente iteracion
}// Pines en alto, pines en bajo, tamaño de paso, movimiento a realizar, indice del movimiento a realizar
void walk(byte Hi, byte Lo, int stp, int scond, char movment, int indx){// Funcion para avanzar un nodo
  t0=millis();                // Guardar el tiempo inicial para pid
  for(int i=0;i<4;i++){       // Para cada conteo de sensores
    puls0[i]=pulses[i];       // Guardar el conteo actual como contador inicial para pid
  }// */
  stepr(stp, Hi, Lo, movment, indx);// avanzar
  stepr(scond, Hi, Lo, movment, indx);
  if(direcciones[indx+1]!=movment){// Si el siguiente moviminto sera en una direccion diferente
    off();                    // Apagar los motores
    for(int i=0;i<4;i++){     // Para cada contador de sensores
      pulses[i]=0;            // Se reinician estos valores para evitar
      cumError[i]=0;          // errores en el PID edespues de haber
      lastError[i]=0;         // detenido el robot por # segundos.
    }
    while(R<500){             // Retardo para detener al robot y evitar derrapes *
      delay(1);               // Retardo para alcanzar segundos detenido
      R+=1;                   // Aumentar el contador del retardo
    }
    R=0;                      // Reinicial el contador para usarse en
  }                           // el siguiente control de velocidad
}
void stepr(int stepr12, byte pinsh, byte pinsl, char dir, int plce){
  while(R<stepr12){                                     // Por <stp> ciclos
    PORTA |= pinsh;                                     // Poner pines en alto usando or y poner pines en
    PORTA &= pinsl;                                     // bajo usando and, para configurar el sentido del motor
    pwm(pwms[0], pwms[1], pwms[2], pwms[3]);            // Enviar pulsos
    encoders();                                         // Leer los encoders
    if((dir=='w')||(dir=='x')||(dir=='d')||(dir=='a')){ // -Si se moveran todas las ruedas
      pid(1,1,1,1);                                     //  pid para todos los motores.
    }else if((dir=='q')||(dir=='c')){                   // -Si solo se moveran dos ruedas
      pid(1,0,0,1);                                     //  pid para dos motores.
    }else if((dir=='e')||(dir=='z')){                   // -Si solo se moveran dos ruedas
      pid(0,1,1,0);                                     //  pid para dos motores.
    }
    R+=1;                                               // Aumenta el contador del ciclo while
  }
  R=0;                                                  // el siguiente control de velocidad
}
void pulse(){                                               // Funcion para determinar el movimiento que se hara
  int L=direcciones.length();                               // Almacenar la longitud de la cadena recibida
  ft=millis();
  for(int j=0;j< L;j++){                                    // Para cada elemento de la cadena
    if(direcciones[j]=='x'){                                // Si el caracter j corresponde a <atras>
      //  Pines en alto, Pines en bajo, tamaño de ressitencia, movimiento a realizar, indice del movimiento a realizar
      walk(B10101010, B10101010, 30000, 11400, direcciones[j], j); // Avanzar al siguiente nodo ---- 30000, 14000
    }else if(direcciones[j]=='d'){                          // Si el caracter j corresponde a <derecha>
      walk(B10010110, B10010110, 30000, 11400, direcciones[j], j); // Avanzar al siguiente nodo
    }else if(direcciones[j]=='a'){                          // Si el caracter j corresponde a <izquierda>
      walk(B01101001, B01101001, 30000, 11400, direcciones[j], j); // Avanzar al siguiente nodo
    }else if(direcciones[j]=='w'){                          // Si el caracter j corresponde a <adelante>
      walk(B01010101, B01010101, 30000, 11400, direcciones[j], j); // Avanzar al siguiente nodo
    }else if(direcciones[j]=='s'){                          // Si el caracter j corresponde a <girar>
      walk(B01100110, B01100110, 30000, 30000, direcciones[j], j); // Girar Hacia otro nodo(pendiente)*
    }else if(direcciones[j]=='q'){                          // Si el caracter j corresponde a <diagonal frente-izquierda>
      walk(B01000001, B01000001, 30000, 14000, direcciones[j], j); // Avanzar al siguiente nodo
    }else if(direcciones[j]=='c'){                          // Si el caracter j corresponde a <diagonal atras-derecha>
      walk(B10000010, B10000010, 30000, 14000, direcciones[j], j); // Avanzar al siguiente nodo
    }else if(direcciones[j]=='e'){                          // Si el caracter j corresponde a <diagonal frente-derecha>
      walk(B00010100, B00010100, 30000, 14000, direcciones[j], j); // Avanzar al siguiente nodo
    }else if(direcciones[j]=='z'){                          // Si el caracter j corresponde a <diagonal atras-izquierda>
      walk(B00101000, B00101000, 30000, 14000, direcciones[j], j); // Avanzar al siguiente nodo
    }
  }
  //ft=millis()-ft;
  //Serial.println(ft);     // 
  for(int i=0;i<4;i++){     // Al final, para cada contador de sensores
    Serial1.print(dpuls[i]);// Imprime los contadores por tiempo de muestreo
    Serial1.print("   ");            
    if(i==3){               // Si ya se imprimio el ultimo encoder
      Serial1.println(" "); // Se cambia de linea
    }//*/
    pulses[i]=0;            // Se reinicia para el siguiente seguimiento
    cumError[i]=0;          // Se reinicial para el siguiente seguimiento
    lastError[i]=0;         // Se reinicia para el siguiente seguimiento
  }
  off();                    // Se detienen los motores al final de la ruta
}

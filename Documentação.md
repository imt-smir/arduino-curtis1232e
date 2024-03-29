# Comunicação entre o Arduino e o driver Curtis 1232e utilizando o protocolo CanOpen
## Ligação entre o Módulo CAN e o driver: 

Em um barramento CAN dois ou mais dispositivos são ligados em paralelo em uma rede na qual em seus 2 extremos temos resistores de 120 Ohms. Um desses fios do barramento é chamado de CAN HIGH e o outro de CAN LOW. 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem1.png)

Em nosso caso só temos 2 dispositivos em nossa rede, então será necessário colocar esse resistor nos 2, por sorte tanto o Módulo CAN quanto o Driver possuem um resistor de 120 Ohms integrado que pode ser acionado caso seja necessário. 

No Módulo CAN precisamos conectar entre si esses 2 pinos: 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem2.png)

Já no caso do Driver temos que conectar entre si os 2 pinos marcados em verde (CAN Term H e CAN Term L) 

Em amarelo temos os pinos CAN H e CAN L 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem3.png)

Na imagem abaixo podemos ver a localização dos pinos no conector do driver 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem4.png)

## Programação do Arduino: 

O protocolo de comunicação utilizado pelo driver é o CANopen, que possui um frame com a mesma quantidade de bits do que o frame de CAN Bus, porém alguns desses bits tem significados diferentes nos 2 protocolos. 

Em CANbus temos um ID da mensagem CAN de 11 bits, em CANopen chamamos esses 11 bits de COB-ID. Os 7 bits menos significativos são o ID, ou seja, o endereço do módulo (ou nó) do dispositivo para o qual a mensagem é direcionada. Os outros 4 bits são o código da função e serão vistos mais a diante. 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem5.png)

Em CANopen temos 2 tipos de mensagens: SDO e PDO, porém para comunicar com o Driver usamos apenas SDO. 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem6.png)

Uma mensagem CANbus possui, como dito anteriormente, 11 bits para o ID e possui também 8 bytes para dados, em CANopen os 4 primeiros bytes não são para dados e sim para comando e para índice e sub-índice, sendo os 4 últimos bytes para dados efetivamente, como podemos observar na imagem abaixo. 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem7.png)

Esse índice e sub-índice servem para definir qual parâmetro do dicionário do driver vamos acessar, como nos exemplos abaixo: 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem8.png)
![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem9.png)

Para exemplificar vamos adquirir a informação de temperatura do motor: 

Para o byte de comando (byte 0) temos as seguintes tabelas: 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem10.png)
![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem11.png)

O módulo CAN é o cliente, logo, temos que usar a tabela de cima, e nós queremos fazer a leitura de uma informação, logo o byte de comando é **40h**. 

Com relação ao ID CANBus da mensagem, temos que achar o código da função, como queremos receber uma informação via SDO de acordo com a tabela 1.2 ele vale **1100** em binário, além disso precisamos saber o ID ou endereço do driver, que no nosso caso é **39** em decimal, agora precisamos juntar esses 2 valores em binário para formar o ID CANbus de 11 bits. 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem13.png)
    
![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem12.png)

Agora precisamos definir os bytes 1, 2 e 3 de acordo com o índice e sub-índice, para a leitura de temperatura eles valem: 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem14.png)

Os 2 bytes do índice são invertidos fazendo com que o primeiro que aparece seja o byte 2 e o segundo seja o byte 1, o sub-índice é o 3. 

A nossa mensagem CAN ficou da seguinte forma: 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem15.png)

**ID = 627h** 

**Byte 0 = 40h** 

**Byte 1 = 0Bh** 

**Byte 2 = 32h**

**Byte 3 = 00**

**Byte 4 = 00**

**Byte 5 = 00**

**Byte 6 = 00**

**Byte 7 = 00**

Como estamos requisitando uma informação, os bytes de dados ficam vazios. 

No código do Arduino o frame fica da seguinte forma: 

```cpp
canMsg1.can_id  = 0x627; // ID CANbus
canMsg1.can_dlc = 8;     // Tamanho da mensagem em bytes
canMsg1.data[0] = 0x40;  // Controle
canMsg1.data[1] = 0x0B;  // Segundo byte do indice
canMsg1.data[2] = 0x32;  // Primeiro byte do indice
canMsg1.data[3] = 0x00;  // sub-indice
canMsg1.data[4] = 0x00;  //
canMsg1.data[5] = 0x00;  //
canMsg1.data[6] = 0x00;  //
canMsg1.data[7] = 0x00;  //
```

A variável: 

**canMsg1.can_dlc = 8;** 

representa quantos bytes de dados serão enviados na mensagem CAN, por padrão, mantemos sempre o máximo que é 8. 

Após enviarmos obtivemos o seguinte retorno: 

**5A7 8 42 B 32 0 E1 0 0 0**

Agora vamos analisar o primeiro byte de dados da resposta, que nesse caso vale E1.

Convertendo de hexadecimal para decimal temos o valor 225 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem16.png)

Pelo manual sabemos que o valor variando de -1000 a +3000 equivale a um intervalo de -100 a +300 graus celsius, então para obtermos o valor de temperatura temos que fazer 225/10 = 22,5 graus celsius. 

## Utilizando o comando VCL_Throttle

Este comando é utilizado para definir qual velocidade queremos no motor, o valor enviado pode ser entendido como uma porcentagem da velocidade máxima do motor, ou seja, o parâmetro MAX_SPEED do driver. Por exemplo, se a velocidade máxima é 1000 rpm e a porcentagem colocada é 50%, ele vai rodar a 500 rpm, o mesmo vale para valores negativos de porcentagem, como por exemplo, se for colocada uma porcentagem de -25%, ele vai girar no sentido oposto a 250rpm. 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem17.png)

A variável VCL_Throttle recebe valores de 0 a 65535 e funciona da seguinte forma: 

De 0 a 32767 representa de 0 a 100%  

De 32768 a 65535 representa de -100% a 0% 

Para representar um valor de 0 a 65535 precisamos de 2 bytes, então usaremos os bytes de dados 4 e 5 do frame, o byte 4 é o menos significativo e o 5 é o mais significativo. O próximo passo é dividir o nosso valor em 2 bytes, fizemos da seguinte forma: para obter o byte mais significativo utilizamos um while que subtrai 256 do valor até que reste um valor menor que 256, a quantidade de vezes que subtraímos representa o valor do byte mais significativo. Para o byte menos significativo fizemos o mesmo processo, porém utilizamos como valor a quantidade restante das subtrações. 

```cpp
// Separa o byte mais significativo do valor a ser enviado

byte converterMSB(long valor){
    byte byte1 = 0;
    while(valor >= 256){
      byte1++;
      valor -= 256;
    }
  return byte1;
}

// Separa o byte menos significativo do valor a ser enviado 

byte converterLSB(long valor){
   while(valor >= 256){
      valor -= 256;
    }
  return valor;
}
``` 

No setup do código os frames para envio do VCL_Throttle ficam da seguinte forma 

```cpp
VCL_ThrottleE.can_id  = 0x627; // ID do driver esquerdo
VCL_ThrottleE.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
VCL_ThrottleE.data[0] = 0x2B;  // Tipo de mensagem (40h recebe, 2Bh envia)
VCL_ThrottleE.data[1] = 0x18;  // Segundo byte do indice
VCL_ThrottleE.data[2] = 0x32;  // Primeiro byte do indice
VCL_ThrottleE.data[3] = 0x00;  // Sub-indice
VCL_ThrottleE.data[4] = 0x00;  // Segundo byte de dados
VCL_ThrottleE.data[5] = 0x00;  // Primeiro byte de dados
VCL_ThrottleE.data[6] = 0x00;  // Byte de dados não usado
VCL_ThrottleE.data[7] = 0x00;  // Byte de dados não usado

VCL_ThrottleD.can_id  = 0x626; // ID do driver direito
VCL_ThrottleD.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
VCL_ThrottleD.data[0] = 0x2B;  // Tipo de mensagem (40h recebe, 2Bh envia)
VCL_ThrottleD.data[1] = 0x18;  // Segundo byte do indice
VCL_ThrottleD.data[2] = 0x32;  // Primeiro byte do indice
VCL_ThrottleD.data[3] = 0x00;  // Sub-indice
VCL_ThrottleD.data[4] = 0x00;  // Segundo byte de dados
VCL_ThrottleD.data[5] = 0x00;  // Primeiro byte de dados
VCL_ThrottleD.data[6] = 0x00;  // Byte de dados não usado
VCL_ThrottleD.data[7] = 0x00;  // Byte de dados não usado
```

No loop os bytes 4 e 5 são atualizados de acordo com a velocidade desejada 

```cpp
if((acelerador_E > 0)&&(acelerador_E <= 100)){
    VCL_ThrottleE.data[4] = converterLSB(map(acelerador_E, 0, 100, 0, 32767));
    VCL_ThrottleE.data[5] = converterMSB(map(acelerador_E, 0, 100, 0, 32767));
}
  
  if((acelerador_E < 0)&&(acelerador_E >= -100)){
    VCL_ThrottleE.data[4] = converterLSB(map(acelerador_E, -100, 0, 32768, 65535));
    VCL_ThrottleE.data[5] = converterMSB(map(acelerador_E, -100, 0, 32768, 65535));
}

  if(acelerador_E == 0){
    VCL_ThrottleE.data[4] = converterLSB(0);
    VCL_ThrottleE.data[5] = converterMSB(0);
}
```

## Utilizando o comando Motor_RPM 

![This is an image](https://github.com/imt-smir/arduino-curtis1232e/blob/main/Imagens/Imagem18.png)

Esse comando será utilizado para obtermos o feedback da velocidade dos motores em RPM, assim como o exemplo da temperatura temos uma solicitação de uma informação, logo, precisamos enviar a mensagem de solicitação e aguardar e filtrar a resposta do sistema. As mensagens de solicitação ficam da seguinte forma:  

```cpp
MOTOR_SPEED_A_E.can_id  = 0x627; // ID do driver esquerdo
MOTOR_SPEED_A_E.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
MOTOR_SPEED_A_E.data[0] = 0x40;  // Tipo de mensagem (40h recebe, 2Bh envia)
MOTOR_SPEED_A_E.data[1] = 0x07;  // Segundo byte do indice
MOTOR_SPEED_A_E.data[2] = 0x32;  // Primeiro byte do indice
MOTOR_SPEED_A_E.data[3] = 0x00;  // Sub-indice
MOTOR_SPEED_A_E.data[4] = 0x00;  // Segundo byte de dados
MOTOR_SPEED_A_E.data[5] = 0x80;  // Primeiro byte de dados
MOTOR_SPEED_A_E.data[6] = 0x00;  // Byte de dados não usado
MOTOR_SPEED_A_E.data[7] = 0x00;  // Byte de dados não usado
  
MOTOR_SPEED_A_D.can_id  = 0x626; // ID do driver direito
MOTOR_SPEED_A_D.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
MOTOR_SPEED_A_D.data[0] = 0x40;  // Tipo de mensagem (40h recebe, 2Bh envia)
MOTOR_SPEED_A_D.data[1] = 0x07;  // Segundo byte do indice
MOTOR_SPEED_A_D.data[2] = 0x32;  // Primeiro byte do indice
MOTOR_SPEED_A_D.data[3] = 0x00;  // Sub-indice
MOTOR_SPEED_A_D.data[4] = 0x00;  // Segundo byte de dados
MOTOR_SPEED_A_D.data[5] = 0x00;  // Primeiro byte de dados
MOTOR_SPEED_A_D.data[6] = 0x00;  // Byte de dados não usado
MOTOR_SPEED_A_D.data[7] = 0x00;  // Byte de dados não usado
```

A mensagem de resposta esperada possui os mesmos bytes de dados de 0 a 3 do que a de solicitação, porém o ID é diferente, ao invés de concatenar o valor do ID do dispositivo com **b1100** em binário, ele concatena com **b1011**, logo, as mensagens esperadas tem a seguinte forma: 

**ID = 5A7h** 

**Byte 0 = 42h**

**Byte 1 = 07h**

**Byte 2 = 32h**

**Byte 3 = 00h**

**Byte 4 = Byte menos significativo da informação**

**Byte 5 = Byte mais significativo da informação**

**Byte 6 = 00h**

**Byte 7 = 00h**

A filtragem de recebimento de resposta nada mais é do que uma comparação da mensagem recebida com a mensagem esperada, na imagem abaixo podemos ver a declaração da mensagem esperada: 

```cpp
MOTOR_SPEED_A_E_recv.can_id  = 0x5A7; // ID do driver esquerdo
MOTOR_SPEED_A_E_recv.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
MOTOR_SPEED_A_E_recv.data[0] = 0x42;  // Tipo de mensagem (40h recebe, 2Bh envia)
MOTOR_SPEED_A_E_recv.data[1] = 0x07;  // Segundo byte do indice
MOTOR_SPEED_A_E_recv.data[2] = 0x32;  // Primeiro byte do indice
MOTOR_SPEED_A_E_recv.data[3] = 0x00;  // Sub-indice
  
MOTOR_SPEED_A_D_recv.can_id  = 0x5A6; // ID do driver direito
MOTOR_SPEED_A_D_recv.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
MOTOR_SPEED_A_D_recv.data[0] = 0x42;  // Tipo de mensagem (40h recebe, 2Bh envia)
MOTOR_SPEED_A_D_recv.data[1] = 0x07;  // Segundo byte do indice
MOTOR_SPEED_A_D_recv.data[2] = 0x32;  // Primeiro byte do indice
MOTOR_SPEED_A_D_recv.data[3] = 0x00;  // Sub-indice
```

Na imagem abaixo podemos ver a filtragem da mensagem por meio de comparação: 
```cpp
if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {

     // Verifica se a mensagem recebida corresponde a velocidade do motor esquerdo
     
     if ((canMsgReceive.can_id == MOTOR_SPEED_A_E_recv.can_id)&& 
       (canMsgReceive.can_dlc == MOTOR_SPEED_A_E_recv.can_dlc)&&
       (canMsgReceive.data[0] == MOTOR_SPEED_A_E_recv.data[0])&&
       (canMsgReceive.data[1] == MOTOR_SPEED_A_E_recv.data[1])&&
       (canMsgReceive.data[2] == MOTOR_SPEED_A_E_recv.data[2])&&
       (canMsgReceive.data[3] == MOTOR_SPEED_A_E_recv.data[3]))
       {
        // Une os valores dos bytes de dados 4 e 5 do frame para gerar o valor do RPM
        rpmLidoE = canMsgReceive.data[5]*256 + canMsgReceive.data[4];
      } 

      // Verifica se a mensagem recebida corresponde a velocidade do motor direito
      
      if ((canMsgReceive.can_id == MOTOR_SPEED_A_D_recv.can_id)&&
       (canMsgReceive.can_dlc == MOTOR_SPEED_A_D_recv.can_dlc)&&
       (canMsgReceive.data[0] == MOTOR_SPEED_A_D_recv.data[0])&&
       (canMsgReceive.data[1] == MOTOR_SPEED_A_D_recv.data[1])&&
       (canMsgReceive.data[2] == MOTOR_SPEED_A_D_recv.data[2])&&
       (canMsgReceive.data[3] == MOTOR_SPEED_A_D_recv.data[3]))
       {
        // Une os valores dos bytes de dados 4 e 5 do frame para gerar o valor do RPM
        rpmLidoD = canMsgReceive.data[5]*256 + canMsgReceive.data[4];
      } 
   }
```
Temos também já a lógica que une os 2 bytes de dados, o mais significativo é multiplicado por 256 e o menos significativo é apenas somado ao valor, a partir dessa conta já obtemos o RPM do motor com "sinal" 

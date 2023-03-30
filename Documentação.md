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

    Código da Função ---------------------- 1100 

    Endereço (39) ---------------------------          0100111 

    COB-ID resultante (ID CANbus) ----- 11000100111 = 627h 
